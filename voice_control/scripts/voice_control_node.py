#!/usr/bin/env python3
"""ROS1 node: spoken commands as an additional frontend to the orchestrator.

Pipeline: mic/WAV (asr_adapter) -> final transcript -> utterance assembly
(end word "over") -> command match (commands.py: exact phrases, then the slot
grammar for the fetch-and-deliver task) -> confirm/dispatch state machine
(dispatch.py) -> one g1_voice_control/TaskCommand on /voice/command
(orchestrator_client.py). The orchestrator subscribes to that topic, runs the
route, and answers on /orchestrator/command_result. Typed control
(rosservice call /orchestrator/run_*) is untouched.

    "go to the desk, pick up a can, go to the chair, and pass me, over"
        -> fetch_deliver target=table_01 destination=chair_01 label=bottle
        -> (confirm) -> /voice/command -> orchestrator

UTTERANCE ASSEMBLY. Vosk emits a final result at every pause, so a spoken
sentence with commas arrives as several results. Each result is first tried
as an exact short command (status / cancel / confirm act at once). Otherwise
it is appended to an utterance buffer; the buffer is parsed as ONE command
when an end word ("over") is heard, and discarded, with a log line, if no end
word arrives within ~utterance_timeout_sec. ~require_end_word:=false parses
every final result on its own instead.

Real dispatch is OFF by default (dry_run_backend: true in the launch files).
system_bringup's launches include this node only when voice_control:=true is
passed explicitly, so the default bringup does not start voice control.
"""

from __future__ import annotations

import os
import threading

import rospy

from g1_voice_control.asr_adapter import (
    MissingDependencyError,
    ScriptedAdapter,
    VoskMicrophoneAdapter,
    WavFileAdapter,
    load_vosk_config,
)
from g1_voice_control.configuration import load_commands_config
from g1_voice_control.dispatch import Dispatcher, RecordingDryRunClient
from g1_voice_control.orchestrator_client import StatusReader, TopicCommandClient
from g1_voice_control.utterance import UtteranceBuffer


class VoiceControlNode:
    def __init__(self) -> None:
        rospy.init_node("voice_control_node")

        commands_config_path = rospy.get_param(
            "~commands_config", os.path.join(os.path.dirname(__file__), "..", "config", "commands.yaml")
        )
        cfg = load_commands_config(commands_config_path)
        # A launch arg may override the YAML's confirmation mode when non-empty.
        mode = str(rospy.get_param("~confirmation_mode", "") or "").strip() or cfg.confirmation_mode
        if mode not in ("confirm", "cancel_window"):
            raise ValueError("~confirmation_mode must be 'confirm' or 'cancel_window'")

        # Status is read-only and remains available in dry-run mode. Dry-run
        # suppresses task-execution service calls, not diagnostics.
        self._status_reader = StatusReader()
        dry_run_backend = rospy.get_param("~dry_run_backend", True)
        if dry_run_backend:
            rospy.logwarn(
                "voice_control: dry_run_backend=true -- recognised commands will be matched and "
                "confirmed, but NOTHING is published to the orchestrator. Set ~dry_run_backend:=false "
                "to publish confirmed tasks on /voice/command (the orchestrator's own enable_motion "
                "and preflight still decide whether anything moves)."
            )
            route_client = RecordingDryRunClient()
        else:
            route_client = TopicCommandClient(
                subscriber_wait_s=float(rospy.get_param("~service_wait_timeout_s", 10.0)))

        self._matcher = cfg.matcher()
        self._parser = self._matcher.parser
        if self._parser is None:
            rospy.logwarn("voice_control: no lexicon configured; only the exact phrases are recognised")
        else:
            rospy.loginfo("voice_control: slot grammar loaded (%d words); places=%s",
                          len(self._parser.vocabulary()), sorted(set(self._parser.places.values())))
        if mode == "cancel_window":
            rospy.logwarn(
                "voice_control: confirmation_mode=cancel_window -- a heard task DISPATCHES when the "
                "%.0fs window closes unless 'cancel task' is said. Silence means GO.",
                cfg.confirmation_window_sec)
        self._dispatcher = Dispatcher(
            route_client=route_client,
            clock=rospy.get_time,
            confirmation_window_sec=cfg.confirmation_window_sec,
            target_resolver=lambda command_id: cfg.targets.get(command_id),
            auto_dispatch_after_window=(mode == "cancel_window"),
        )
        self._require_end_word = bool(rospy.get_param("~require_end_word", True)) and self._parser is not None
        self._buffer = UtteranceBuffer(self._parser, rospy.get_time,
                                       float(rospy.get_param("~utterance_timeout_sec", 15.0))) \
            if self._parser is not None else None
        self._expiry_timer = rospy.Timer(rospy.Duration(0.1), self._poll_expiry)
        self._stop = threading.Event()

        input_mode = rospy.get_param("~input_mode", "microphone")
        if input_mode == "scripted":
            # Debug aid: replay transcripts with no audio, model or microphone,
            # so the match -> confirm -> dispatch path can be exercised in a
            # terminal. Never a substitute for real ASR.
            scripted = rospy.get_param("~scripted_transcripts", [])
            if isinstance(scripted, str):
                scripted = [part.strip() for part in scripted.split("|") if part.strip()]
            rospy.logwarn("voice_control: input_mode=scripted, replaying %d transcript(s); no audio",
                          len(scripted))
            self._adapter = ScriptedAdapter(scripted)
            rospy.on_shutdown(self._shutdown)
            return

        vosk_config_path = rospy.get_param(
            "~vosk_config", os.path.join(os.path.dirname(__file__), "..", "config", "vosk.yaml")
        )
        asr_cfg = load_vosk_config(vosk_config_path)
        model_path = rospy.get_param("~vosk_model_path", "") or asr_cfg.model_path
        language = rospy.get_param("~language", "") or asr_cfg.language
        if language not in ("zh-cn", "en-us"):
            raise ValueError("~language must be exactly 'zh-cn' or 'en-us'")
        sample_rate_param = rospy.get_param("~sample_rate", "")
        sample_rate = int(sample_rate_param) if str(sample_rate_param).strip() else asr_cfg.sample_rate
        grammar = self._grammar(cfg) if bool(rospy.get_param("~use_grammar", True)) else None
        rospy.loginfo("voice_control: ASR language=%s model=%s sample_rate=%s grammar=%s",
                      language, model_path, sample_rate,
                      "%d words" % len(grammar) if grammar else "off (free dictation)")

        if input_mode == "microphone":
            # The launch file's default is "" (empty string), since roslaunch
            # <param> has no concept of None; treat "" the same as unset.
            device_param = rospy.get_param("~input_device", "")
            device = int(device_param) if str(device_param).strip() != "" else None
            latency = rospy.get_param("~stream_latency_sec", "")
            latency = float(latency) if str(latency).strip() else None
            self._adapter = VoskMicrophoneAdapter(model_path, sample_rate=sample_rate, device=device,
                                                  grammar=grammar, latency_sec=latency)
            rospy.loginfo("voice_control: microphone device %r opened at %d Hz -> %d Hz for the model, "
                          "latency %.2fs", device, self._adapter.device_rate, sample_rate,
                          self._adapter.latency_sec)
        elif input_mode == "wav":
            wav_path = rospy.get_param("~wav_path", "")
            if not wav_path:
                raise MissingDependencyError("~input_mode is 'wav' but ~wav_path is not set")
            self._adapter = WavFileAdapter(model_path, wav_path, sample_rate=sample_rate, grammar=grammar)
        else:
            raise ValueError(f"~input_mode must be 'microphone', 'wav' or 'scripted', got {input_mode!r}")

        rospy.on_shutdown(self._shutdown)

    def _grammar(self, cfg):
        """The recogniser vocabulary: lexicon words plus every exact phrase."""
        words = []
        if self._parser is not None:
            words += self._parser.vocabulary()
        for spec in cfg.command_specs:
            for phrase in spec.phrases:
                words += phrase.lower().split()
        return words

    # --- transcripts ----------------------------------------------------------------
    def _on_final_transcript(self, text: str) -> None:
        rospy.loginfo("voice_control: heard %r", text)
        # Short exact commands act immediately, end word or not.
        exact = self._matcher.match_exact(text)
        if exact.matched:
            self._handle_match(exact)
            return
        if exact.reason == "negated":
            rospy.loginfo("voice_control: ignored (negated)")
            return
        if not self._require_end_word or self._buffer is None:
            self._handle_match(self._matcher.match(text))
            return
        complete, dropped = self._buffer.push(text)
        if dropped:
            rospy.logwarn("voice_control: dropped unfinished utterance %r (no end word)", dropped)
        if complete is None:
            rospy.loginfo("voice_control: waiting for the end word %s; so far %r",
                          self._parser.end_words, self._buffer.text)
            return
        rospy.loginfo("voice_control: utterance complete: %r", complete)
        self._handle_match(self._matcher.match(complete))

    def _handle_match(self, match) -> None:
        if not match.matched:
            rospy.loginfo("voice_control: not a supported command (%s)%s", match.reason,
                          (" | understood: " + match.summary) if match.summary else "")
            return
        if match.summary:
            rospy.loginfo("voice_control: %s", match.summary)
        if match.command_id == "status":
            self._report_status()
            return

        # State transitions are deliberately synchronous and non-blocking, so
        # final transcripts are handled in recognition order. Confirmation
        # creates a single-use action; only the service call goes to a worker.
        slots = dict(match.slots)
        if match.command_id in ("fetch_deliver",):
            slots["summary"] = match.summary
        event = self._dispatcher.handle_command(match.command_id, slots)
        self._log_dispatch_event(match.command_id, event)
        self._start_action(event)

    def _start_action(self, event) -> None:
        if event is not None and event.dispatch_action is not None:
            thread = threading.Thread(target=self._execute_action, args=(event.dispatch_action,), daemon=True)
            thread.start()

    def _execute_action(self, action) -> None:
        event = self._dispatcher.execute_action(action)
        self._log_dispatch_event(action.command_id, event)

    def _poll_expiry(self, _timer_event) -> None:
        if self._buffer is not None:
            dropped = self._buffer.expire()
            if dropped:
                rospy.logwarn("voice_control: dropped unfinished utterance %r (no end word)", dropped)
        event = self._dispatcher.poll_expiry()
        if event is not None:
            self._log_dispatch_event("confirmation", event)
            self._start_action(event)

    def _on_audio_fault(self, detail: str) -> None:
        if self._buffer is not None:
            self._buffer.clear()
        event = self._dispatcher.handle_fault(detail)
        rospy.logerr("voice_control: audio -> %s: %s", event.kind, event.detail)

    @staticmethod
    def _log_dispatch_event(command_id: str, event) -> None:
        rospy.loginfo("voice_control: %s -> %s: %s", command_id, event.kind, event.detail)
        if event.route_result is not None and event.route_result.blockers:
            rospy.loginfo("voice_control: blockers: %s", "; ".join(event.route_result.blockers))

    def _report_status(self) -> None:
        rospy.loginfo("voice_control: orchestrator status: %s", self._status_reader.latest())

    def _shutdown(self) -> None:
        self._stop.set()
        self._adapter.close()

    def spin(self) -> None:
        self._adapter.run(
            on_final=self._on_final_transcript,
            should_stop=lambda: self._stop.is_set() or rospy.is_shutdown(),
            on_fault=self._on_audio_fault,
        )
        if isinstance(self._adapter, (ScriptedAdapter, WavFileAdapter)):
            # Scripted / WAV input ends; keep spinning so a confirmation
            # window and an in-flight dispatch can finish and be logged.
            rospy.loginfo("voice_control: input exhausted; spinning for timers and dispatch")
            rospy.spin()


def main() -> None:
    node = VoiceControlNode()
    node.spin()


if __name__ == "__main__":
    main()
