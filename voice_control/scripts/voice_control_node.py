#!/usr/bin/env python3
"""ROS1 node: spoken commands as an additional frontend to the orchestrator.

Untested on real hardware/ROS -- built on a machine with no ROS, no Vosk, no
microphone, and no real robot. Verify end to end on the actual system before
relying on it; see README.md's acceptance procedure.

Pipeline: mic/WAV (asr_adapter) -> final transcript -> command match
(commands.py) -> confirm/dispatch state machine (dispatch.py) -> orchestrator
RunRoute services (orchestrator_client.py). Terminal control (rosservice
call) is untouched; this is a second, independent frontend to the same
services, not a replacement.

Real dispatch is OFF by default (dry_run_backend: true in the launch files).
system_bringup's full_system.launch includes this node only when
voice_control:=true is passed explicitly, so the default bringup does not
start voice control.
"""

from __future__ import annotations

import os
import threading

import rospy

from g1_voice_control.asr_adapter import (
    MissingDependencyError,
    VoskMicrophoneAdapter,
    WavFileAdapter,
    load_vosk_config,
)
from g1_voice_control.commands import CommandMatcher
from g1_voice_control.configuration import load_commands_config
from g1_voice_control.dispatch import Dispatcher, RecordingDryRunClient
from g1_voice_control.orchestrator_client import RosRouteClient, StatusReader


class VoiceControlNode:
    def __init__(self) -> None:
        rospy.init_node("voice_control_node")

        commands_config_path = rospy.get_param(
            "~commands_config", os.path.join(os.path.dirname(__file__), "..", "config", "commands.yaml")
        )
        cfg = load_commands_config(commands_config_path)

        # Status is read-only and remains available in dry-run mode. Dry-run
        # suppresses task-execution service calls, not diagnostics.
        self._status_reader = StatusReader()
        dry_run_backend = rospy.get_param("~dry_run_backend", True)
        if dry_run_backend:
            rospy.logwarn(
                "voice_control: dry_run_backend=true -- recognised commands will be matched and "
                "confirmed, but NO orchestrator service will be called. Set ~dry_run_backend:=false "
                "to enable real dispatch (it still respects the orchestrator's own enable_motion gate "
                "and preflight -- this flag only controls whether THIS node calls the service at all)."
            )
            route_client = RecordingDryRunClient()
        else:
            route_client = RosRouteClient(
                service_wait_timeout_s=rospy.get_param("~service_wait_timeout_s", 10.0)
            )

        self._matcher = CommandMatcher(cfg.command_specs)
        self._dispatcher = Dispatcher(
            route_client=route_client,
            clock=rospy.get_time,
            confirmation_window_sec=cfg.confirmation_window_sec,
            target_resolver=lambda command_id: cfg.targets.get(command_id),
        )
        self._expiry_timer = rospy.Timer(rospy.Duration(0.1), self._poll_expiry)
        self._stop = threading.Event()

        mode = rospy.get_param("~input_mode", "microphone")
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
        rospy.loginfo("voice_control: ASR language=%s model=%s sample_rate=%s", language, model_path, sample_rate)

        if mode == "microphone":
            # The launch file's default is "" (empty string), since roslaunch
            # <param> has no concept of None; treat "" the same as unset.
            device_param = rospy.get_param("~input_device", "")
            device = int(device_param) if str(device_param).strip() != "" else None
            self._adapter = VoskMicrophoneAdapter(model_path, sample_rate=sample_rate, device=device)
        elif mode == "wav":
            wav_path = rospy.get_param("~wav_path", "")
            if not wav_path:
                raise MissingDependencyError("~input_mode is 'wav' but ~wav_path is not set")
            self._adapter = WavFileAdapter(model_path, wav_path, sample_rate=sample_rate)
        else:
            raise ValueError(f"~input_mode must be 'microphone' or 'wav', got {mode!r}")

        rospy.on_shutdown(self._shutdown)

    def _on_final_transcript(self, text: str) -> None:
        rospy.loginfo("voice_control: heard %r", text)
        match = self._matcher.match(text)
        if not match.matched:
            rospy.loginfo("voice_control: not a supported command (%s)", match.reason)
            return

        if match.command_id == "status":
            self._report_status()
            return

        # State transitions are deliberately synchronous and non-blocking, so
        # final transcripts are handled in recognition order. Confirmation
        # creates a single-use action; only the service call goes to a worker.
        event = self._dispatcher.handle_command(match.command_id)
        self._log_dispatch_event(match.command_id, event)
        if event.dispatch_action is not None:
            thread = threading.Thread(target=self._execute_action, args=(event.dispatch_action,), daemon=True)
            thread.start()

    def _execute_action(self, action) -> None:
        event = self._dispatcher.execute_action(action)
        self._log_dispatch_event(action.command_id, event)

    def _poll_expiry(self, _timer_event) -> None:
        event = self._dispatcher.poll_expiry()
        if event is not None:
            self._log_dispatch_event("confirmation", event)

    def _on_audio_fault(self, detail: str) -> None:
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


def main() -> None:
    node = VoiceControlNode()
    node.spin()


if __name__ == "__main__":
    main()
