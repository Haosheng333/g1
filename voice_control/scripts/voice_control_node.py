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

Real dispatch is OFF by default (dry_run_backend: true in
config/commands.yaml) and this node is not referenced by system_bringup's
full_system.launch, so bringing the rest of the system up does not start
voice control.
"""

from __future__ import annotations

import os
import threading

import rospy

from g1_voice_control.asr_adapter import (
    MissingDependencyError,
    VoskMicrophoneAdapter,
    WavFileAdapter,
)
from g1_voice_control.commands import CommandMatcher, default_specs
from g1_voice_control.dispatch import Dispatcher, RecordingDryRunClient
from g1_voice_control.orchestrator_client import RosRouteClient, StatusReader, load_commands_config


class VoiceControlNode:
    def __init__(self) -> None:
        rospy.init_node("voice_control_node")

        commands_config_path = rospy.get_param(
            "~commands_config", os.path.join(os.path.dirname(__file__), "..", "config", "commands.yaml")
        )
        cfg = load_commands_config(commands_config_path)

        dry_run_backend = rospy.get_param("~dry_run_backend", True)
        if dry_run_backend:
            rospy.logwarn(
                "voice_control: dry_run_backend=true -- recognised commands will be matched and "
                "confirmed, but NO orchestrator service will be called. Set ~dry_run_backend:=false "
                "to enable real dispatch (it still respects the orchestrator's own enable_motion gate "
                "and preflight -- this flag only controls whether THIS node calls the service at all)."
            )
            route_client = RecordingDryRunClient()
            self._status_reader = None  # no ROS status subscription needed in pure dry-run mode
        else:
            route_client = RosRouteClient(service_timeout_s=rospy.get_param("~service_timeout_s", 10.0))
            self._status_reader = StatusReader()

        self._matcher = CommandMatcher(default_specs())
        self._dispatcher = Dispatcher(
            route_client=route_client,
            clock=rospy.get_time,
            confirmation_window_sec=cfg.confirmation_window_sec,
            target_resolver=lambda command_id: cfg.targets.get(command_id),
        )
        self._dispatch_lock = threading.Lock()  # serialises confirm-triggered calls, not the mic thread
        self._stop = threading.Event()

        mode = rospy.get_param("~input_mode", "microphone")
        model_path = rospy.get_param("~vosk_model_path", "")
        if not model_path:
            raise MissingDependencyError(
                "~vosk_model_path is not set -- point it at a downloaded Vosk model directory "
                "(scripts/download_model.sh), e.g. models/vosk-model-small-cn-0.22"
            )

        if mode == "microphone":
            # The launch file's default is "" (empty string), since roslaunch
            # <param> has no concept of None; treat "" the same as unset.
            device_param = rospy.get_param("~input_device", "")
            device = int(device_param) if str(device_param).strip() != "" else None
            sample_rate = rospy.get_param("~sample_rate", 16000)
            self._adapter = VoskMicrophoneAdapter(model_path, sample_rate=sample_rate, device=device)
        elif mode == "wav":
            wav_path = rospy.get_param("~wav_path", "")
            if not wav_path:
                raise MissingDependencyError("~input_mode is 'wav' but ~wav_path is not set")
            self._adapter = WavFileAdapter(model_path, wav_path)
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

        # grasp/deliver/confirm/cancel all go through the state machine. A
        # task-start or confirm call may block on a real service call, so run
        # it off the audio thread -- the mic must stay responsive per spec.
        thread = threading.Thread(target=self._dispatch_one, args=(match.command_id,), daemon=True)
        thread.start()

    def _dispatch_one(self, command_id: str) -> None:
        with self._dispatch_lock:
            event = self._dispatcher.handle_command(command_id)
        rospy.loginfo("voice_control: %s -> %s: %s", command_id, event.kind, event.detail)
        if event.route_result is not None and event.route_result.blockers:
            rospy.loginfo("voice_control: blockers: %s", "; ".join(event.route_result.blockers))

    def _report_status(self) -> None:
        if self._status_reader is None:
            rospy.loginfo("voice_control: dry_run_backend=true, no orchestrator status subscription")
            return
        rospy.loginfo("voice_control: orchestrator status: %s", self._status_reader.latest())

    def _shutdown(self) -> None:
        self._stop.set()
        self._adapter.close()

    def spin(self) -> None:
        self._adapter.run(on_final=self._on_final_transcript, should_stop=lambda: self._stop.is_set() or rospy.is_shutdown())


def main() -> None:
    node = VoiceControlNode()
    node.spin()


if __name__ == "__main__":
    main()
