#!/usr/bin/env python3
"""
hand_node.py - Module 3.2: thin std_srvs/Trigger services over the hand backend.

One instance per hand side. One Trigger service per preset in config/finger_presets.yaml:
    /hand_{side}/pre_grasp   open / pre-shape the fingers
    /hand_{side}/grasp       close the fingers (caller retries 3 x 0.4 s)
    /hand_{side}/release     open to release
(pre_grasp / grasp / release are required; extra presets in the YAML become extra services.)

Real hardware:  roslaunch hand_control hand.launch side:=left
Simulation:     roslaunch hand_control hand.launch side:=left sim:=true render:=true

Design notes
  * The node never moves the hand on startup or shutdown: restarting the node while the hand is
    holding a bottle must not drop it.
  * Commands are serialized with a lock; a service never raises - failures come back as
    success=False + message, so the orchestrator's retry logic works.
  * If a command fails on the real hand, the node reconnects once and retries once.
"""

import asyncio
import concurrent.futures
import os
import threading
import time
from typing import Optional

import rospy
from std_srvs.srv import Trigger, TriggerResponse

from hand_control.finger_presets import FingerPose, load_presets
from hand_control.revo_hand_link import RevoHandLink

try:
    from hand_control.hand_sim_link import (
        RevoHandSimLink, real_hand_joint_names, real_hand_actuator_names, real_hand_finger_bodies,
    )
except ImportError:  # mujoco not installed - fine unless ~sim:=true
    RevoHandSimLink = None


class HandTimeout(RuntimeError):
    pass


class AsyncLoopThread:
    """One asyncio event loop on its own thread, so rospy's synchronous service callbacks can
    drive the async SDK."""

    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run, name="hand-asyncio", daemon=True)

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def start(self):
        self._thread.start()

    def run_coro(self, coro, timeout: Optional[float] = None):
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        try:
            return future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            future.cancel()
            raise HandTimeout("hand operation timed out after %.1fs" % (timeout or 0.0))

    def stop(self):
        self.loop.call_soon_threadsafe(self.loop.stop)


def _package_path() -> str:
    try:
        import rospkg
        return rospkg.RosPack().get_path("hand_control")
    except Exception:
        return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


class HandNode:
    def __init__(self):
        rospy.init_node("hand_node")

        self._side = rospy.get_param("~side", "left")
        if self._side not in ("left", "right"):
            raise ValueError("~side must be 'left' or 'right', got %r" % (self._side,))
        self._use_sim = rospy.get_param("~sim", False)
        self._reconnect_on_error = rospy.get_param("~reconnect_on_error", True)
        presets_yaml = rospy.get_param("~presets_yaml", "") or None

        self._presets = load_presets(presets_yaml)  # raises on a bad YAML -> node refuses to start
        self._cmd_lock = threading.Lock()           # one hand command at a time

        self._async = AsyncLoopThread()
        self._async.start()
        self._link = self._make_link()
        self._connect_with_retries(int(rospy.get_param("~connect_retries", 3)))

        self._services = []
        for preset_name in self._presets:
            self._services.append(
                rospy.Service("/hand_%s/%s" % (self._side, preset_name), Trigger,
                              self._make_handler(preset_name)))

        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("[hand_%s] ready: %s", self._side, ", ".join(self._presets))

    # ------------------------------------------------------------------ setup
    def _make_link(self):
        side = self._side
        if self._use_sim:
            if RevoHandSimLink is None:
                raise RuntimeError("~sim:=true but mujoco / hand_control.hand_sim_link is not importable "
                                   "(pip3 install mujoco)")
            with_bottle = rospy.get_param("~with_bottle", False)
            model_file = "%s_hand_scene.xml" % side if with_bottle else "%s_hand.xml" % side
            default_model = os.path.join(_package_path(), "config", "revo2_hand_real", model_file)
            model_path = rospy.get_param("~mujoco_model", "") or default_model
            render = rospy.get_param("~render", True)
            rospy.loginfo("[hand_%s] SIM MODE: MuJoCo (%s, render=%s)", side, model_path, render)
            return RevoHandSimLink(
                model_path=model_path,
                joint_names=real_hand_joint_names(side),
                actuator_names=real_hand_actuator_names(side),
                finger_bodies=real_hand_finger_bodies(side),
                render=render,
            )
        port = rospy.get_param("~com_port", "") or None   # empty = auto-detect
        return RevoHandLink(side=side, port=port)

    def _connect_with_retries(self, retries: int):
        last = None
        for attempt in range(1, max(1, retries) + 1):
            try:
                self._async.run_coro(self._link.connect(), timeout=15.0)
                rospy.loginfo("[hand_%s] connected (%s)", self._side, "sim" if self._use_sim else "real Revo2")
                return
            except Exception as exc:
                last = exc
                rospy.logwarn("[hand_%s] connect attempt %d/%d failed: %s", self._side, attempt, retries, exc)
                time.sleep(1.0)
        rospy.logfatal("[hand_%s] cannot connect to the hand: %s", self._side, last)
        raise RuntimeError("cannot connect to the hand: %s" % (last,))

    # ------------------------------------------------------------------ services
    def _make_handler(self, preset_name: str):
        pose = self._presets[preset_name]

        def _handler(_req):
            with self._cmd_lock:
                try:
                    ok, msg = self._execute_with_retry(preset_name, pose)
                except Exception as exc:  # a service callback must never raise
                    rospy.logerr("[hand_%s] %s FAILED: %s", self._side, preset_name, exc)
                    return TriggerResponse(success=False, message="%s: %s" % (preset_name, exc))
            (rospy.loginfo if ok else rospy.logwarn)("[hand_%s] %s -> %s", self._side, preset_name, msg)
            return TriggerResponse(success=ok, message=msg)

        return _handler

    def _execute_with_retry(self, name: str, pose: FingerPose):
        try:
            return self._execute(name, pose)
        except Exception as exc:
            if self._use_sim or not self._reconnect_on_error or not hasattr(self._link, "reconnect"):
                raise
            rospy.logwarn("[hand_%s] %s failed (%s) - reconnecting and retrying once", self._side, name, exc)
            self._async.run_coro(self._link.reconnect(), timeout=15.0)
            return self._execute(name, pose)

    def _execute(self, name: str, pose: FingerPose):
        move_timeout = pose.ramp_steps * pose.ramp_dt + 3.0
        self._async.run_coro(self._link.write_pose(pose), timeout=move_timeout)

        settled = self._async.run_coro(self._link.wait_until_settled(pose), timeout=pose.settle_timeout_s + 1.0)
        feedback = self._async.run_coro(self._link.read_positions(), timeout=2.0)

        if pose.require_settle and not settled:
            return False, "%s: fingers did not reach target within %.1fs (feedback=%s, target=%s)" % (
                name, pose.settle_timeout_s, feedback, pose.positions)

        if pose.expect_contact:
            # Closed direction = increasing value. A finger stopped short of its target is touching something.
            blocked = [i for i, (f, t) in enumerate(zip(feedback, pose.positions))
                       if t - f > pose.settle_tolerance]
            if len(blocked) < pose.min_contact_fingers:
                return False, "%s: nothing in hand - fingers closed to target (feedback=%s)" % (name, feedback)
            return True, "%s: holding (fingers %s stopped short, feedback=%s)" % (name, blocked, feedback)

        if settled:
            return True, "%s reached" % name
        return True, "%s commanded, did not settle (feedback=%s) - expected if holding an object" % (name, feedback)

    # ------------------------------------------------------------------ shutdown
    def _shutdown(self):
        # Deliberately does NOT open the hand: a restart must not drop a held bottle.
        try:
            self._async.run_coro(self._link.close(), timeout=2.0)
        except Exception:
            pass
        self._async.stop()


if __name__ == "__main__":
    HandNode()
    rospy.spin()
