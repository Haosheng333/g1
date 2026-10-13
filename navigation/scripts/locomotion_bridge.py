#!/usr/bin/env python3
"""locomotion_bridge - /locomotion/cmd_vel  ->  the Unitree G1 walking controller (LocoClient.Move).

Why it exists: cmd_vel_gate ends at /locomotion/cmd_vel, but nothing in the repo turned that topic into
walking.  In the simulator sim_world consumes it; on the robot THIS node does.

Safety layers (every one is independent, any one of them stops the robot):
  1. navigation_node  only publishes while a goal is active
  2. cmd_vel_gate     disabled until `rosservice call /navigation/enable_motion "data: true"`; clamps; 0.5 s watchdog
  3. THIS node        dry_run by default (logs, sends nothing); clamps again; acceleration limit; 0.4 s watchdog
                      -> StopMove(); non-finite input -> StopMove(); 3 failed SDK calls -> latched fault + StopMove()
  4. the robot        velocity set by Move() expires on its own after ~1 s unless refreshed
  5. the operator     hand-held remote / e-stop always wins

It never changes the robot's mode (no Start / Damp / SetFsmId): put the G1 into walking mode with the remote first.
While idle it sends NOTHING, so it does not fight the hand-held remote.  It sends StopMove() once when motion ends.

    rosrun g1_navigation locomotion_bridge.py _network_interface:=enp2s0 _dry_run:=false

Params: ~dry_run (True)  ~network_interface ("")  ~in_topic (/locomotion/cmd_vel)  ~rate (20)  ~timeout (0.4)
        ~max_vx/vy/wz (0.5/0.3/1.0)  ~acc_vx/vy/wz (0.6/0.4/1.5)  ~max_failures (3)
"""
import math
import sys


def _clamp(v, lim):
    return max(-lim, min(lim, v))


class LocoCore:
    """All decisions, no ROS.  `client` needs move(vx, vy, wz) -> int|None and stop() -> int|None."""

    def __init__(self, client, max_v=(0.5, 0.3, 1.0), acc=(0.6, 0.4, 1.5), timeout=0.4, max_failures=3):
        self.client = client
        self.max_v, self.acc = tuple(max_v), tuple(acc)
        self.timeout = float(timeout)
        self.max_failures = int(max_failures)
        self.cur = [0.0, 0.0, 0.0]
        self.moving = False
        self.failures = 0
        self.faulted = False

    def _call(self, fn, *args):
        try:
            code = fn(*args)
            ok = code in (None, 0)
        except Exception:  # noqa: BLE001 - any SDK/DDS error counts as a failed call
            ok = False
        self.failures = 0 if ok else self.failures + 1
        if self.failures >= self.max_failures:
            self.faulted = True
        return ok

    def _stop(self):
        self.cur = [0.0, 0.0, 0.0]
        if self.moving:
            self.moving = False
            self._call(self.client.stop)

    def update(self, cmd, age, dt):
        """cmd = (vx, vy, wz) or None; age = seconds since it arrived; dt = seconds since the last update.
        Returns what was commanded this tick (vx, vy, wz)."""
        if self.faulted:
            if self.moving:
                self.moving = False
                try:
                    self.client.stop()
                except Exception:  # noqa: BLE001
                    pass
            self.cur = [0.0, 0.0, 0.0]
            return (0.0, 0.0, 0.0)
        if cmd is None or age > self.timeout or not all(math.isfinite(c) for c in cmd):
            self._stop()
            return (0.0, 0.0, 0.0)
        tgt = [_clamp(c, m) for c, m in zip(cmd, self.max_v)]
        if all(abs(t) < 1e-3 for t in tgt):
            self._stop()                                  # an explicit zero command: stop immediately
            return (0.0, 0.0, 0.0)
        for i in range(3):                                 # slew-limit toward the target
            step = self.acc[i] * max(dt, 0.0)
            self.cur[i] += _clamp(tgt[i] - self.cur[i], step)
        self.moving = True
        self._call(self.client.move, *self.cur)
        return tuple(self.cur)


class DryRunClient:
    """Logs what WOULD be sent.  The default: nothing reaches the robot."""

    def __init__(self, log):
        self.log, self.n = log, 0

    def move(self, vx, vy, wz):
        self.n += 1
        if self.n % 20 == 1:
            self.log("[locomotion_bridge] DRY RUN move vx=%.2f vy=%.2f wz=%.2f (nothing sent)" % (vx, vy, wz))
        return 0

    def stop(self):
        self.log("[locomotion_bridge] DRY RUN stop (nothing sent)")
        return 0


class DdsLocoClient:
    """Real robot: unitree_sdk2py high-level LocoClient (the walking controller's own API)."""

    def __init__(self, iface):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        if iface:
            ChannelFactoryInitialize(0, iface)
        else:
            ChannelFactoryInitialize(0)
        self.c = LocoClient()
        self.c.SetTimeout(5.0)
        self.c.Init()

    def move(self, vx, vy, wz):
        return self.c.Move(vx, vy, wz)      # continuous_move=False: the robot drops it after ~1 s unless refreshed

    def stop(self):
        return self.c.StopMove()


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    rospy.init_node("locomotion_bridge")
    p = rospy.get_param
    dry = bool(p("~dry_run", True))
    if dry:
        client = DryRunClient(rospy.loginfo)
    else:
        try:
            client = DdsLocoClient(p("~network_interface", ""))
        except ImportError as exc:
            rospy.logfatal("[locomotion_bridge] dry_run:=false needs unitree_sdk2py in THIS python (%s)", exc)
            sys.exit(2)
        except Exception as exc:  # noqa: BLE001
            rospy.logfatal("[locomotion_bridge] cannot reach the G1 loco service (%s). Wrong network_interface, "
                           "robot off, or not in a mode that accepts it? Refusing to start.", exc)
            sys.exit(2)
    core = LocoCore(client,
                    max_v=(float(p("~max_vx", 0.5)), float(p("~max_vy", 0.3)), float(p("~max_wz", 1.0))),
                    acc=(float(p("~acc_vx", 0.6)), float(p("~acc_vy", 0.4)), float(p("~acc_wz", 1.5))),
                    timeout=float(p("~timeout", 0.4)), max_failures=int(p("~max_failures", 3)))
    state = {"cmd": None, "t": 0.0}

    def cb(msg):
        state["cmd"] = (msg.linear.x, msg.linear.y, msg.angular.z)
        state["t"] = rospy.Time.now().to_sec()

    rospy.Subscriber(p("~in_topic", "/locomotion/cmd_vel"), Twist, cb, queue_size=5)
    pub = rospy.Publisher("/locomotion/bridge_state", String, queue_size=1, latch=True)
    pub.publish(String(data="dry_run" if dry else "live"))
    (rospy.logwarn if not dry else rospy.loginfo)("[locomotion_bridge] %s", "LIVE: the robot will walk" if not dry
                                                  else "dry run: nothing is sent to the robot")
    rate = rospy.Rate(float(p("~rate", 20.0)))
    last = rospy.Time.now().to_sec()
    announced = False
    rospy.on_shutdown(lambda: core.update(None, 1e9, 0.0))
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        core.update(state["cmd"], now - state["t"], now - last)
        last = now
        if core.faulted and not announced:
            announced = True
            pub.publish(String(data="FAULT"))
            rospy.logfatal("[locomotion_bridge] %d consecutive SDK failures: stopped and LATCHED. Restart the node "
                           "after checking the robot.", core.max_failures)
        rate.sleep()


if __name__ == "__main__":
    main()
