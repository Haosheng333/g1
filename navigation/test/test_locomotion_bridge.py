"""locomotion_bridge safety logic (no ROS, no robot).

    cd navigation/test && python3 -m unittest test_locomotion_bridge -v

Pins: nothing is sent when idle, a stale command stops the robot, clamping, acceleration limit,
non-finite input, latched fault after repeated SDK failures, dry-run default.
"""
import math
import os
import re
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))

from locomotion_bridge import DryRunClient, LocoCore  # noqa: E402


class Fake:
    def __init__(self, code=0, boom=False):
        self.calls, self.code, self.boom = [], code, boom

    def move(self, vx, vy, wz):
        self.calls.append(("move", vx, vy, wz))
        if self.boom:
            raise RuntimeError("dds down")
        return self.code

    def stop(self):
        self.calls.append(("stop",))
        return 0


def run(core, cmd, age=0.0, n=1, dt=0.05):
    out = None
    for _ in range(n):
        out = core.update(cmd, age, dt)
    return out


class TestLocoCore(unittest.TestCase):
    def test_idle_sends_nothing(self):
        c = Fake()
        core = LocoCore(c)
        run(core, None, n=50)
        run(core, (0.2, 0, 0), age=5.0, n=50)       # stale from the start
        self.assertEqual(c.calls, [])               # must not fight the hand-held remote

    def test_stale_command_stops_once(self):
        c = Fake()
        core = LocoCore(c, timeout=0.4)
        run(core, (0.2, 0.0, 0.0), n=20)
        run(core, (0.2, 0.0, 0.0), age=0.6, n=20)
        self.assertEqual(c.calls.count(("stop",)), 1)
        self.assertEqual(c.calls[-1], ("stop",))
        self.assertEqual(core.cur, [0.0, 0.0, 0.0])

    def test_explicit_zero_stops_immediately(self):
        c = Fake()
        core = LocoCore(c)
        run(core, (0.2, 0.0, 0.0), n=10)
        self.assertEqual(run(core, (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        self.assertEqual(c.calls[-1], ("stop",))

    def test_clamp_and_ramp(self):
        c = Fake()
        core = LocoCore(c, max_v=(0.5, 0.3, 1.0), acc=(0.6, 0.4, 1.5))
        first = run(core, (5.0, -5.0, 5.0), dt=0.05)
        self.assertAlmostEqual(first[0], 0.03, 6)          # 0.6 m/s^2 * 0.05 s
        self.assertAlmostEqual(first[1], -0.02, 6)
        self.assertAlmostEqual(first[2], 0.075, 6)
        final = run(core, (5.0, -5.0, 5.0), n=200, dt=0.05)
        self.assertEqual(final, (0.5, -0.3, 1.0))          # clamped, never above the limits
        for call in c.calls:
            self.assertLessEqual(abs(call[1]), 0.5 + 1e-9)
            self.assertLessEqual(abs(call[2]), 0.3 + 1e-9)
            self.assertLessEqual(abs(call[3]), 1.0 + 1e-9)

    def test_step_never_exceeds_acceleration(self):
        c = Fake()
        core = LocoCore(c, acc=(0.6, 0.4, 1.5))
        prev = 0.0
        for _ in range(60):
            vx = run(core, (0.3, 0.0, 0.0), dt=0.05)[0]
            self.assertLessEqual(vx - prev, 0.6 * 0.05 + 1e-9)
            prev = vx

    def test_non_finite_input_stops(self):
        for bad in (float("nan"), float("inf")):
            c = Fake()
            core = LocoCore(c)
            run(core, (0.2, 0.0, 0.0), n=5)
            self.assertEqual(run(core, (bad, 0.0, 0.0)), (0.0, 0.0, 0.0))
            self.assertEqual(c.calls[-1], ("stop",))

    def test_repeated_failures_latch_a_fault(self):
        for c in (Fake(code=7), Fake(boom=True)):
            core = LocoCore(c, max_failures=3)
            run(core, (0.2, 0.0, 0.0), n=10)
            self.assertTrue(core.faulted)
            n = len(c.calls)
            run(core, (0.2, 0.0, 0.0), n=10)                # latched: no further move commands
            self.assertEqual([x for x in c.calls[n:] if x[0] == "move"], [])

    def test_single_failure_recovers(self):
        c = Fake()
        core = LocoCore(c, max_failures=3)
        c.code = 7
        run(core, (0.2, 0.0, 0.0), n=2)
        c.code = 0
        run(core, (0.2, 0.0, 0.0), n=5)
        self.assertFalse(core.faulted)

    def test_resumes_after_stop(self):
        c = Fake()
        core = LocoCore(c)
        run(core, (0.2, 0.0, 0.0), n=10)
        run(core, None, n=3)
        out = run(core, (0.2, 0.0, 0.0), dt=0.05)
        self.assertAlmostEqual(out[0], 0.03, 6)             # ramps from zero again, no jump


class TestDefaults(unittest.TestCase):
    def test_dry_run_client_sends_nothing(self):
        logs = []
        d = DryRunClient(logs.append)
        self.assertEqual(d.move(0.1, 0, 0), 0)
        self.assertEqual(d.stop(), 0)
        self.assertTrue(all("nothing sent" in m for m in logs))

    def test_node_defaults_to_dry_run(self):
        src = open(os.path.join(REPO, "navigation", "scripts", "locomotion_bridge.py")).read()
        self.assertRegex(src, r'p\("~dry_run", True\)')
        self.assertIsNone(re.search(r"\.(Start|Damp|SetFsmId)\(", src))

    def test_launch_defaults_are_safe(self):
        text = open(os.path.join(REPO, "system_bringup", "launch", "navigation.launch")).read()
        self.assertIn('<arg name="locomotion" default="false"/>', text)
        self.assertIn('<arg name="locomotion_dry_run" default="true"/>', text)
        self.assertIn('<arg name="motion_enabled" default="false"/>', text)

    def test_limits_not_wider_than_the_gate(self):
        gate = open(os.path.join(REPO, "navigation", "scripts", "cmd_vel_gate.py")).read()
        bridge = open(os.path.join(REPO, "navigation", "scripts", "locomotion_bridge.py")).read()
        for name, val in (("max_vx", "0.5"), ("max_vy", "0.3"), ("max_wz", "1.0")):
            self.assertIn('"~%s", %s' % (name, val), gate)
            self.assertIn('"~%s", %s' % (name, val), bridge)


if __name__ == "__main__":
    unittest.main()
