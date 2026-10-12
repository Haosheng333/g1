"""Offline tests of the tactile adapter: disabled by default, generic when on."""
import math
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from handover import tactile_adapter as ta  # noqa: E402


class _Msg(object):
    def __init__(self, **kw):
        self.__dict__.update(kw)


class DisabledByDefault(unittest.TestCase):
    """No tactile topic exists in this workspace, so the default must be off."""

    def test_default_is_disabled(self):
        self.assertFalse(ta.AdapterConfig().enabled)

    def test_disabled_adapter_reports_one_clear_blocker(self):
        problems = ta.AdapterConfig().validate()
        self.assertEqual(len(problems), 1)
        blocker = problems[0]
        self.assertIn("disabled", blocker)
        self.assertIn("hand owner", blocker)
        self.assertIn("message type", blocker)

    def test_blocker_description_mentions_the_config_file(self):
        self.assertIn("auto_release.yaml", ta.AdapterConfig().describe_blocker())

    def test_disabled_adapter_blocks_even_when_fully_configured(self):
        """Filling the fields in is not enough; enabling is a separate decision."""
        config = ta.AdapterConfig(
            enabled=False, topic="/hand_left/tactile",
            message_type="std_msgs/Float32MultiArray", value_field="data",
        )
        self.assertEqual(len(config.validate()), 1)
        self.assertIn("disabled", config.validate()[0])


class EnabledConfigValidation(unittest.TestCase):
    def _config(self, **kw):
        base = dict(
            enabled=True, topic="/hand_left/tactile",
            message_type="std_msgs/Float32MultiArray", value_field="data", reduction="mean",
        )
        base.update(kw)
        return ta.AdapterConfig(**base)

    def test_fully_specified_config_is_accepted(self):
        self.assertEqual(self._config().validate(), [])

    def test_missing_topic_is_named(self):
        self.assertIn("topic is empty", "; ".join(self._config(topic="").validate()))

    def test_missing_message_type_is_named(self):
        self.assertIn("message_type is empty", "; ".join(self._config(message_type="").validate()))

    def test_malformed_message_type_is_named(self):
        for bad in ("Float32MultiArray", "a/b/c", "/Msg", "pkg/"):
            problems = "; ".join(self._config(message_type=bad).validate())
            self.assertIn("pkg/Msg", problems, bad)

    def test_missing_value_field_is_named(self):
        self.assertIn("value_field is empty", "; ".join(self._config(value_field="").validate()))

    def test_unknown_reduction_is_named(self):
        self.assertIn("reduction must be one of", "; ".join(self._config(reduction="median").validate()))

    def test_reductions_are_the_documented_set(self):
        self.assertEqual(ta.REDUCTIONS, ("mean", "max", "min", "sum", "first", "norm"))


class FieldExtraction(unittest.TestCase):
    def test_flat_field(self):
        self.assertEqual(ta.get_field(_Msg(data=3.0), "data"), 3.0)

    def test_nested_dotted_field(self):
        msg = _Msg(wrench=_Msg(force=_Msg(z=-1.5)))
        self.assertEqual(ta.get_field(msg, "wrench.force.z"), -1.5)

    def test_absent_field_raises_keyerror_naming_the_path(self):
        with self.assertRaises(KeyError) as ctx:
            ta.get_field(_Msg(data=1.0), "wrench.force.z")
        self.assertIn("wrench.force.z", str(ctx.exception))

    def test_empty_path_raises(self):
        with self.assertRaises(KeyError):
            ta.get_field(_Msg(data=1.0), "")


class Reductions(unittest.TestCase):
    def test_scalar_passes_through_every_reduction(self):
        for reduction in ta.REDUCTIONS:
            self.assertEqual(ta.reduce_value(4.0, reduction), 4.0 if reduction != "norm" else 4.0)

    def test_sequence_reductions(self):
        values = [1.0, 2.0, 3.0, 4.0]
        self.assertEqual(ta.reduce_value(values, "mean"), 2.5)
        self.assertEqual(ta.reduce_value(values, "max"), 4.0)
        self.assertEqual(ta.reduce_value(values, "min"), 1.0)
        self.assertEqual(ta.reduce_value(values, "sum"), 10.0)
        self.assertEqual(ta.reduce_value(values, "first"), 1.0)
        self.assertAlmostEqual(ta.reduce_value([3.0, 4.0], "norm"), 5.0)

    def test_tuple_is_accepted(self):
        self.assertEqual(ta.reduce_value((1.0, 3.0), "mean"), 2.0)

    def test_empty_sequence_raises(self):
        with self.assertRaises(ValueError):
            ta.reduce_value([], "mean")

    def test_non_finite_raises(self):
        for bad in ([1.0, float("nan")], [float("inf")], float("nan")):
            with self.assertRaises(ValueError):
                ta.reduce_value(bad, "mean")

    def test_bool_is_rejected_as_a_measurement(self):
        with self.assertRaises(TypeError):
            ta.reduce_value(True, "mean")

    def test_non_numeric_raises(self):
        with self.assertRaises(TypeError):
            ta.reduce_value("1.0, 2.0", "mean")

    def test_unknown_reduction_raises(self):
        with self.assertRaises(ValueError):
            ta.reduce_value([1.0], "median")


class ExtractScalar(unittest.TestCase):
    def test_end_to_end_with_a_sequence_field(self):
        config = ta.AdapterConfig(
            enabled=True, topic="/t", message_type="std_msgs/Float32MultiArray",
            value_field="data", reduction="mean",
        )
        self.assertEqual(ta.extract_scalar(_Msg(data=[2.0, 4.0]), config), 3.0)

    def test_end_to_end_with_a_nested_scalar_field(self):
        config = ta.AdapterConfig(
            enabled=True, topic="/t", message_type="geometry_msgs/WrenchStamped",
            value_field="wrench.force.z", reduction="first",
        )
        msg = _Msg(wrench=_Msg(force=_Msg(z=-7.25)))
        self.assertEqual(ta.extract_scalar(msg, config), -7.25)


class MessageClassResolution(unittest.TestCase):
    def test_malformed_type_raises_before_importing_anything(self):
        for bad in ("Float32", "a/b/c"):
            with self.assertRaises(ValueError):
                ta.resolve_message_class(bad)

    def test_resolves_from_a_stub_package(self):
        pkg = types.ModuleType("fake_msgs")
        msg = types.ModuleType("fake_msgs.msg")
        msg.Touch = type("Touch", (object,), {})
        pkg.msg = msg
        sys.modules["fake_msgs"] = pkg
        sys.modules["fake_msgs.msg"] = msg
        self.addCleanup(sys.modules.pop, "fake_msgs", None)
        self.addCleanup(sys.modules.pop, "fake_msgs.msg", None)
        self.assertIs(ta.resolve_message_class("fake_msgs/Touch"), msg.Touch)

    def test_unknown_message_in_a_known_package_raises_valueerror(self):
        pkg = types.ModuleType("fake2_msgs")
        msg = types.ModuleType("fake2_msgs.msg")
        pkg.msg = msg
        sys.modules["fake2_msgs"] = pkg
        sys.modules["fake2_msgs.msg"] = msg
        self.addCleanup(sys.modules.pop, "fake2_msgs", None)
        self.addCleanup(sys.modules.pop, "fake2_msgs.msg", None)
        with self.assertRaises(ValueError):
            ta.resolve_message_class("fake2_msgs/Nope")


class ShippedConfigIsSafe(unittest.TestCase):
    """The checked-in YAML must not be armable."""

    def _yaml(self):
        import yaml

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config",
                            "auto_release.yaml")
        with open(path) as f:
            return yaml.safe_load(f)

    def test_tactile_is_disabled_in_the_shipped_config(self):
        self.assertFalse(self._yaml()["tactile"]["enabled"])

    def test_hardware_thresholds_are_unset_in_the_shipped_config(self):
        data = self._yaml()
        self.assertIsNone(data["release_delta"], "a shipped threshold would be a guess")
        self.assertIsNone(data["direction"], "a shipped direction would be a guess")

    def test_shipped_config_cannot_be_armed(self):
        from handover.release_monitor import MonitorConfig, ReleaseMonitor

        data = self._yaml()
        monitor = ReleaseMonitor(
            config=MonitorConfig(
                release_delta=data["release_delta"],
                direction=data["direction"],
                baseline_samples=data["baseline_samples"],
                debounce_samples=data["debounce_samples"],
                min_hold_s=data["min_hold_s"],
                sample_timeout_s=data["sample_timeout_s"],
                monitor_timeout_s=data["monitor_timeout_s"],
            )
        )
        outcome = monitor.arm(0.0)
        self.assertFalse(outcome.ok)
        self.assertIn("NOT_READY", outcome.reason)


if __name__ == "__main__":
    unittest.main()
