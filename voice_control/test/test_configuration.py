import os
import math
import tempfile
import unittest

import yaml

from g1_voice_control.configuration import load_commands_config
import g1_voice_control.asr_adapter as asr_adapter


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestCommandsConfiguration(unittest.TestCase):
    def test_shipped_config_has_no_unverified_dispatch_targets(self):
        config = load_commands_config(os.path.join(PACKAGE_ROOT, "config", "commands.yaml"))
        self.assertIsNone(config.targets["grasp"])
        self.assertIsNone(config.targets["deliver"])

    def test_shipped_config_provides_configurable_command_specs(self):
        config = load_commands_config(os.path.join(PACKAGE_ROOT, "config", "commands.yaml"))
        phrases = {spec.command_id: spec.phrases for spec in config.command_specs}
        self.assertIn("开始抓取", phrases["grasp"])
        self.assertIn("start grasping", phrases["grasp"])
        self.assertIn("确认执行", phrases["confirm"])

    def test_vosk_config_is_loaded_explicitly(self):
        self.assertTrue(hasattr(asr_adapter, "load_vosk_config"))
        config = asr_adapter.load_vosk_config(os.path.join(PACKAGE_ROOT, "config", "vosk.yaml"))
        self.assertEqual(config.language, "en-us")
        self.assertEqual(config.sample_rate, 16000)
        self.assertIn("vosk-model-small-en-us-0.15", config.model_path)

    def test_shipped_config_loads_the_lexicon_and_defaults_to_explicit_confirm(self):
        config = load_commands_config(os.path.join(PACKAGE_ROOT, "config", "commands.yaml"))
        self.assertIsNotNone(config.lexicon)
        self.assertEqual(config.confirmation_mode, "confirm")
        parser = config.task_parser()
        self.assertEqual(parser.places["desk"], "table_01")
        self.assertEqual(parser.places["chair"], "chair_01")
        self.assertEqual(parser.objects["can"], "bottle")
        self.assertIn("over", parser.end_words)

    def test_an_unknown_confirmation_mode_is_rejected(self):
        raw = {
            "confirmation_window_sec": 8.0, "confirmation_mode": "yolo",
            "valid_grasp_targets": [], "grasp_target": None,
            "valid_deliver_targets": [], "deliver_target": None,
            "phrases": {"grasp": ["g"], "deliver": ["d"], "status": ["s"],
                        "cancel": ["c"], "confirm": ["k"]},
        }
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", encoding="utf-8") as stream:
            yaml.safe_dump(raw, stream)
            stream.flush()
            with self.assertRaises(ValueError):
                load_commands_config(stream.name)

    def test_a_lexicon_without_places_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            lex = os.path.join(tmp, "lexicon.yaml")
            with open(lex, "w", encoding="utf-8") as fh:
                yaml.safe_dump({"verb_classes": {"navigate": ["go"]}}, fh)
            raw = {
                "confirmation_window_sec": 8.0, "lexicon": "lexicon.yaml",
                "valid_grasp_targets": [], "grasp_target": None,
                "valid_deliver_targets": [], "deliver_target": None,
                "phrases": {"grasp": ["g"], "deliver": ["d"], "status": ["s"],
                            "cancel": ["c"], "confirm": ["k"]},
            }
            path = os.path.join(tmp, "commands.yaml")
            with open(path, "w", encoding="utf-8") as fh:
                yaml.safe_dump(raw, fh)
            with self.assertRaises(ValueError):
                load_commands_config(path)

    def test_duplicate_phrase_across_commands_is_rejected_at_load_time(self):
        raw = {
            "confirmation_window_sec": 8.0,
            "valid_grasp_targets": [],
            "grasp_target": None,
            "valid_deliver_targets": [],
            "deliver_target": None,
            "phrases": {
                "grasp": ["go"],
                "deliver": ["go"],
                "status": ["status"],
                "cancel": ["cancel"],
                "confirm": ["confirm"],
            },
        }
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", encoding="utf-8") as stream:
            yaml.safe_dump(raw, stream, allow_unicode=True)
            stream.flush()
            with self.assertRaises(ValueError):
                load_commands_config(stream.name)

    def test_confirmation_window_must_be_finite(self):
        raw = {
            "valid_grasp_targets": [], "grasp_target": None,
            "valid_deliver_targets": [], "deliver_target": None,
            "phrases": {
                "grasp": ["grasp"], "deliver": ["deliver"],
                "status": ["status"], "cancel": ["cancel"],
                "confirm": ["confirm"],
            },
        }
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                raw["confirmation_window_sec"] = value
                with tempfile.NamedTemporaryFile("w", suffix=".yaml", encoding="utf-8") as stream:
                    yaml.safe_dump(raw, stream)
                    stream.flush()
                    with self.assertRaises(ValueError):
                        load_commands_config(stream.name)


if __name__ == "__main__":
    unittest.main()
