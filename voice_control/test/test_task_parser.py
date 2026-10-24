"""The slot grammar: the spoken fetch-and-deliver task, and what it refuses.

Offline, no ROS, no Vosk. Uses the shipped config/lexicon.yaml so the tests
pin what the robot will actually hear.
"""
import os
import unittest

import yaml

from g1_voice_control.commands import CommandMatcher, TaskParser, default_specs
from g1_voice_control.utterance import UtteranceBuffer

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

with open(os.path.join(PACKAGE_ROOT, "config", "lexicon.yaml"), encoding="utf-8") as _fh:
    LEXICON = yaml.safe_load(_fh)

EXPECTED = {"target": "table_01", "label": "bottle", "destination": "chair_01"}


class TestTheSpokenTask(unittest.TestCase):
    def setUp(self):
        self.parser = TaskParser(LEXICON)
        self.matcher = CommandMatcher(default_specs(), self.parser)

    def test_the_target_sentence_with_punctuation(self):
        r = self.matcher.match("go to the desk, pick up a can, go to the chair, and pass me, over")
        self.assertTrue(r.matched, r.reason)
        self.assertEqual(r.command_id, "fetch_deliver")
        self.assertEqual(r.slots, EXPECTED)
        self.assertIn("table_01", r.summary)

    def test_the_target_sentence_as_vosk_emits_it(self):
        """No punctuation, lower case, end word glued on."""
        r = self.matcher.match("go to the desk pick up a can go to the chair and pass me over")
        self.assertTrue(r.matched, r.reason)
        self.assertEqual(r.slots, EXPECTED)

    def test_synonyms_and_the_wake_word(self):
        for text in (
            "lucy go to the table grab a bottle then go to the chair and hand it to me over",
            "walk to the desk take the can walk to the seat and give it to me",
            "please go to the table, pick the cup, head to the chair and hand over",
        ):
            with self.subTest(text=text):
                r = self.matcher.match(text)
                self.assertTrue(r.matched, "%s: %s" % (text, r.reason))
                self.assertEqual(r.slots, EXPECTED)

    def test_the_priors_fill_in_the_places_main_process_fills_in(self):
        for text in ("bring a can to me over", "pick up a can and pass me", "bring me a can"):
            with self.subTest(text=text):
                r = self.matcher.match(text)
                self.assertTrue(r.matched, "%s: %s" % (text, r.reason))
                self.assertEqual(r.slots, EXPECTED)

    def test_can_you_is_a_filler_not_a_drink(self):
        r = self.matcher.match("can you go to the desk pick up a can go to the chair and pass me over")
        self.assertTrue(r.matched, r.reason)
        self.assertEqual(r.slots, EXPECTED)

    def test_other_shapes_are_understood_but_not_dispatched(self):
        for text, shape in (
            ("go to the chair over", "navigate"),
            ("pick up a can over", "navigate -> grasp"),
            ("go to the desk and pick up a can over", "navigate -> grasp"),
            ("pass me over", "navigate -> handover"),
        ):
            with self.subTest(text=text):
                r = self.matcher.match(text)
                self.assertFalse(r.matched)
                self.assertTrue(r.reason.startswith("unsupported_plan"), r.reason)
                self.assertIn(shape, r.reason)
                self.assertTrue(r.summary)      # the operator sees what was understood

    def test_a_negated_sentence_is_rejected_before_parsing(self):
        r = self.matcher.match("don't go to the desk pick up a can go to the chair and pass me over")
        self.assertFalse(r.matched)
        self.assertEqual(r.reason, "negated")

    def test_unrelated_speech_is_no_match(self):
        for text in ("what's the weather today", "", "hello there", "over"):
            with self.subTest(text=text):
                r = self.matcher.match(text)
                self.assertFalse(r.matched)
                self.assertEqual(r.reason, "no_match")

    def test_exact_phrases_still_win(self):
        for text, cmd in (("confirm", "confirm"), ("cancel task", "cancel"), ("status", "status"),
                          ("start grasping", "grasp")):
            r = self.matcher.match(text)
            self.assertTrue(r.matched)
            self.assertEqual(r.command_id, cmd)
            self.assertEqual(r.slots, {})

    def test_an_unknown_place_cannot_be_invented(self):
        """A place not in the lexicon is skipped; the parser never makes one up."""
        r = self.matcher.match("go to the kitchen pick up a can go to the chair and pass me over")
        # "kitchen" is unknown -> the first navigate has no place -> the object
        # prior supplies table_01; the task still only names lexicon waypoints.
        self.assertTrue(r.matched, r.reason)
        self.assertEqual(r.slots["target"], "table_01")

    def test_the_grammar_vocabulary_covers_the_sentence(self):
        words = set(self.parser.vocabulary())
        for w in "go to the desk pick up a can go to the chair and pass me over".split():
            self.assertIn(w, words, w)
        self.assertNotIn("", words)

    def test_end_word_detection(self):
        self.assertTrue(self.parser.has_end_word("and pass me over"))
        self.assertTrue(self.parser.has_end_word("over"))
        self.assertFalse(self.parser.has_end_word("go over to the desk"))
        self.assertEqual(self.parser.strip_end_word("pass me, over."), ("pass me", True))

    def test_a_lexicon_without_verbs_or_places_is_refused(self):
        with self.assertRaises(ValueError):
            TaskParser({"places": {"desk": "table_01"}})
        with self.assertRaises(ValueError):
            TaskParser({"verb_classes": {"navigate": ["go"]}})


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class TestUtteranceBuffer(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.buffer = UtteranceBuffer(TaskParser(LEXICON), self.clock, timeout_sec=10.0)

    def test_pieces_are_joined_until_the_end_word(self):
        self.assertEqual(self.buffer.push("go to the desk"), (None, None))
        self.clock.t = 1.0
        self.assertEqual(self.buffer.push("pick up a can"), (None, None))
        self.clock.t = 2.0
        self.assertEqual(self.buffer.push("go to the chair"), (None, None))
        self.clock.t = 3.0
        full, dropped = self.buffer.push("and pass me over")
        self.assertEqual(full, "go to the desk pick up a can go to the chair and pass me over")
        self.assertIsNone(dropped)
        self.assertFalse(self.buffer.pending)

    def test_a_stale_half_sentence_is_dropped_not_glued(self):
        self.buffer.push("go to the desk")
        self.clock.t = 11.0
        full, dropped = self.buffer.push("pick up a can and pass me over")
        self.assertEqual(dropped, "go to the desk")
        self.assertEqual(full, "pick up a can and pass me over")

    def test_expire_reports_the_dropped_text_once(self):
        self.buffer.push("go to the desk")
        self.clock.t = 10.5
        self.assertEqual(self.buffer.expire(), "go to the desk")
        self.assertIsNone(self.buffer.expire())

    def test_a_single_complete_result_passes_straight_through(self):
        full, dropped = self.buffer.push("bring a can to me over")
        self.assertEqual(full, "bring a can to me over")

    def test_timeout_must_be_positive(self):
        with self.assertRaises(ValueError):
            UtteranceBuffer(TaskParser(LEXICON), self.clock, timeout_sec=0.0)


if __name__ == "__main__":
    unittest.main()
