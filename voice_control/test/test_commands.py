import unittest

from g1_voice_control.commands import CommandMatcher, default_specs, normalise


class TestNormalise(unittest.TestCase):
    def test_strips_and_lowercases(self):
        self.assertEqual(normalise("  Start Grasping  "), "start grasping")

    def test_collapses_punctuation(self):
        self.assertEqual(normalise("开始，抓取。"), "开始 抓取")

    def test_fullwidth_to_halfwidth(self):
        self.assertEqual(normalise("Ｓｔａｔｕｓ"), "status")


class TestCommandMatcher(unittest.TestCase):
    def setUp(self):
        self.matcher = CommandMatcher(default_specs())

    def test_supported_phrases_match(self):
        for phrase, expected_id in [
            ("开始抓取", "grasp"),
            ("start grasping", "grasp"),
            ("开始递送", "deliver"),
            ("start delivery", "deliver"),
            ("查看状态", "status"),
            ("status", "status"),
            ("取消任务", "cancel"),
            ("cancel task", "cancel"),
            ("确认执行", "confirm"),
            ("confirm", "confirm"),
        ]:
            with self.subTest(phrase=phrase):
                result = self.matcher.match(phrase)
                self.assertTrue(result.matched, f"{phrase!r} should match")
                self.assertEqual(result.command_id, expected_id)

    def test_case_and_whitespace_insensitive(self):
        result = self.matcher.match("  START GRASPING  ")
        self.assertTrue(result.matched)
        self.assertEqual(result.command_id, "grasp")

    def test_unsupported_phrase_rejected(self):
        result = self.matcher.match("what's the weather today")
        self.assertFalse(result.matched)
        self.assertEqual(result.reason, "no_match")

    def test_negation_rejected_not_matched_as_positive(self):
        for phrase in ["不要开始抓取", "don't start grasping", "先别开始递送"]:
            with self.subTest(phrase=phrase):
                result = self.matcher.match(phrase)
                self.assertFalse(result.matched, f"{phrase!r} must not match a positive command")
                self.assertEqual(result.reason, "negated")

    def test_substring_containment_does_not_match(self):
        # Extra words around an allowlisted phrase must NOT match -- only an
        # exact (post-normalisation) utterance does.
        result = self.matcher.match("should I say 开始抓取 now")
        self.assertFalse(result.matched)
        self.assertEqual(result.reason, "no_match")

    def test_ambiguous_phrase_not_matched_to_two_commands(self):
        # Constructing a matcher with a phrase shared by two commands must
        # fail fast at construction time, not silently pick one.
        from g1_voice_control.commands import CommandSpec

        with self.assertRaises(ValueError):
            CommandMatcher(
                [
                    CommandSpec("grasp", ("go",)),
                    CommandSpec("deliver", ("go",)),
                ]
            )

    def test_empty_transcript_no_match(self):
        result = self.matcher.match("   ")
        self.assertFalse(result.matched)
        self.assertEqual(result.reason, "no_match")


if __name__ == "__main__":
    unittest.main()
