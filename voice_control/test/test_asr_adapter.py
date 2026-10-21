import queue
import unittest

import g1_voice_control.asr_adapter as asr_adapter


class TestFinalResultFiltering(unittest.TestCase):
    def test_adapter_exposes_a_testable_final_result_filter(self):
        self.assertTrue(
            hasattr(asr_adapter, "emit_final_result"),
            "ASR final/partial filtering must be testable without Vosk or a microphone",
        )

    def test_partial_result_is_not_forwarded(self):
        class PartialRecognizer:
            def AcceptWaveform(self, audio):
                return False

            def Result(self):
                raise AssertionError("Result must not be requested for a partial")

        transcripts = []
        completed = asr_adapter.emit_final_result(PartialRecognizer(), b"audio", transcripts.append)
        self.assertFalse(completed)
        self.assertEqual(transcripts, [])

    def test_final_result_is_forwarded_once(self):
        class FinalRecognizer:
            def AcceptWaveform(self, audio):
                return True

            def Result(self):
                return '{"text": "确认执行"}'

        transcripts = []
        completed = asr_adapter.emit_final_result(FinalRecognizer(), b"audio", transcripts.append)
        self.assertTrue(completed)
        self.assertEqual(transcripts, ["确认执行"])


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class TestFreshAudioQueue(unittest.TestCase):
    def test_queue_is_bounded_and_keeps_newest_audio(self):
        clock = FakeClock()
        audio = asr_adapter.FreshAudioQueue(max_blocks=2, max_age_sec=1.0, clock=clock)
        audio.put(b"oldest")
        audio.put(b"middle")
        audio.put(b"newest")
        self.assertEqual(audio.get(timeout=0.0), b"middle")
        self.assertEqual(audio.get(timeout=0.0), b"newest")

    def test_stale_audio_is_discarded(self):
        clock = FakeClock()
        audio = asr_adapter.FreshAudioQueue(max_blocks=2, max_age_sec=0.5, clock=clock)
        audio.put(b"old")
        clock.now = 1.0
        with self.assertRaises(queue.Empty):
            audio.get(timeout=0.0)

    def test_clear_removes_accumulated_audio(self):
        clock = FakeClock()
        audio = asr_adapter.FreshAudioQueue(max_blocks=2, max_age_sec=1.0, clock=clock)
        audio.put(b"pending")
        audio.clear()
        with self.assertRaises(queue.Empty):
            audio.get(timeout=0.0)


if __name__ == "__main__":
    unittest.main()
