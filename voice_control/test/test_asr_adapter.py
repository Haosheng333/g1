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




class TestResampler(unittest.TestCase):
    """48 kHz codec audio -> the 16 kHz the model expects, without numpy."""

    @staticmethod
    def _pcm(values):
        import array
        return array.array("h", values).tobytes()

    @staticmethod
    def _values(data):
        import array
        a = array.array("h")
        a.frombytes(data)
        return list(a)

    def test_integer_ratio_averages_each_group(self):
        from g1_voice_control.asr_adapter import Resampler
        r = Resampler(48000, 16000)
        out = self._values(r.process(self._pcm([3, 3, 3, 6, 6, 6, -9, -9, -9])))
        self.assertEqual(out, [3, 6, -9])

    def test_a_dc_level_survives_and_the_length_shrinks_by_the_ratio(self):
        from g1_voice_control.asr_adapter import Resampler
        r = Resampler(48000, 16000)
        out = self._values(r.process(self._pcm([1000] * 9600)))     # one 200 ms block
        self.assertEqual(len(out), 3200)
        self.assertTrue(all(v == 1000 for v in out))

    def test_leftover_samples_carry_into_the_next_block(self):
        from g1_voice_control.asr_adapter import Resampler
        r = Resampler(48000, 16000)
        first = self._values(r.process(self._pcm([9, 9, 9, 9])))
        second = self._values(r.process(self._pcm([9, 9])))
        self.assertEqual(first, [9])
        self.assertEqual(second, [9])

    def test_non_integer_ratio_interpolates(self):
        from g1_voice_control.asr_adapter import Resampler
        r = Resampler(44100, 16000)
        ramp = list(range(0, 4410))
        out = self._values(r.process(self._pcm(ramp)))
        self.assertAlmostEqual(len(out) / 1600.0, 1.0, delta=0.01)
        self.assertEqual(out[0], 0)
        self.assertTrue(all(b >= a for a, b in zip(out, out[1:])), "a ramp stays monotonic")

    def test_identity_passes_bytes_through(self):
        from g1_voice_control.asr_adapter import Resampler
        r = Resampler(16000, 16000)
        self.assertTrue(r.identity)
        self.assertEqual(r.process(b"\x01\x02\x03\x04"), b"\x01\x02\x03\x04")

    def test_upsampling_is_refused(self):
        from g1_voice_control.asr_adapter import Resampler
        with self.assertRaises(ValueError):
            Resampler(8000, 16000)

    def test_the_device_rate_falls_back_to_the_codec_default(self):
        """A codec refusing 16 kHz is opened at its default rate, else refused."""
        from g1_voice_control.asr_adapter import MissingDependencyError, VoskMicrophoneAdapter

        class FakeSD:
            def __init__(self, ok_rates, default=48000.0):
                self.ok, self.default = set(ok_rates), default

            def check_input_settings(self, device=None, samplerate=None, channels=1):
                if samplerate not in self.ok:
                    raise RuntimeError("Invalid sample rate")

            def query_devices(self, device=None, kind=None):
                return {"default_samplerate": self.default}

        pick = VoskMicrophoneAdapter._usable_device_rate
        self.assertEqual(pick(FakeSD({16000, 48000}), 4, 16000), 16000)
        self.assertEqual(pick(FakeSD({44100, 48000}), 4, 16000), 48000)
        with self.assertRaises(MissingDependencyError):
            pick(FakeSD({44100}), 4, 16000)
        with self.assertRaises(MissingDependencyError):
            pick(FakeSD({8000}, default=8000.0), 4, 16000)


if __name__ == "__main__":
    unittest.main()
