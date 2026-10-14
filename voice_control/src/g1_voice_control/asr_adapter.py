"""ASR adapters: microphone/WAV audio in, final transcripts out.

Vosk and sounddevice are imported lazily (inside __init__/methods, not at
module level) so importing this module -- and therefore commands.py and
dispatch.py's test suite, which does not import this module at all, plus
anything that imports the package's __init__ -- never requires audio
dependencies to be installed. `Adapter` is the seam a second model (Whisper,
etc.) would implement without touching orchestrator integration.
"""

from __future__ import annotations

import json
import wave
from abc import ABC, abstractmethod
from typing import Callable, Iterator, Optional

# Final-result callback type: str transcript -> None
TranscriptCallback = Callable[[str], None]


class Adapter(ABC):
    """Produces FINAL recognition results only -- partials are never surfaced
    to commands.py, per the spec ("use final recognition results only")."""

    @abstractmethod
    def run(self, on_final: TranscriptCallback, should_stop: Callable[[], bool]) -> None:
        """Blocks, calling on_final(text) for each final result, until
        should_stop() returns True."""

    @abstractmethod
    def close(self) -> None:
        ...


class MissingDependencyError(RuntimeError):
    """Raised with a specific missing package/model/device name, per the
    spec's "report missing dependencies, models or microphone access
    clearly" -- never a bare traceback from deep inside a third-party lib."""


class VoskMicrophoneAdapter(Adapter):
    """Live microphone -> Vosk -> final transcripts.

    Model weights are NEVER downloaded here; `model_path` must already exist
    on disk (see scripts/download_model.sh and config/vosk.yaml). Device
    selection is explicit (index or None for the system default); use
    scripts/list_microphones.py to find an index.
    """

    def __init__(self, model_path: str, sample_rate: int = 16000, device: Optional[int] = None):
        try:
            import vosk
        except ImportError as exc:
            raise MissingDependencyError(
                "the 'vosk' package is not installed -- pip install vosk (see requirements.txt)"
            ) from exc
        try:
            import sounddevice as sd
        except ImportError as exc:
            raise MissingDependencyError(
                "the 'sounddevice' package (or its PortAudio dependency) is not installed -- "
                "pip install sounddevice; on Linux you may also need `apt install portaudio19-dev`"
            ) from exc

        import os

        if not os.path.isdir(model_path):
            raise MissingDependencyError(
                f"Vosk model not found at {model_path!r} -- run scripts/download_model.sh first "
                "(models are never downloaded automatically)"
            )

        self._vosk = vosk
        self._sd = sd
        self.sample_rate = int(sample_rate)
        self.device = device
        self._model = vosk.Model(model_path)
        self._recognizer = vosk.KaldiRecognizer(self._model, self.sample_rate)
        self._stream = None

        # Surface a clear error now if the requested device cannot be opened,
        # rather than failing silently inside the audio callback thread.
        try:
            sd.check_input_settings(device=device, samplerate=self.sample_rate, channels=1)
        except Exception as exc:
            raise MissingDependencyError(
                f"microphone device {device!r} is not usable at {self.sample_rate} Hz / 1 channel: {exc}. "
                "Run scripts/list_microphones.py to see available devices."
            ) from exc

    def run(self, on_final: TranscriptCallback, should_stop: Callable[[], bool]) -> None:
        blocksize = int(self.sample_rate * 0.2)  # 200ms chunks

        def callback(indata, frames, time_info, status):  # noqa: ARG001 - sounddevice signature
            if status:
                pass  # xruns etc. are not fatal; Vosk tolerates gaps
            if self._recognizer.AcceptWaveform(bytes(indata)):
                result = json.loads(self._recognizer.Result())
                text = result.get("text", "").strip()
                if text:
                    on_final(text)

        self._stream = self._sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=blocksize,
            device=self.device,
            dtype="int16",
            channels=1,
            callback=callback,
        )
        with self._stream:
            while not should_stop():
                self._sd.sleep(100)

    def close(self) -> None:
        if self._stream is not None:
            self._stream.close()


class WavFileAdapter(Adapter):
    """Vosk over a WAV file -- for testing without live hardware. Requires
    16-bit mono PCM at the recognizer's sample rate (the format Vosk's own
    examples use); raises MissingDependencyError with a clear message for
    anything else rather than mis-decoding it."""

    def __init__(self, model_path: str, wav_path: str, sample_rate: int = 16000):
        try:
            import vosk
        except ImportError as exc:
            raise MissingDependencyError(
                "the 'vosk' package is not installed -- pip install vosk (see requirements.txt)"
            ) from exc

        import os

        if not os.path.isdir(model_path):
            raise MissingDependencyError(f"Vosk model not found at {model_path!r}")
        if not os.path.isfile(wav_path):
            raise MissingDependencyError(f"WAV file not found at {wav_path!r}")

        wf = wave.open(wav_path, "rb")
        if wf.getnchannels() != 1 or wf.getsampwidth() != 2:
            raise MissingDependencyError(
                f"{wav_path!r} must be 16-bit mono PCM; got {wf.getnchannels()} channel(s), "
                f"{wf.getsampwidth() * 8}-bit"
            )
        self._wf = wf
        self._model = vosk.Model(model_path)
        self._recognizer = vosk.KaldiRecognizer(self._model, wf.getframerate())

    def run(self, on_final: TranscriptCallback, should_stop: Callable[[], bool]) -> None:
        chunk = 4000
        while not should_stop():
            data = self._wf.readframes(chunk)
            if len(data) == 0:
                break
            if self._recognizer.AcceptWaveform(data):
                result = json.loads(self._recognizer.Result())
                text = result.get("text", "").strip()
                if text:
                    on_final(text)
        final = json.loads(self._recognizer.FinalResult())
        text = final.get("text", "").strip()
        if text:
            on_final(text)

    def close(self) -> None:
        self._wf.close()


class ScriptedAdapter(Adapter):
    """Replays a fixed list of transcripts with no audio/model at all.

    For the offline test suite and for anyone wiring up a non-Vosk ASR later
    without installing Vosk just to exercise dispatch.py end to end."""

    def __init__(self, transcripts: Iterator[str]):
        self._transcripts = list(transcripts)

    def run(self, on_final: TranscriptCallback, should_stop: Callable[[], bool]) -> None:
        for text in self._transcripts:
            if should_stop():
                return
            on_final(text)

    def close(self) -> None:
        pass
