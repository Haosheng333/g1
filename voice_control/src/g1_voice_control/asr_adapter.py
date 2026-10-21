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
import queue
import time
import wave
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Iterator, Optional

# Final-result callback type: str transcript -> None
TranscriptCallback = Callable[[str], None]
FaultCallback = Callable[[str], None]


class FreshAudioQueue:
    """Bounded audio queue that drops oldest and stale microphone blocks."""

    def __init__(
        self,
        max_blocks: int,
        max_age_sec: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if int(max_blocks) <= 0:
            raise ValueError("max_blocks must be positive")
        if float(max_age_sec) <= 0.0:
            raise ValueError("max_age_sec must be positive")
        self._queue = queue.Queue(maxsize=int(max_blocks))
        self._max_age_sec = float(max_age_sec)
        self._clock = clock

    def put(self, audio: bytes) -> None:
        item = (self._clock(), bytes(audio))
        while True:
            try:
                self._queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass

    def get(self, timeout: float) -> bytes:
        timeout = max(0.0, float(timeout))
        deadline = self._clock() + timeout
        while True:
            remaining = max(0.0, deadline - self._clock())
            captured_at, audio = self._queue.get(timeout=remaining)
            if self._clock() - captured_at <= self._max_age_sec:
                return audio
            if timeout == 0.0 and self._queue.empty():
                raise queue.Empty

    def clear(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return


@dataclass(frozen=True)
class VoskConfig:
    language: str
    model_path: str
    sample_rate: int


def load_vosk_config(path: str) -> VoskConfig:
    """Load one explicit language/model selection from YAML."""
    import os
    import yaml

    with open(path, "r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}

    language = raw.get("language")
    if language not in ("zh-cn", "en-us"):
        raise ValueError("language must be exactly 'zh-cn' or 'en-us'")

    model_path = raw.get("model_path")
    if not isinstance(model_path, str) or not model_path.strip():
        raise ValueError("model_path must be a non-empty string")
    model_path = model_path.strip()
    if not os.path.isabs(model_path):
        package_root = os.path.dirname(os.path.dirname(os.path.abspath(path)))
        model_path = os.path.normpath(os.path.join(package_root, model_path))

    sample_rate = int(raw.get("sample_rate", 16000))
    if sample_rate <= 0:
        raise ValueError("sample_rate must be greater than zero")

    return VoskConfig(language=language, model_path=model_path, sample_rate=sample_rate)


def _emit_result_json(payload: str, on_final: TranscriptCallback) -> None:
    result = json.loads(payload)
    text = result.get("text", "").strip()
    if text:
        on_final(text)


def emit_final_result(recognizer, audio: bytes, on_final: TranscriptCallback) -> bool:
    """Feed one audio block and emit only a completed Vosk result.

    Returns True when Vosk completed an utterance. Partial results are never
    requested or forwarded, which makes this safety boundary testable without
    importing Vosk or opening an audio device.
    """
    if not recognizer.AcceptWaveform(audio):
        return False
    _emit_result_json(recognizer.Result(), on_final)
    return True


class Adapter(ABC):
    """Produces FINAL recognition results only -- partials are never surfaced
    to commands.py, per the spec ("use final recognition results only")."""

    @abstractmethod
    def run(
        self,
        on_final: TranscriptCallback,
        should_stop: Callable[[], bool],
        on_fault: Optional[FaultCallback] = None,
    ) -> None:
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
        # Five 200 ms blocks cap latency and memory at roughly one second.
        # Older audio is unsafe command input and is discarded, never replayed.
        self._audio_queue = FreshAudioQueue(max_blocks=5, max_age_sec=1.0)
        self._fault_queue = queue.Queue(maxsize=1)

        # Surface a clear error now if the requested device cannot be opened,
        # rather than failing silently inside the audio callback thread.
        try:
            sd.check_input_settings(device=device, samplerate=self.sample_rate, channels=1)
        except Exception as exc:
            raise MissingDependencyError(
                f"microphone device {device!r} is not usable at {self.sample_rate} Hz / 1 channel: {exc}. "
                "Run scripts/list_microphones.py to see available devices."
            ) from exc

    def run(
        self,
        on_final: TranscriptCallback,
        should_stop: Callable[[], bool],
        on_fault: Optional[FaultCallback] = None,
    ) -> None:
        blocksize = int(self.sample_rate * 0.2)  # 200ms chunks

        def callback(indata, frames, time_info, status):  # noqa: ARG001 - sounddevice signature
            # PortAudio callbacks must stay short and deterministic. Recognition
            # and all downstream logging/dispatch happen in run()'s thread.
            if status:
                self._audio_queue.clear()
                try:
                    self._fault_queue.put_nowait(f"microphone input fault: {status}")
                except queue.Full:
                    pass  # one pending fault already causes the required reset
                return
            self._audio_queue.put(bytes(indata))

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
                try:
                    fault = self._fault_queue.get_nowait()
                except queue.Empty:
                    fault = None
                if fault is not None and on_fault is not None:
                    on_fault(fault)
                try:
                    data = self._audio_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                emit_final_result(self._recognizer, data, on_final)

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
        if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getframerate() != int(sample_rate):
            channels = wf.getnchannels()
            sample_width = wf.getsampwidth()
            frame_rate = wf.getframerate()
            wf.close()
            raise MissingDependencyError(
                f"{wav_path!r} must be {int(sample_rate)} Hz, 16-bit mono PCM; got "
                f"{frame_rate} Hz, {channels} channel(s), {sample_width * 8}-bit"
            )
        self._wf = wf
        self._model = vosk.Model(model_path)
        self._recognizer = vosk.KaldiRecognizer(self._model, wf.getframerate())

    def run(
        self,
        on_final: TranscriptCallback,
        should_stop: Callable[[], bool],
        on_fault: Optional[FaultCallback] = None,
    ) -> None:
        chunk = 4000
        while not should_stop():
            data = self._wf.readframes(chunk)
            if len(data) == 0:
                break
            emit_final_result(self._recognizer, data, on_final)
        _emit_result_json(self._recognizer.FinalResult(), on_final)

    def close(self) -> None:
        self._wf.close()


class ScriptedAdapter(Adapter):
    """Replays a fixed list of transcripts with no audio/model at all.

    For the offline test suite and for anyone wiring up a non-Vosk ASR later
    without installing Vosk just to exercise dispatch.py end to end."""

    def __init__(self, transcripts: Iterator[str]):
        self._transcripts = list(transcripts)

    def run(
        self,
        on_final: TranscriptCallback,
        should_stop: Callable[[], bool],
        on_fault: Optional[FaultCallback] = None,
    ) -> None:
        for text in self._transcripts:
            if should_stop():
                return
            on_final(text)

    def close(self) -> None:
        pass
