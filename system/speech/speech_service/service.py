#!/usr/bin/env python3
"""Speech service -- the srv-layer voice interaction service for Robonix.

Architecture position: robonix/system/speech
  - Sits ABOVE the primitive layer (audio_driver) and BELOW the application layer
  - Receives raw audio from audio_driver via gRPC, returns transcriptions
  - Receives text from applications, returns synthesized audio (MP3)

Provides 5 RPCs across 5 gRPC service contracts (from robonix_contracts.proto):

  SystemSpeechAsr (Automatic Speech Recognition -- one-shot):
    Call(req) -> resp          -- full-utterance transcription

  SystemSpeechAsrStream (Streaming ASR):
    Stream(stream) -> stream   -- real-time chunk-by-chunk transcription

  SystemSpeechTts (Text-to-Speech -- one-shot):
    Call(req) -> resp          -- returns complete MP3 audio

  SystemSpeechTtsStream (Streaming TTS):
    Stream(req) -> stream      -- yields MP3 chunks as generated

  SystemSpeechDialog (Voice Dialog Session):
    Stream(req) -> stream      -- managed session with state transitions

Backend engines:
  - Whisper (transformers pipeline)  -- one-shot ASR, GPU FP16, high accuracy
  - FunASR Paraformer-zh-streaming   -- streaming ASR, 600ms granularity
  - Edge TTS (Microsoft, free)       -- TTS synthesis, zh-CN-XiaoxiaoNeural

Audio adaptation:
  The service auto-adapts any input format (sample_rate, channels, encoding)
  to 16kHz mono pcm_s16le using speech_service.audio_utils.adapt_audio().
  Callers do NOT need to pre-process audio -- the service handles it.

Atlas integration:
  On startup, the service optionally registers with the Atlas control plane
  (RegisterNode + DeclareInterface x 5). If Atlas is unavailable, the service
  runs in standalone mode. Set SPEECH_STANDALONE=1 to skip registration.

Environment variables:
  ASR_MODEL          Whisper model path (default: openai/whisper-large-v3)
  ASR_DEVICE         Torch device: cuda | cpu (default: cuda)
  ASR_CHUNK_LENGTH   Whisper chunk length in seconds (default: 30.0)
  ASR_BATCH_SIZE     Whisper batch size (default: 4)
  FUNASR_MODEL       FunASR model name or path (default: paraformer-zh-streaming)
  FUNASR_CHUNK_SIZE  Paraformer chunk_size as JSON (default: [0,10,5])
  TTS_VOICE          Edge TTS voice name (default: zh-CN-XiaoxiaoNeural)
  ROBONIX_ATLAS      Atlas control-plane address (default: localhost:50051)
  SPEECH_PORT        gRPC listen port, 0 = auto-pick (default: 0)
  SPEECH_BIND_ADDR   gRPC bind address (default: 0.0.0.0)
  SPEECH_CI_MODE     Set to 1 for mock backends (no GPU/model needed)
"""
import io
import json
import os
import sys
import time
import uuid
import asyncio
import logging
import wave
from concurrent import futures
from pathlib import Path
from typing import Optional

logging.basicConfig(level=logging.INFO, format="[speech-service] %(levelname)s %(message)s")
log = logging.getLogger(__name__)

# -- Proto stub resolution ---------------------------------------------------
# Walks up from this file's directory looking for proto_gen/ containing the
# codegen-generated *_pb2.py / *_pb2_grpc.py files. Adds it to sys.path so
# "import asr_pb2" etc. work regardless of the working directory.

def _ensure_proto_gen() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_contracts_pb2_grpc.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent

_ensure_proto_gen()

import grpc
import asr_pb2
import tts_pb2
import speech_pb2
import robonix_msg_pb2
import robonix_contracts_pb2_grpc as contracts_grpc

# -- CI mock mode ------------------------------------------------------------

CI_MODE = os.environ.get("SPEECH_CI_MODE", "").strip() in ("1", "true", "yes")

# -- ASR Backend (Whisper) ---------------------------------------------------

class WhisperASRBackend:
    """One-shot ASR backend using HuggingFace Whisper pipeline.

    Loads a Whisper model (default: openai/whisper-large-v3) onto GPU with FP16.
    Best suited for complete utterances -- not real-time streaming.

    Pipeline:
        raw PCM bytes -> numpy float32 -> transformers pipeline -> text
        + automatic traditional-to-simplified Chinese conversion (OpenCC t2s)

    Config (environment):
        ASR_MODEL       -- local model path or HuggingFace model ID
        ASR_DEVICE      -- "cuda" or "cpu"
        ASR_CHUNK_LENGTH -- long-form audio chunk length in seconds
        ASR_BATCH_SIZE  -- batch size for pipeline inference
    """

    def __init__(self):
        import torch
        from transformers import pipeline as hf_pipeline

        model_path = os.environ.get("ASR_MODEL", "openai/whisper-large-v3")
        device = os.environ.get("ASR_DEVICE", "cuda")
        chunk_length = float(os.environ.get("ASR_CHUNK_LENGTH", "30.0"))
        batch_size = int(os.environ.get("ASR_BATCH_SIZE", "4"))

        log.info("Loading Whisper model from %s on %s ...", model_path, device)
        # `local_files_only` must be a top-level kwarg, not inside model_kwargs:
        # transformers>=5 reads it from kwargs into hub_kwargs and also forwards
        # model_kwargs to AutoConfig.from_pretrained — so duplicating it via
        # model_kwargs raises "got multiple values for keyword argument".
        self.pipe = hf_pipeline(
            "automatic-speech-recognition",
            model=model_path,
            device=device,
            torch_dtype=torch.float16,
            local_files_only=True,
        )
        self.chunk_length_s = chunk_length
        self.batch_size = batch_size
        log.info("Whisper model loaded (chunk_length=%.1fs, batch_size=%d)", chunk_length, batch_size)

    def recognize(self, audio_bytes: bytes, encoding: str, sample_rate: int, language: str) -> dict:
        """Run one-shot ASR on complete audio.

        Args:
            audio_bytes: Raw PCM audio (already adapted to 16kHz mono pcm_s16le
                by the servicer, but can handle other encodings too).
            encoding: Audio encoding string (e.g. "pcm_s16le").
            sample_rate: Sample rate of the audio data.
            language: BCP-47 language tag (reserved, Whisper auto-detects).

        Returns:
            dict with "text" (str) and "confidence" (float, fixed 0.9).
        """
        import numpy as np

        wav_data = self._to_wav_numpy(audio_bytes, encoding, sample_rate)
        gen_kwargs = {"chunk_length_s": self.chunk_length_s, "batch_size": self.batch_size}

        result = self.pipe(wav_data, **gen_kwargs)
        text = result.get("text", "")

        # Whisper outputs traditional Chinese by default; convert to simplified
        text = self._t2s(text)
        return {"text": text, "confidence": 0.9}

    @staticmethod
    def _to_wav_numpy(audio_bytes: bytes, encoding: str, sample_rate: int):
        """Convert raw audio bytes to numpy float32 array for the pipeline.

        Handles pcm_s16le, pcm_f32le, and falls back to WAV parsing via
        the stdlib wave module for container formats.
        """
        import numpy as np

        if encoding == "pcm_s16le":
            audio = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32) / 32768.0
        elif encoding == "pcm_f32le":
            audio = np.frombuffer(audio_bytes, dtype=np.float32)
        else:
            buf = io.BytesIO(audio_bytes)
            with wave.open(buf, "rb") as wf:
                frames = wf.readframes(wf.getnframes())
                audio = np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0
        return audio

    @staticmethod
    def _t2s(text: str) -> str:
        """Traditional Chinese -> Simplified Chinese."""
        try:
            from opencc import OpenCC
            cc = OpenCC("t2s")
            return cc.convert(text)
        except Exception:
            return text


class MockASRBackend:
    """CI mock ASR -- returns a fixed canned response, no model loaded.

    Activated when SPEECH_CI_MODE=1. Useful for testing the gRPC layer
    without requiring GPU or model weights.
    """

    def recognize(self, audio_bytes: bytes, encoding: str, sample_rate: int, language: str) -> dict:
        return {"text": "[ci-mock] hello world", "confidence": 1.0}


# -- ASR Backend (FunASR Paraformer streaming) --------------------------------

class FunASRStreamingBackend:
    """Streaming ASR backend using FunASR Paraformer-zh-streaming.

    Designed for real-time chunk-by-chunk recognition. The model processes
    fixed-size audio chunks and maintains streaming state via a cache dict.

    Model parameters:
      - Model: paraformer-zh-streaming (or FUNASR_MODEL env override)
      - chunk_size: [0, 10, 5] -- 0 left context, 10 current (600ms), 5 right (300ms)
      - chunk_stride: 10 x 960 = 9600 samples = 600ms at 16kHz
      - encoder_chunk_look_back: 4 (retain 4 past encoder chunks)
      - decoder_chunk_look_back: 1 (retain 1 past decoder chunk)

    The cache dict is passed across recognize_chunk() calls within a single
    stream session. Each Stream gRPC call gets its own cache.

    Config (environment):
        FUNASR_MODEL      -- model name or local path
        FUNASR_CHUNK_SIZE -- JSON array [left, current, right]
    """

    def __init__(self):
        from funasr import AutoModel

        model_name = os.environ.get("FUNASR_MODEL", "paraformer-zh-streaming")
        chunk_size_str = os.environ.get("FUNASR_CHUNK_SIZE", "[0,10,5]")
        # Try GPU first, fall back to CPU on init failure (typically
        # `CUDA error: out of memory` because scene's YOLO+SAM+CLIP
        # already saturated the card). FUNASR_DEVICE pins one device
        # explicitly and skips the fallback.
        device_pref = os.environ.get("FUNASR_DEVICE", "auto")
        try:
            self.chunk_size = json.loads(chunk_size_str)
        except json.JSONDecodeError:
            self.chunk_size = [0, 10, 5]

        self.chunk_stride = self.chunk_size[1] * 960

        if device_pref == "auto":
            try:
                import torch
                cuda_available = torch.cuda.is_available()
            except Exception:
                cuda_available = False
            candidates = ["cuda", "cpu"] if cuda_available else ["cpu"]
        else:
            candidates = [device_pref]

        last_err = None
        for device in candidates:
            log.info(
                "Loading FunASR model %s on %s (chunk_size=%s, stride=%d samples)...",
                model_name, device, self.chunk_size, self.chunk_stride,
            )
            try:
                self.model = AutoModel(
                    model=model_name,
                    device=device,
                    hub_kwargs={"local_files_only": True},
                )
                self.device = device
                log.info("FunASR model loaded on %s.", device)
                return
            except Exception as e:
                last_err = e
                log.warning("FunASR load on %s failed: %s", device, e)
        raise RuntimeError(
            f"FunASR failed to load on any of {candidates}: {last_err}"
        )

    def recognize_chunk(
        self,
        audio_chunk: bytes,
        cache: dict,
        is_final: bool = False,
        encoding: str = "pcm_s16le",
        sample_rate: int = 16000,
    ) -> list[dict]:
        """Process one audio chunk through the streaming Paraformer.

        Args:
            audio_chunk: Raw PCM bytes (already adapted to 16kHz mono pcm_s16le).
                Chunk size should be ~600ms (9600 samples x 2 bytes = 19200 bytes)
                for optimal granularity, but any size works.
            cache: Mutable dict maintained across chunks in one session.
                The model stores encoder/decoder state here. Must be the same
                dict for all chunks in one Stream call.
            is_final: True on the last call to flush remaining state.
                Send empty audio_chunk with is_final=True to get the final result.
            encoding: Audio encoding (default pcm_s16le).
            sample_rate: Sample rate (default 16000).

        Returns:
            List of dicts, each with "text" (str) and "confidence" (float).
            May be empty if the chunk is too short for a recognition event.
            On is_final=True, returns the final transcription.
        """
        import numpy as np

        if encoding == "pcm_s16le":
            audio = np.frombuffer(audio_chunk, dtype=np.int16)
        elif encoding == "pcm_f32le":
            audio = (np.frombuffer(audio_chunk, dtype=np.float32) * 32768).astype(np.int16)
        else:
            buf = io.BytesIO(audio_chunk)
            with wave.open(buf, "rb") as wf:
                frames = wf.readframes(wf.getnframes())
                audio = np.frombuffer(frames, dtype=np.int16)

        results = self.model.generate(
            input=audio,
            cache=cache,
            is_final=is_final,
            chunk_size=self.chunk_size,
            encoder_chunk_look_back=4,
            decoder_chunk_look_back=1,
        )

        outputs = []
        for res in results:
            if isinstance(res, dict):
                text = res.get("text", "")
            else:
                text = str(res)
            outputs.append({"text": text, "confidence": 0.9})
        return outputs


class MockASRStreamingBackend:
    """CI mock streaming ASR -- returns empty results during streaming,
    canned result on is_final. No model loaded.
    """

    def recognize_chunk(self, audio_chunk, cache, is_final=False, encoding="pcm_s16le", sample_rate=16000):
        if is_final:
            return [{"text": "[ci-mock-stream] hello world", "confidence": 1.0}]
        return [{"text": "", "confidence": 0.0}]


# -- TTS Backend (Edge TTS) --------------------------------------------------

class EdgeTTSBackend:
    """TTS backend using Microsoft Edge TTS (free, no API key required).

    Uses the edge-tts Python package which wraps Microsoft's Cognitive
    Services TTS endpoint. Outputs MP3 audio at 24kHz.

    Config (environment):
        TTS_VOICE -- voice name (default: zh-CN-XiaoxiaoNeural)
        Other voices: zh-CN-YunxiNeural, en-US-JennyNeural, ja-JP-NanamiNeural

    Speed control:
        speed parameter is a multiplier. Internally converted to Edge TTS
        rate string format: "+N%" or "-N%" where N = |speed-1| x 100.
    """

    def __init__(self):
        self.voice = os.environ.get("TTS_VOICE", "zh-CN-XiaoxiaoNeural")
        log.info("Edge TTS initialized with voice=%s", self.voice)

    async def synthesize(self, text: str, voice: str = "", speed: float = 1.0) -> bytes:
        """Synthesize text to complete MP3 audio bytes (one-shot).

        Args:
            text: Text to synthesize.
            voice: Voice name override (uses TTS_VOICE env if empty).
            speed: Speed multiplier (1.0 = normal, 2.0 = 2x fast).

        Returns:
            Complete MP3 audio bytes.
        """
        import edge_tts

        v = voice or self.voice
        sign = "+" if speed > 1 else "-"
        rate = f"{sign}{int(abs(speed - 1) * 100)}%" if speed != 1.0 else "+0%"
        communicate = edge_tts.Communicate(text, v, rate=rate)
        chunks = []
        async for chunk in communicate.stream():
            if chunk["type"] == "audio":
                chunks.append(chunk["data"])
        return b"".join(chunks)

    async def synthesize_stream(self, text: str, voice: str = "", speed: float = 1.0):
        """Async generator that yields MP3 audio chunks as they're generated.

        Useful for long text -- the caller can start playback before the
        full audio is ready.

        Args:
            text: Text to synthesize.
            voice: Voice name override.
            speed: Speed multiplier.

        Yields:
            bytes -- individual MP3 audio chunks from Edge TTS streaming.
        """
        import edge_tts

        v = voice or self.voice
        sign = "+" if speed > 1 else "-"
        rate = f"{sign}{int(abs(speed - 1) * 100)}%" if speed != 1.0 else "+0%"
        communicate = edge_tts.Communicate(text, v, rate=rate)
        async for chunk in communicate.stream():
            if chunk["type"] == "audio":
                yield chunk["data"]


class MockTTSBackend:
    """CI mock TTS -- returns a minimal valid WAV file (silence).
    Activated when SPEECH_CI_MODE=1.
    """

    async def synthesize(self, text: str, voice: str = "", speed: float = 1.0) -> bytes:
        return b"RIFF\x24\x00\x00\x00WAVEfmt \x10\x00\x00\x00\x01\x00\x01\x00\x44\xac\x00\x00\x88X\x01\x00\x02\x00\x10\x00data\x00\x00\x00\x00"

    async def synthesize_stream(self, text: str, voice: str = "", speed: float = 1.0):
        data = await self.synthesize(text, voice, speed)
        yield data


# -- Dialog Session Manager --------------------------------------------------
# Manages the lifecycle of voice dialog sessions. Each StartDialog call
# creates a session that tracks state (IDLE -> LISTENING -> PROCESSING ->
# SPEAKING) until the client disconnects.

class DialogSession:
    """Tracks state for a single voice dialog session.

    State machine:
        IDLE -> LISTENING -> PROCESSING -> SPEAKING -> IDLE (loop)
        Any state -> ERROR (on failure)

    Attributes:
        session_id: 8-char UUID prefix, unique per session.
        language: BCP-47 language tag for this session.
        enable_vad: Whether voice activity detection is active.
        state: Current state string.
        created_at: Unix timestamp of session creation.
    """

    def __init__(self, session_id: str, language: str, enable_vad: bool):
        self.session_id = session_id
        self.language = language
        self.enable_vad = enable_vad
        self.state = "IDLE"  # IDLE | LISTENING | PROCESSING | SPEAKING | ERROR
        self.created_at = time.time()

    STATE_MAP = {"IDLE": 0, "LISTENING": 1, "PROCESSING": 2, "SPEAKING": 3, "ERROR": 4}


class DialogManager:
    """Manages multiple concurrent voice dialog sessions.

    Thread-safe via dict operations (single-threaded gRPC server pool).
    Sessions are created by StartDialog and removed when the client
    disconnects (servicer finally block).
    """

    def __init__(self):
        self.sessions: dict[str, DialogSession] = {}

    def create_session(self, language: str, enable_vad: bool) -> DialogSession:
        sid = str(uuid.uuid4())[:8]
        session = DialogSession(sid, language, enable_vad)
        self.sessions[sid] = session
        log.info("Created dialog session %s (lang=%s, vad=%s)", sid, language, enable_vad)
        return session

    def get_session(self, session_id: str) -> Optional[DialogSession]:
        return self.sessions.get(session_id)

    def remove_session(self, session_id: str) -> bool:
        if session_id in self.sessions:
            del self.sessions[session_id]
            log.info("Removed dialog session %s", session_id)
            return True
        return False


# -- gRPC Servicers ----------------------------------------------------------
# These classes implement the gRPC service interfaces defined in
# robonix_contracts.proto. Each servicer wraps one or more backend engines.
# Note: the codegen service methods are named Call / Stream (not Recognize
# or Synthesize) because the contract RPC is always called "Call" or "Stream".

class SpeechAsrServicer(contracts_grpc.SystemSpeechAsrServicer):
    """ASR gRPC servicer -- handles the Call RPC for one-shot speech recognition.

    Delegates to WhisperASRBackend for transcription.

    If the backend is None (failed to load at startup), the RPC returns
    UNAVAILABLE with actionable instructions instead of crashing.
    """

    def __init__(self, asr_backend):
        self.asr_backend = asr_backend

    def Recognize(self, request, context):
        """Handle one-shot ASR: receive complete audio, return transcription.

        Request fields (from asr.proto Recognize_Request):
            audio_data     -- raw audio bytes
            encoding       -- audio encoding string (e.g. "pcm_s16le")
            sample_rate_hz -- sample rate in Hz
            language       -- BCP-47 language tag

        Returns asr_pb2.Recognize_Response with text, confidence, error.
        """
        if self.asr_backend is None:
            context.set_code(grpc.StatusCode.UNAVAILABLE)
            context.set_details(
                "ASR backend not available. "
                "Set ASR_MODEL to a local model path with pre-downloaded weights. "
                "Runtime downloads are disabled (local_files_only=True). "
                "Current ASR_MODEL="
                + os.environ.get("ASR_MODEL", "openai/whisper-large-v3")
            )
            return asr_pb2.Recognize_Response()

        from speech_service.audio_utils import adapt_audio

        encoding = request.encoding or "pcm_s16le"
        sample_rate = request.sample_rate_hz or 16000
        language = request.language

        try:
            # Adapt to 16kHz mono pcm_s16le regardless of input format
            audio_data, _ = adapt_audio(
                request.audio_data,
                encoding=encoding,
                sample_rate=sample_rate,
            )
            result = self.asr_backend.recognize(audio_data, "pcm_s16le", 16000, language)
            return asr_pb2.Recognize_Response(
                text=result["text"],
                confidence=result.get("confidence", 0.0),
                error="",
            )
        except Exception as e:
            log.exception("ASR recognize failed")
            return asr_pb2.Recognize_Response(text="", confidence=0.0, error=str(e))


class SpeechAsrStreamServicer(contracts_grpc.SystemSpeechAsrStreamServicer):
    """Streaming ASR gRPC servicer -- handles the Stream RPC for chunk-by-chunk
    speech recognition.

    Delegates to FunASRStreamingBackend for real-time recognition.

    This is a bidirectional stream: the client sends AsrAudioChunk messages
    (each containing an AudioChunk with raw audio data), and the server
    yields RecognizeStreamEvent messages with partial/final transcriptions.

    Note: the stream carries no AudioConfig or language -- the service uses
    defaults (16kHz mono pcm_s16le). Each chunk is adapted if needed.
    """

    def __init__(self, stream_asr_backend):
        self.stream_asr_backend = stream_asr_backend

    def RecognizeStream(self, request_iterator, context):
        """Handle streaming ASR: receive audio chunks, yield partial/final results.

        Input: asr_pb2.AsrAudioChunk (has .chunk with .data field from robonix_msg.AudioChunk)
        Output: asr_pb2.RecognizeStreamEvent

        Since the stream has no AudioConfig, we use defaults: 16kHz mono pcm_s16le.
        """
        if self.stream_asr_backend is None:
            context.set_code(grpc.StatusCode.UNAVAILABLE)
            context.set_details(
                "Streaming ASR backend not available. "
                "Set FUNASR_MODEL to a local model path with pre-downloaded weights. "
                "Runtime downloads are disabled (local_files_only=True). "
                "Current FUNASR_MODEL="
                + os.environ.get("FUNASR_MODEL", "paraformer-zh-streaming")
            )
            return

        from speech_service.audio_utils import adapt_audio

        cache = {}
        chunk_count = 0

        try:
            for req in request_iterator:
                # Extract raw audio bytes from AsrAudioChunk.chunk.data
                chunk_data = bytes(req.chunk.data) if req.chunk and req.chunk.data else None
                if chunk_data is None:
                    continue

                chunk_count += 1

                # Adapt each chunk to 16kHz mono pcm_s16le using defaults
                # (stream carries no AudioConfig, so assume pcm_s16le at 16kHz mono)
                adapted, _ = adapt_audio(
                    chunk_data,
                    encoding="pcm_s16le",
                    sample_rate=16000,
                    channels=1,
                    bits_per_sample=16,
                )

                results = self.stream_asr_backend.recognize_chunk(
                    adapted, cache, is_final=False,
                    encoding="pcm_s16le", sample_rate=16000,
                )
                for r in results:
                    if r.get("text"):
                        yield asr_pb2.RecognizeStreamEvent(
                            event_type=0, text=r["text"],
                            confidence=r.get("confidence", 0.0),
                            language="",
                        )

            # Final flush
            if chunk_count > 0:
                final_results = self.stream_asr_backend.recognize_chunk(
                    b"", cache, is_final=True,
                    encoding="pcm_s16le", sample_rate=16000,
                )
                for r in final_results:
                    if r.get("text"):
                        yield asr_pb2.RecognizeStreamEvent(
                            event_type=1, text=r["text"],
                            confidence=r.get("confidence", 0.0),
                            language="", is_final=True,
                        )
            else:
                yield asr_pb2.RecognizeStreamEvent(
                    event_type=2, error="No audio data received",
                )
        except Exception as e:
            log.exception("ASR stream recognize failed")
            yield asr_pb2.RecognizeStreamEvent(event_type=2, error=str(e))


class SpeechTtsServicer(contracts_grpc.SystemSpeechTtsServicer):
    """TTS gRPC servicer -- handles the Call RPC for one-shot text-to-speech.

    Delegates to EdgeTTSBackend for audio generation.
    Output format: MP3 at 24kHz (fixed by Edge TTS).

    Note: Edge TTS uses asyncio internally, but gRPC servicers are sync.
    Call uses asyncio.run().
    """

    def __init__(self, tts_backend):
        self.tts_backend = tts_backend

    def Synthesize(self, request, context):
        """Handle one-shot TTS: receive text, return complete MP3 audio.

        Request fields (from tts.proto Synthesize_Request):
            text     -- text to synthesize
            language -- BCP-47 language tag
            voice    -- voice name override
            speed    -- speed multiplier (1.0 = normal)

        Returns tts_pb2.Synthesize_Response with audio_data, encoding, sample_rate_hz.
        """
        if self.tts_backend is None:
            context.set_code(grpc.StatusCode.UNAVAILABLE)
            context.set_details(
                "TTS backend not available. "
                "Edge TTS requires network access to Microsoft Cognitive Services. "
                "Check your network connection or set SPEECH_CI_MODE=1 for testing."
            )
            return tts_pb2.Synthesize_Response()

        text = request.text
        voice = request.voice
        speed = request.speed or 1.0

        try:
            audio_data = asyncio.run(self.tts_backend.synthesize(text, voice, speed))
            return tts_pb2.Synthesize_Response(
                audio_data=audio_data,
                encoding="mp3",
                sample_rate_hz=24000,
                error="",
            )
        except Exception as e:
            log.exception("TTS synthesize failed")
            return tts_pb2.Synthesize_Response(audio_data=b"", error=str(e))


class SpeechTtsStreamServicer(contracts_grpc.SystemSpeechTtsStreamServicer):
    """Streaming TTS gRPC servicer -- handles the Stream RPC for chunk-by-chunk
    text-to-speech synthesis.

    Delegates to EdgeTTSBackend for audio generation.
    Output format: MP3 at 24kHz (fixed by Edge TTS).

    This is a server stream: unary request with text/voice/speed,
    streams back tts_pb2.SynthesizeAudioChunk messages.

    Note: Edge TTS uses asyncio internally. A dedicated event loop is
    created per call to avoid "cannot run from running loop" conflicts.
    """

    def __init__(self, tts_backend):
        self.tts_backend = tts_backend

    def SynthesizeStream(self, request, context):
        """Handle streaming TTS: receive text, yield MP3 audio chunks.

        Request fields (from tts.proto SynthesizeStream_Request):
            text        -- text to synthesize
            language    -- BCP-47 language tag
            voice       -- voice name override
            speed       -- speed multiplier
            audio_config -- AudioConfig (reserved, not used by Edge TTS)

        Yields tts_pb2.SynthesizeAudioChunk (wrapping robonix_msg.AudioChunk).
        """
        if self.tts_backend is None:
            context.set_code(grpc.StatusCode.UNAVAILABLE)
            context.set_details(
                "TTS backend not available. "
                "Edge TTS requires network access to Microsoft Cognitive Services."
            )
            return

        text = request.text
        voice = request.voice
        speed = request.speed or 1.0

        try:
            loop = asyncio.new_event_loop()
            async_gen = self.tts_backend.synthesize_stream(text, voice, speed)
            seq = 0
            while True:
                try:
                    chunk_data = loop.run_until_complete(async_gen.__anext__())
                    yield tts_pb2.SynthesizeAudioChunk(
                        chunk=robonix_msg_pb2.AudioChunk(
                            data=chunk_data,
                            sequence=seq,
                        ),
                        encoding="mp3",
                        sample_rate_hz=24000,
                        is_final=False,
                    )
                    seq += 1
                except StopAsyncIteration:
                    yield tts_pb2.SynthesizeAudioChunk(
                        chunk=robonix_msg_pb2.AudioChunk(data=b""),
                        encoding="mp3",
                        sample_rate_hz=24000,
                        is_final=True,
                    )
                    break
            loop.close()
        except Exception as e:
            log.exception("TTS stream synthesize failed")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))


class SpeechDialogServicer(contracts_grpc.SystemSpeechDialogServicer):
    """Dialog gRPC servicer -- handles the Stream RPC for voice dialog sessions.

    Creates a DialogSession and streams DialogEvent updates to the client.
    The session stays alive until the client disconnects, then is
    automatically cleaned up.

    This is a foundation for future VAD + turn-taking logic. Currently
    it only tracks session state without active audio processing.
    """

    def __init__(self, dialog_manager):
        self.dialog_manager = dialog_manager

    def StartDialog(self, request, context):
        """Handle dialog session: create session, stream state updates.

        Request fields (from speech.proto StartDialog_Request):
            language    -- BCP-47 language tag
            enable_vad  -- enable voice activity detection
            audio_config -- AudioConfig for the session

        Yields speech_pb2.StartDialog_Response wrapping robonix_msg.DialogEvent.
        """
        language = request.language or "zh-CN"
        enable_vad = request.enable_vad if hasattr(request, "enable_vad") else True

        session = self.dialog_manager.create_session(language, enable_vad)

        yield speech_pb2.StartDialog_Response(
            event=robonix_msg_pb2.DialogEvent(
                session_id=session.session_id,
                event_type=DialogSession.STATE_MAP["IDLE"],
                text="",
                confidence=0.0,
                language=session.language,
                is_final=False,
                error="",
            )
        )

        try:
            while context.is_active():
                time.sleep(1.0)
        except Exception:
            pass
        finally:
            self.dialog_manager.remove_session(session.session_id)


# -- Atlas registration (optional) -------------------------------------------
# Registers the speech service with the Robonix Atlas control plane so that
# other nodes can discover it. If Atlas is unavailable or the runtime protos
# are not installed, the service runs in standalone mode.

from robonix_py import Capability  # noqa: E402

cap = Capability(id="com.robonix.system.speech", namespace="robonix/system/speech")


def _try_backend(name: str, factory):
    """Best-effort backend init; failed backends become None and the
    corresponding servicer returns UNAVAILABLE rather than bringing the
    whole package down."""
    try:
        return factory()
    except Exception as e:  # noqa: BLE001
        log.error("%s backend FAILED (%s); the contract will return UNAVAILABLE", name, str(e)[:120])
        return None


# Servicers attached with None backends; backends are loaded inside
# on_init(cfg) so the boot manifest's `system: speech: {...}` block can
# gate which engines come up. Each servicer's Recognize/Synthesize
# already returns UNAVAILABLE when its backend is None, which covers
# the small window between gRPC server start and Driver(CMD_INIT).
log.info("Starting speech service (ci_mode=%s)", CI_MODE)
_dialog_manager = DialogManager()
_asr_servicer        = SpeechAsrServicer(None)
_asr_stream_servicer = SpeechAsrStreamServicer(None)
_tts_servicer        = SpeechTtsServicer(None)
_tts_stream_servicer = SpeechTtsStreamServicer(None)
_dialog_servicer     = SpeechDialogServicer(_dialog_manager)
cap.attach_grpc_servicer("robonix/system/speech/asr",        _asr_servicer)
cap.attach_grpc_servicer("robonix/system/speech/asr_stream", _asr_stream_servicer)
cap.attach_grpc_servicer("robonix/system/speech/tts",        _tts_servicer)
cap.attach_grpc_servicer("robonix/system/speech/tts_stream", _tts_stream_servicer)
cap.attach_grpc_servicer("robonix/system/speech/dialog",     _dialog_servicer)


# Map package_manifest cfg keys to the env vars that backend
# constructors read. Cfg wins; absent keys leave existing env intact so
# operators who export ASR_MODEL etc. in the start: block still work.
_CFG_ENV_MAP = {
    "asr_model":         "ASR_MODEL",
    "asr_device":        "ASR_DEVICE",
    "asr_chunk_length":  "ASR_CHUNK_LENGTH",
    "asr_batch_size":    "ASR_BATCH_SIZE",
    "funasr_model":      "FUNASR_MODEL",
    "funasr_device":     "FUNASR_DEVICE",
    "funasr_chunk_size": "FUNASR_CHUNK_SIZE",
    "tts_voice":         "TTS_VOICE",
}


def _apply_cfg_to_env(cfg: dict) -> None:
    for key, env in _CFG_ENV_MAP.items():
        if key not in cfg:
            continue
        v = cfg[key]
        os.environ[env] = v if isinstance(v, str) else json.dumps(v)


@cap.on_init
def init(cfg):
    log.info("Driver(INIT) cfg keys: %s", sorted(cfg.keys()))
    _apply_cfg_to_env(cfg)

    disable_whisper = bool(cfg.get(
        "disable_whisper",
        os.environ.get("SPEECH_DISABLE_WHISPER", "").strip() in ("1", "true", "yes"),
    ))

    if CI_MODE:
        asr = MockASRBackend()
        asr_stream = MockASRStreamingBackend()
        tts = MockTTSBackend()
    else:
        if disable_whisper:
            log.info("Whisper ASR disabled by config; asr contract will return UNAVAILABLE")
            asr = None
        else:
            asr = _try_backend("Whisper ASR", WhisperASRBackend)
        asr_stream = _try_backend("FunASR (streaming)", FunASRStreamingBackend)
        tts = _try_backend("Edge TTS", EdgeTTSBackend)

    _asr_servicer.asr_backend = asr
    _asr_stream_servicer.stream_asr_backend = asr_stream
    _tts_servicer.tts_backend = tts
    _tts_stream_servicer.tts_backend = tts

    log.info("Backend status: Whisper=%s FunASR=%s EdgeTTS=%s",
             "OK" if asr else "UNAVAILABLE",
             "OK" if asr_stream else "UNAVAILABLE",
             "OK" if tts else "UNAVAILABLE")

    if not any([asr, asr_stream, tts]):
        return cap.error(
            "all backends failed; set SPEECH_CI_MODE=1 for mocks or fix "
            "model / network issues (see backend errors above)"
        )
    return cap.ready()


def main() -> int:
    cap.run()
    return 0


if __name__ == "__main__":
    main()
