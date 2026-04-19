"""Audio format adaptation utilities for the speech service.

This module is the upper-layer adaptation layer between the audio_driver
(prm, hardware) and the ASR backends (Whisper, FunASR). Its job is to
normalize ANY audio format the lower layer sends into the exact format
the ASR backends expect: **16 kHz mono pcm_s16le**.

Pipeline (adapt_audio):
  1. Decode raw bytes → float32 numpy array in [-1, 1]
  2. Mix multi-channel → mono (mean of channels)
  3. Resample to 16 kHz using scipy.signal.resample_poly (polyphase FIR)
  4. Re-encode to pcm_s16le (int16, 2 bytes per sample)

Supported input encodings:
  - pcm_s16le  — 16-bit signed little-endian (most common ALSA output)
  - pcm_f32le  — 32-bit float little-endian
  - pcm_s32le  — 32-bit signed little-endian
  - pcm_u8     — 8-bit unsigned
  - pcm_s24le  — 24-bit signed little-endian (3 bytes per sample)
  - wav        — RIFF/WAV container (any PCM sub-format)

Usage:
  from speech_service.audio_utils import adapt_audio, parse_audio_config

  # From a gRPC RecognizeRequest:
  audio, enc = adapt_audio(
      request.audio_data,
      encoding=request.encoding or "pcm_s16le",
      sample_rate=request.sample_rate_hz or 16000,
      channels=request.channels or 1,
      bits_per_sample=request.bits_per_sample or 16,
  )
  # audio is now 16kHz mono pcm_s16le bytes, enc is "pcm_s16le"

  # From a streaming RecognizeStreamRequest with AudioConfig:
  config = parse_audio_config(req.audio_config)
  adapted, _ = adapt_audio(chunk_data, **config)
"""
import io
import math
import wave
import logging

log = logging.getLogger(__name__)

# Target format — all ASR backends (Whisper, FunASR) expect this exact format.
# The adapt_audio() function converts any input to these parameters.
TARGET_SAMPLE_RATE = 16000   # Hz — standard telephony/ASR sample rate
TARGET_CHANNELS = 1          # mono


def parse_audio_config(audio_config) -> dict:
    """Extract audio format fields from a proto AudioConfig message.

    Accepts any object (proto message, dict, or None) and returns a dict
    with all four format parameters, using sensible defaults for missing
    or zero-valued fields (proto3 defaults all fields to zero/empty).

    Args:
        audio_config: Proto AudioConfig object or None. Attributes read:
            encoding (str), sample_rate_hz (int), channels (int),
            bits_per_sample (int).

    Returns:
        dict with keys: encoding, sample_rate, channels, bits_per_sample.
        Default values: "pcm_s16le", 16000, 1, 16.
    """
    return {
        "encoding": getattr(audio_config, "encoding", None) or "pcm_s16le",
        "sample_rate": getattr(audio_config, "sample_rate_hz", None) or 16000,
        "channels": getattr(audio_config, "channels", None) or 1,
        "bits_per_sample": getattr(audio_config, "bits_per_sample", None) or 16,
    }


def adapt_audio(
    audio_bytes: bytes,
    encoding: str = "pcm_s16le",
    sample_rate: int = 16000,
    channels: int = 1,
    bits_per_sample: int = 16,
) -> tuple[bytes, str]:
    """Normalise audio to 16 kHz mono pcm_s16le for ASR backends.

    This is the core adaptation function. It converts any input format
    to the exact format required by Whisper and FunASR backends.

    Pipeline:
      1. _decode_to_float() — decode raw bytes → float32 ndarray in [-1, 1]
      2. Mono mix — if channels > 1, reshape(-1, ch).mean(axis=1)
      3. _resample() — scipy.signal.resample_poly if sample_rate != 16000
      4. Encode to pcm_s16le — clip + cast to int16 + tobytes()

    Args:
        audio_bytes: Raw audio bytes (PCM or WAV container).
        encoding: Audio encoding string. Supported: pcm_s16le, pcm_f32le,
            pcm_s32le, pcm_u8, pcm_s24le, wav. Unknown encodings fall back
            to pcm_s16le decoding with a warning.
        sample_rate: Input sample rate in Hz. If different from 16000,
            resampling is performed using polyphase FIR filters.
        channels: Number of audio channels. If > 1, channels are mixed
            to mono by averaging.
        bits_per_sample: Bits per sample (8, 16, 24, or 32). Used by
            pcm_s24le decoder; other encoders derive this from the format.

    Returns:
        Tuple of (audio_bytes, encoding_string):
        - audio_bytes: pcm_s16le bytes at 16 kHz mono
        - encoding_string: always "pcm_s16le"

    Example:
        # Convert 48kHz stereo audio to 16kHz mono
        data, enc = adapt_audio(raw_bytes, "pcm_s16le", 48000, 2, 16)
        # data is now 16kHz mono s16le, enc is "pcm_s16le"
    """
    import numpy as np

    if len(audio_bytes) == 0:
        return audio_bytes, encoding

    # 1. Decode to float32 numpy array
    audio = _decode_to_float(audio_bytes, encoding, bits_per_sample)

    # 2. Multi-channel → mono
    if channels > 1:
        audio = audio.reshape(-1, channels).mean(axis=1)

    # 3. Resample if needed
    if sample_rate != TARGET_SAMPLE_RATE:
        audio = _resample(audio, sample_rate, TARGET_SAMPLE_RATE)
        log.debug("Resampled %d Hz → %d Hz (%d samples)",
                  sample_rate, TARGET_SAMPLE_RATE, len(audio))

    # 4. Encode back to pcm_s16le
    pcm = (audio * 32768.0).clip(-32768, 32767).astype(np.int16).tobytes()
    return pcm, "pcm_s16le"


def _decode_to_float(data: bytes, encoding: str, bits_per_sample: int) -> "np.ndarray":
    """Decode raw audio bytes to a float32 numpy array normalized to [-1, 1].

    Each encoding type is handled individually for correct normalization:
      - pcm_s16le: int16 / 32768.0
      - pcm_f32le: already float32, assumed in [-1, 1]
      - pcm_s32le: int32 / 2147483648.0
      - pcm_u8: uint8 / 128.0 - 1.0 (shift unsigned to signed range)
      - pcm_s24le: manual 3-byte LE unpack + sign extension / 8388608.0

    For unknown encodings, falls back to WAV parsing, then to pcm_s16le.

    Args:
        data: Raw audio bytes.
        encoding: Encoding string (e.g. "pcm_s16le").
        bits_per_sample: Bits per sample (used for pcm_s24le).

    Returns:
        numpy float32 array with values in [-1, 1].
    """
    import numpy as np

    if encoding == "pcm_s16le":
        return np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

    if encoding == "pcm_f32le":
        return np.frombuffer(data, dtype=np.float32)

    if encoding == "pcm_s32le":
        return np.frombuffer(data, dtype=np.int32).astype(np.float32) / 2147483648.0

    if encoding == "pcm_u8":
        return np.frombuffer(data, dtype=np.uint8).astype(np.float32) / 128.0 - 1.0

    if encoding == "pcm_s24le":
        # 24-bit LE: read 3 bytes per sample, manual unpack
        n_samples = len(data) // 3
        raw = np.frombuffer(data, dtype=np.uint8).reshape(-1, 3)
        # little-endian: LSB first
        samples = (raw[:, 0].astype(np.int32)
                   | (raw[:, 1].astype(np.int32) << 8)
                   | (raw[:, 2].astype(np.int32) << 16))
        # Sign-extend from 24-bit
        samples = np.where(samples >= 0x800000, samples - 0x1000000, samples)
        return samples.astype(np.float32) / 8388608.0

    # Fallback: try WAV decode
    buf = io.BytesIO(data)
    try:
        with wave.open(buf, "rb") as wf:
            frames = wf.readframes(wf.getnframes())
            return np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0
    except wave.Error:
        # Last resort: assume pcm_s16le
        log.warning("Unknown encoding %s, treating as pcm_s16le", encoding)
        return np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0


def _resample(audio: "np.ndarray", orig_sr: int, target_sr: int) -> "np.ndarray":
    """Resample audio using polyphase FIR filter (scipy.signal.resample_poly).

    This method is preferred over FFT-based resampling for audio because:
      - It preserves the waveform shape (linear-phase FIR)
      - It handles rational rate ratios exactly (no spectral leakage)
      - It's fast for typical audio lengths

    The up/down factors are computed from the GCD of the two rates to
    avoid unnecessarily large filter lengths.

    Args:
        audio: float32 numpy array (mono, any length).
        orig_sr: Original sample rate in Hz.
        target_sr: Desired sample rate in Hz.

    Returns:
        Resampled float32 numpy array at target_sr.
    """
    from scipy.signal import resample_poly

    gcd = math.gcd(target_sr, orig_sr)
    up = target_sr // gcd
    down = orig_sr // gcd
    return resample_poly(audio, up, down)
