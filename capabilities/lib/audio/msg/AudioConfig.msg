# Audio configuration descriptor for speech services.
# Shared across ASR, TTS, mic, and speaker primitives.

string encoding           # e.g. "pcm_s16le", "pcm_f32le", "opus", "mp3"
uint32 sample_rate_hz     # e.g. 16000, 22050, 44100
uint32 channels           # 1 = mono, 2 = stereo
uint32 bits_per_sample    # e.g. 16 for pcm_s16le
