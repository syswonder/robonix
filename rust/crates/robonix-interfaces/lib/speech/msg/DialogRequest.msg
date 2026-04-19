# Dialog request sent by client to start a voice dialog session.
string language           # BCP-47, e.g. "zh-CN", "" = auto-detect
string voice_id           # preferred voice, "" = default
bool   enable_vad         # enable voice activity detection
float32 vad_silence_threshold_s  # seconds of silence before ending speech (default 0.8)
float32 vad_energy_threshold     # energy threshold for VAD (0.0 = auto)
robonix_msg/msg/AudioConfig audio_config  # desired audio format
