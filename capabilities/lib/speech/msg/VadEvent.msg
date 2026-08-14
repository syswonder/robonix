# Voice Activity Detection event.
# Emitted by the dialog manager or ASR service to signal speech state changes.

uint64 timestamp_ns       # nanoseconds since epoch
uint8 SPEECH_BEGIN = 0
uint8 SPEECH_END   = 1
uint8 SILENCE      = 2
uint8 event_type
float32 confidence       # 0.0 – 1.0 confidence of the VAD decision
float32 energy_level     # audio energy level (optional, 0.0 if unavailable)
