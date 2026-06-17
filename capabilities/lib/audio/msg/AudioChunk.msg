# A chunk of raw or encoded audio data.
# Used as the stream element for mic topic_out, speaker topic_in,
# ASR streaming, and TTS streaming.

uint64 timestamp_ns       # nanoseconds since epoch (replaces std_msgs/Header for portability)
uint8[] data              # raw or encoded audio bytes
uint32 sequence           # monotonically increasing within a session
float32 duration_s        # duration of this chunk in seconds (0 if unknown)
