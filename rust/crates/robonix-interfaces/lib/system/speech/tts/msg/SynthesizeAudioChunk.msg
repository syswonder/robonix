# Audio chunk streamed back by TTS during streaming synthesis.
audio/msg/AudioChunk chunk
string encoding           # e.g. "pcm_s16le", "mp3"
uint32 sample_rate_hz
bool is_final             # true when this is the last chunk
