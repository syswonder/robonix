# Event emitted by the speech dialog service during a voice interaction session.
# Streamed back to the caller (e.g. Liaison) over the dialog RPC stream.

uint64 timestamp_ns       # nanoseconds since epoch
string session_id

uint8 VAD_EVENT       = 0
uint8 PARTIAL_TRANSCRIPT = 1
uint8 FINAL_TRANSCRIPT   = 2
uint8 BOT_RESPONSE_TEXT  = 3
uint8 BOT_AUDIO_CHUNK    = 4
uint8 STATE_CHANGE       = 5
uint8 ERROR              = 6
uint8 event_type

# VAD event data
speech/msg/VadEvent vad_event

# Transcript data (for PARTIAL_TRANSCRIPT and FINAL_TRANSCRIPT)
string text
float32 confidence
string language           # detected or specified language

# Bot response text (for BOT_RESPONSE_TEXT)
string response_text

# Audio data (for BOT_AUDIO_CHUNK)
audio/msg/AudioChunk audio_chunk

# State change (for STATE_CHANGE)
speech/msg/DialogState dialog_state

# Error info (for ERROR)
string error
