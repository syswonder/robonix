# Streamed by SrvLiaison.StartVoiceSession.
# event_kind values are stable; clients should ignore unknown kinds.
uint32 SESSION_STARTED   = 0
uint32 RECORDING_STARTED = 1
uint32 RECORDING_DONE    = 2
uint32 ASR_PARTIAL       = 3
uint32 ASR_FINAL         = 4
uint32 USER_IDENTIFIED   = 5
uint32 PILOT             = 6   # `pilot` field carries the wrapped PilotEvent
uint32 TTS_STARTED       = 7
uint32 TTS_DONE          = 8
uint32 SESSION_DONE      = 9
uint32 ERROR             = 10

uint32           event_kind
string           session_id
string           text
string           user_id
float32          confidence
pilot/PilotEvent pilot
string           error
string           status_message
uint64           timestamp_ms
