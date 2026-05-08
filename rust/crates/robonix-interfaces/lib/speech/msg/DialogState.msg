# State of a voice dialog session.
# Used to track the lifecycle of a voice interaction.

string session_id
uint8 IDLE       = 0
uint8 LISTENING  = 1    # mic is active, waiting for speech
uint8 PROCESSING = 2    # ASR/NLP processing in progress
uint8 SPEAKING   = 3    # TTS is playing audio to speaker
uint8 ERROR      = 4
uint8 state
string language           # current BCP-47 language code
string error              # non-empty if state == ERROR
