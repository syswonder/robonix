# Event streamed back by the ASR service during streaming recognition.

uint8 PARTIAL = 0
uint8 FINAL   = 1
uint8 ERROR   = 2
uint8 event_type

string text             # recognised text so far (partial or final)
float32 confidence      # 0.0 – 1.0
string language         # detected language (BCP-47)
bool is_final           # true when this is the final recognition result
string error            # non-empty on error
