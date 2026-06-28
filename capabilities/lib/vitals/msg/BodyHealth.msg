uint32 NORMAL=0
uint32 FAULT=1
uint32 ESTOP=2

string body_type             # "arm" | "dog" | "humanoid" | ...
string model                 # "piper" | "koch" | "go2" | ...
uint32 state                 # NORMAL / FAULT / ESTOP
string message               # human-readable status summary (can be empty)
BodyComponent[] components   # structured health entries per body sub-component
