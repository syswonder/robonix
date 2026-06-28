uint32 NORMAL=0
uint32 FAULT=1
uint32 ESTOP=2

string body_type             # root child id/name, e.g. "computer_jetson" | "arm_right"
string model                 # "jetson_agx_orin" | "piper" | "mock_bms" | ...
uint32 state                 # NORMAL / FAULT / ESTOP
string message               # human-readable status summary (can be empty)
BodyComponent[] components   # component-tree entries below this root child
