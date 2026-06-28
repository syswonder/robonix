string body_type              # "arm" | "dog" | "humanoid"
string model                  # "piper" | "koch" | "go2"
uint32 state                  # 0=NORMAL 1=FAULT 2=ESTOP
JointHealth[] joints
