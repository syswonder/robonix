string name                  # "joint_1" / "motor_drive" / "display"
string kind                  # "joint" | "motor" | "actuator" | "sensor" | "other"
float32 temperature          # °C, -1 if unavailable
uint32 error_code            # 0 = OK, model-defined bitmask otherwise
bool enabled                 # torque / driver enable status
