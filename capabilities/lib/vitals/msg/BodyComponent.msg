string name                  # "joint_1" / "motor_drive" / "display"
string kind                  # "joint" | "motor" | "actuator" | "sensor" | "other"
float32 temperature          # °C, -1 if unavailable
uint32 error_code            # 0 = OK, model-defined bitmask otherwise
bool enabled                 # torque / driver enable status
string id                    # stable component path, e.g. "body/arm_right/joint_1"
string parent_id             # parent component path, empty for root
string model                 # component model, e.g. "piper_motor" / "jetson_agx_orin"
