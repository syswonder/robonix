# One control axis addressed by name, with a normalized value. Reused across
# move_joint (target), state_joint (current), set_joint_*_limits (limit) --
# the value's meaning is fixed by whichever interface carries it.
string  name         # matches a JointInfo.name from info; opaque -- never parse it
float64 value        # normalized [0,1]
