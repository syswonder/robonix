# One finger motion addressed by (finger, type), with a normalized value.
# Reused across move_finger (target), state_finger (current) and
# set_finger_*_limits (limit). The driver maps this to the hand's underlying
# axes, so it stays hardware-independent.
string  finger       # thumb|index|middle|ring|pinky
string  type         # bend | sway | oppose | other; endpoints defined by JointInfo
float64 value        # normalized [0,1]
