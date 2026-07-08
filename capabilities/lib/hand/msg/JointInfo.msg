# One control axis (degree of freedom) of the hand, self-describing.
# Returned by info. A physical finger may expose several axes (e.g. bend at
# two links); address each by `name`, ordered palm->tip by joint_index.
# `finger`+`type` group them for the finger-level interface but are NOT unique
# on their own.
# finger role, mapped from the thumb outward (drop the outermost when fewer):
#   3-finger -> thumb,index,middle   4-finger -> +ring   5-finger -> +pinky
#   DOF beyond 5 human fingers: leave finger="", keep the real type, by name
string  finger       # thumb|index|middle|ring|pinky; "" if not finger-mapped
# type + normalized value [0,1] endpoints (intrinsic axes, observer-independent):
#   bend   : 0 = extended (straight)    1 = flexed (curled)
#   sway   : 0 = toward pinky (ulnar)   1 = toward thumb (radial)
#   oppose : 0 = neutral                1 = full opposition   (thumb only)
#   other  : 0 and 1 defined by `description`
string  type         # bend | sway | oppose | other
uint8   joint_index  # position along the finger chain, palm(0) -> tip; 0 if single
string  name         # unique addressing key; opaque -- never parse it for meaning
string  description  # human/LLM readable; MUST state 0/1 for type=other, and what/where the DOF is for finger=""
