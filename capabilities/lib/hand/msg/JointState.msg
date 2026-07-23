# Current per-axis positions, published on the state_joint stream. Each entry
# carries its axis `name`, so the stream is self-describing -- the axis-level
# counterpart to FingerState.
JointValue[] joints   # value = current normalized position [0,1]
