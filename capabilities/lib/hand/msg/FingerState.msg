# Current per-finger positions, published on the state_finger stream. The
# finger-level counterpart to JointState; driver reports whatever
# (finger, type) pairs it maps.
FingerValue[] fingers  # value = current normalized position [0,1]
