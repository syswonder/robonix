# Current device-reported jaw motion, independent of contact and command results.
# Unchanged width or a successful dispatch alone does not prove stopped motion.
# STOPPED does not imply arrival, successful gripping or a safety-rated stop.
uint8 UNKNOWN=0          # No trustworthy current motion observation.
uint8 MOVING=1           # The device reports ongoing opening/closing motion.
uint8 STOPPED=2          # The device reports no ongoing opening/closing motion.
