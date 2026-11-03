# One acquisition round, not an atomic hardware snapshot.
# Publish at acquisition rate; only newly acquired readings are valid.
# Cached, failed or unsupported readings use opening_width_valid=false,
# grasp_state=UNKNOWN or motion_state=UNKNOWN for the affected observation.
bool opening_width_valid  # False: ignore opening_width_m.
float64 opening_width_m   # Measured clear width between current gripping faces, m.
                          # Finite and nonnegative when valid.
uint8 grasp_state         # GraspState constants; UNKNOWN invalidates the reading.
# Acquisition-round start in Unix epoch time; (sec=0, nanosec=0) means unknown.
# Convert device or simulation clocks to this base; never use publication time.
builtin_interfaces/Time stamp
uint8 motion_state        # MotionState constants; UNKNOWN invalidates the reading.
