# Independent physical observations; no atomic multi-field snapshot is implied.
# Publish at acquisition rate; never republish cached readings as fresh.
# Invalidate only the affected observation on read failure; unsupported readings
# remain invalid (valid=false or grasp_state=UNKNOWN).
bool opening_width_valid  # False: ignore opening_width_m.
float64 opening_width_m   # Measured clear width between current gripping faces, m.
                          # Finite and nonnegative when valid.
uint8 grasp_state         # GraspState constants; UNKNOWN invalidates the reading.
