# Payload for robonix/primitive/chassis/move. Three modes — driver picks
# whichever is non-zero, in priority order:
#
#   1. forward_m  != 0 → drive straight by that signed distance (m).
#                        Driver picks a sensible linear speed and computes
#                        how long to run cmd_vel. positive = forward.
#   2. rotate_deg != 0 → in-place yaw rotation by that signed angle (deg).
#                        Driver picks angular speed; positive = CCW.
#   3. velocity mode  → use linear_*/angular_* fields directly (cmd_vel-
#                        style twist). `duration_sec > 0` overrides the
#                        deployment-owned driver default;
#                        `duration_sec == 0` keeps that default.
#
# Modes are exclusive: forward_m takes priority over rotate_deg over the
# velocity fields. Distance / angle modes are the recommended shortcuts
# for an LLM driving the robot — "rotate 360°" or "move forward 1 m" map
# directly without the agent having to guess a velocity-time product.
float64 linear_x
float64 linear_y
float64 linear_z
float64 angular_x
float64 angular_y
float64 angular_z
float64 duration_sec
float64 forward_m
float64 rotate_deg
