# Ordered world-frame path; header stamp is the decision start time.
# camera_init, metres. Empty path requests zero output without disabling navigation.
nav_msgs/Path path
# Positive limits, additionally clamped to vehicle limits.
float64 max_linear_speed
float64 max_angular_speed
