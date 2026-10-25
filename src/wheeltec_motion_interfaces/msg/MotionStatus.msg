# Control state, not measured chassis velocity or physical stop confirmation.
uint8 IDLE=0
uint8 ACTION=1
uint8 NAVIGATION=2
builtin_interfaces/Time stamp
uint8 mode
string phase
string reason
bool control_enabled
bool extrinsics_calibrated
bool pose_ready
uint64 navigation_session
int64 navigation_enabled_stamp_ns
# -1 when no accepted request is cached.
float64 navigation_request_age_sec
# Path tracker target velocities before acceleration limiting.
float64 requested_linear_x
float64 requested_angular_z
float64 output_linear_x
float64 output_linear_y
float64 output_angular_z
