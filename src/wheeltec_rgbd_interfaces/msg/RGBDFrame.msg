# Header stamp = newest of the two preserved source image stamps.
# Frames describe camera optics only; no camera-to-chassis TF is implied.
std_msgs/Header header
uint64 sequence
# Input calibration provenance; does not certify physical calibration accuracy.
string calibration_source
bool rectified
sensor_msgs/Image color
# 32FC1, metres; NaN outside configured usable range or missing depth.
sensor_msgs/Image depth
sensor_msgs/CameraInfo color_info
sensor_msgs/CameraInfo depth_info
float32 valid_depth_fraction
float32 pairing_delta_sec
