# Metadata for one successfully published inspection image. The header is
# copied from image_raw, so stamp + frame_id are the exact image key.
std_msgs/Header header

string task_id
uint32 revision
int32 segment_index
uint32 trigger_index

# Target and measured camera-centre progress along the frozen SCAN reference.
float64 target_along_track
float64 actual_along_track

# EKF pose of the optical centre, interpolated at the image timestamp.
geometry_msgs/PoseWithCovariance camera_pose

# Robot-forward direction in the wall plane at exposure time, radians in the
# header frame. This is also image-up for the nominal fixed camera mount.
float64 wall_heading_rad

# Frozen wall-plane reference used for this capture.
geometry_msgs/Point reference_start
geometry_msgs/Point reference_end
