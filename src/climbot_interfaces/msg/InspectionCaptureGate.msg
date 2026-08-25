# Position barrier owned by automatic inspection capture.
# While active, the tracker must not let the camera centre advance past
# maximum_camera_along_track on the identified frozen SCAN reference.
std_msgs/Header header
string task_id
uint32 revision
int32 segment_index
bool active
float64 maximum_camera_along_track
string reason
