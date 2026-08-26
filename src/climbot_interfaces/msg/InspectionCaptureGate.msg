# Health heartbeat owned by automatic inspection capture.
# v1 requires active=false; an active position barrier is rejected because
# normal image capture must not modulate scan speed. The retained field is
# reserved for a future, explicitly versioned recovery protocol.
std_msgs/Header header
string task_id
uint32 revision
int32 segment_index
bool active
float64 maximum_camera_along_track
string reason
