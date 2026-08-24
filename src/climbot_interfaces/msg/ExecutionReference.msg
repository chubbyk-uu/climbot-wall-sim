# The current frozen execution reference. Unlike the nominal CoverageTask,
# start/end include post-turn slip handling and parallel-line locking.
std_msgs/Header header

string task_id
uint32 revision
int32 segment_index
uint8 segment_type
uint8 executor_state

geometry_msgs/Point start
geometry_msgs/Point end
float64 detection_forward_offset

# True only while the executor is following a formal SCAN reference. ALIGN,
# TURN_SETTLE, arc entry, transitions and start approach always publish false.
bool inspection_enabled
