# State of the task-level raw inspection archive. The archive recorder owns
# this state; the coverage manager only mirrors it for task-level UI.

uint8 IDLE=0
uint8 PREPARING=1
uint8 READY=2
uint8 RECORDING=3
uint8 FINALIZING=4
uint8 COMPLETED=5
uint8 CANCELED=6
uint8 FAILED=7

std_msgs/Header header

uint8 state
bool inspection_enabled
string task_id
uint32 revision
string run_id
string task_directory
uint32 expected_images
uint32 saved_images
uint32 failed_images
uint64 estimated_bytes
string message
