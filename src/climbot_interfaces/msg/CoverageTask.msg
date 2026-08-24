# A complete, immutable coverage task in one wall-plane coordinate frame.

uint8 SWEEP_HORIZONTAL=1
uint8 SWEEP_VERTICAL=2

uint8 SEGMENT_SCAN=1
uint8 SEGMENT_TRANSITION=2
# Reserved, and rejected by validateCoverageTask(). No executor has a behaviour
# for it, so a task carrying one would be driven as something nobody defined.
# The number stays here rather than being removed so it can never be reused for
# something else and silently change the meaning of a recorded bag.
uint8 SEGMENT_RETURN=3

std_msgs/Header header

# Identifies one user task. It comes from planner configuration, so give each
# region its own value if results are to be told apart by task_id alone.
string task_id

# A monotonic publication counter, not a version of the region. Planning
# failures, cleared selections and empty tasks all consume a number, so gaps
# are expected: compare for newer or older, never assume consecutive values.
uint32 revision
uint8 sweep_direction

# N waypoints define N-1 directed line segments. Pose orientation points along
# the next segment; the last pose retains the arrival heading.
geometry_msgs/Pose[] waypoints
uint8[] segment_types

# Both polygons inherit header.frame_id. coverage_region is the required
# inspection area; motion_region bounds every nominal and dynamic robot path.
geometry_msgs/Polygon coverage_region
geometry_msgs/Polygon motion_region

# Dimensions of the two-dimensional inspection footprint, in metres.
float64 detection_width
float64 detection_length

# Detection-centre displacement along robot-forward from base_link. Zero is
# valid for legacy/contact tools; the front inspection camera uses +0.300 m.
float64 detection_forward_offset
