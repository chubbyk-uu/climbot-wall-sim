# The operator-facing view of one coverage manager. Everything a user interface
# needs arrives on this one topic, so the interface holds no state of its own
# and cannot disagree with the manager about what is running.

# Manager states. These describe the manager, not the motion: while EXECUTING,
# executor_state says what the robot is actually doing.
uint8 IDLE=0        # no region selected, or the selection was cleared
uint8 INVALID=1     # a preview arrived that cannot be executed; see message
uint8 READY=2       # a valid task is cached and waiting for an explicit start
uint8 STARTING=3    # start requested, the executor has not answered yet
uint8 EXECUTING=4   # the executor accepted the goal and is running it
uint8 FINISHED=5    # the last run ended; see result_code and message
uint8 STOPPING=6    # contact with the executor was lost mid-run and the manager
                    # is stopping the robot; it stays here until it has evidence
                    # the robot is no longer being driven, and never reports
                    # FINISHED on the strength of the executor merely having
                    # gone quiet on the Action interface
uint8 RECOVERY_LOCKED=7  # supervision of an unknown Goal was explicitly
                         # abandoned; the speed hold remains engaged and no new
                         # task may start until the operator explicitly rearms

std_msgs/Header header

uint8 state

# Identity of the cached or executing task. An empty task_id with revision 0
# means no task has ever been received.
string task_id
uint32 revision

# current_segment is -1 while the robot is still approaching the first
# waypoint, and is only meaningful while state is EXECUTING. total_segments
# comes from the cached task and is known from READY onwards.
int32 current_segment
uint32 total_segments

# Fraction of the task the executor reports as complete, 0 to 1.
float32 progress

# The schedule as reported by the executor; see ExecuteCoverage.action, which
# stays the authoritative definition. Only meaningful while state is EXECUTING,
# and schedule_lag_s is zero throughout when the executor runs in distance
# mode, which has no schedule to be behind.
float64 planned_total_s
float64 schedule_lag_s
float64 estimated_remaining_s

# Motion state reported by the executor. The values mirror the constants in
# ExecuteCoverage.action's feedback, which stays the authoritative definition:
# WAITING, ALIGN, TURN_SETTLE, TRACK_LINE, FINAL_APPROACH, STOPPED,
# APPROACH_START. Only meaningful while state is EXECUTING.
uint8 executor_state

# Outcome of the last finished run, using the ExecuteCoverage result codes.
# Only meaningful while state is FINISHED.
uint16 result_code

# Whether the manager would accept each request right now, decided by the same
# preconditions its services apply. An interface renders these rather than
# inferring them from state, so the decision stays in one place. They are a
# hint, not a guarantee: a request can still be refused, for instance when the
# executor's Action server is not running, and the refusal explains itself.
bool can_start
bool can_cancel
bool can_force_abandon
bool can_rearm

# Task-level inspection/archive state, owned by the archive recorder and
# mirrored by the manager so the RViz task panel has one authoritative task
# status stream. inspection_enabled is frozen before the Action Goal is sent.
bool inspection_enabled
uint8 archive_state
string archive_run_id
string archive_directory
# Nominal whole-task estimate made before motion starts. It reserves archive
# capacity and stays constant during the task; it is not the frozen plan.
uint32 archive_preflight_expected_images
# Cumulative count from SCAN references frozen so far. This starts at zero and
# grows line by line; only after the final SCAN is it the task's final count.
uint32 archive_expected_images
uint32 archive_saved_images
uint32 archive_failed_images
uint64 archive_estimated_bytes
string archive_message

# The same line the manager logs. Publishing it here keeps one wording for the
# log, the panel and ros2 topic echo.
string message
