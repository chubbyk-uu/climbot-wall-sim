# What shape and sweep the planner is currently configured for, and whether a
# replan would be accepted right now. Published latched so a panel that starts
# later still renders the truth instead of a guess.
#
# The operator interface must not decide any of this for itself: it renders
# these fields and lets the planner reject what it cannot do (PROJECT_GUIDE
# 11.1). can_plan is a hint for greying a button out, never a guarantee - the
# request may still be refused, and the reason arrives in the service response.

string REGION_RECTANGLE=rectangle
string REGION_TRAPEZOID=trapezoid

string SWEEP_HORIZONTAL=horizontal
string SWEEP_VERTICAL=vertical

std_msgs/Header header

string region_type
string sweep_direction

# How the region is being supplied: "rviz" for clicked points, "parameters"
# for the configured corners. Point counts below are meaningless in the latter.
string input_mode

# Clicks this shape needs, and clicks currently held. A shape change never
# discards points: switching to one that needs more simply leaves can_plan
# false until the missing clicks arrive, and switching back restores it.
uint8 required_points
uint8 selected_points

bool can_plan
string message
