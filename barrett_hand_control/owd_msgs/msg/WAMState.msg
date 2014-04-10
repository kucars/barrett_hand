# All of the data fields are at least as recent as the timestamp in
# the in header, and possibly more recent.  If a trajectory that OWD added at
# time T does not appear in the published queue at time t>=T, then that
# trajectory has definitely finished.
Header header

float64[] positions
float64[] target_positions
float64[] jpositions
float64[] velocities
float64[] torques
float64   stiffness
owd_msgs/TrajInfo[] trajectory_queue
owd_msgs/TrajInfo prev_trajectory
string controller
uint8 state

uint8 state_free=0
uint8 state_fixed=1
uint8 state_traj_active=2
uint8 state_traj_stalled=3
uint8 state_traj_paused=4
uint8 state_inactive=255

# Deprecated state names; please switch to the ones above
uint8 state_moving=2 # deprecated
uint8 state_stalled=3 # deprecated
