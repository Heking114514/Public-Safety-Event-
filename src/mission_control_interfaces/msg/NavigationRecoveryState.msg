std_msgs/Header header
bool valid
string reason
uint64 route_id
geometry_msgs/PoseStamped safe_pose
# Empty phase preserves the legacy recovery contract.
string phase
uint64 recovery_id
uint64 obstacle_revision
geometry_msgs/Polygon obstacle
string blocked_lane_id
RecoveryCandidate[] candidates
string resume_node
string resume_lane
float64 resume_heading
