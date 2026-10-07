uint8 DRIVE=0
uint8 TURN_LEFT=1
uint8 TURN_RIGHT=2
uint8 START_ALIGN=3
uint8 FINISH=4
uint8 REVERSE=5

uint8 type
string node_id
string incoming_lane_id
string outgoing_lane_id
geometry_msgs/Pose start_pose
geometry_msgs/Pose end_pose
float64 distance_m
bool stop_required
bool collision_checked
