# ImageCovering.msg
# Header header: header for timestamp
# int32 drone_id: self drone id
# sensor_msgs/Image image: best image i.e. one which gives best objective function score
# int32 camera_orientation_setting: integer from 0,1,2,...7 to indicate the camera orientation setting at which the best photo was taken
# geometry_msgs/Pose pose: pose of self robot with x, y, z, yaw angle. (only x and y will change)

Header header
int32 drone_id
sensor_msgs/Image image
int32 camera_orientation_setting
geometry_msgs/Pose pose
bool flag_completed_action_selection
float64 best_marginal_gain
bool terminate_decision_making_flag

