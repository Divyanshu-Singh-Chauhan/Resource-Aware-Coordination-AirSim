# PursuerEvaderData.msg
# int32 drone_id: self drone ID 
# float64[] self_drone_pose: pose of self drone x, y, z, theta
# int32[] pursuers_range_bearing_ids: IDS of pursuers corresponding to pursuers_range_bearing_values array
# float64[] pursuers_range_bearing_values: range, bearing flattened array of other pursuers relative to self drone's x,y,theta
# int32[] evaders_range_bearing_ids: IDS of pursuers corresponding to evaders_range_bearing_values array
# float64[] evaders_range_bearing_values: range, bearing flattened array of other pursuers relative to self drone's x,y,theta
# float64[] evaders_x_y_z_values: similar to range, bearing, this is just flattened array with x,y positions of evaders
# int32[] evaders_in_FOV: IDs of evaders which are currently in the field of view
# int32[] in_neighbor_ids:  IDs of all in-neighbors pursuers (can receive data from them) of current pursuer
# int32[] out_neighbor_ids: IDs of all out-neighbors pursuers (can send data to them) of current pursuer
Header header
int32 drone_id
float64[] self_drone_pose
int32[] pursuers_range_bearing_ids
float64[] pursuers_range_bearing_values
int32[] evaders_range_bearing_ids
float64[] evaders_range_bearing_values
float64[] evaders_x_y_z_values
int32[] evaders_in_FOV
int32[] in_neighbor_ids
int32[] out_neighbor_ids