#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math
from math import sqrt, atan2, pi
from multi_target_tracking.msg import PursuerEvaderData, PursuerEvaderDataArray
import tf
from geometry_msgs.msg import Quaternion

# Detection range
# DETECTION_RANGE = 30  # m. Distance above which pursuer cannot detect evader in RGBD and SS camera streams
# COMMUNICATION_RANGE = 25  #m pursuers can communicate only within this range

class PursuerEvaderDataSender:
    def __init__(self):
        rospy.init_node("pursuer_evader_data_node")
        self.pursuer_drone_poses = {}
        self.evader_drone_poses = {}

        # airsim camera FOV angles: previously chosen
        # self.fov_y = 1.262 # in rads; 72.3 deg vertical FOV
        # self.fov_x = 1.5708 # in rads; 90.0 deg

        # for use with launch file
        self.num_pursuer_drones = rospy.get_param('~num_pursuer_drones')
        self.num_evader_drones = rospy.get_param('~num_evader_drones')
        self.DETECTION_RANGE = rospy.get_param('~detection_range') # m. Distance above which pursuer cannot detect evader in RGBD and SS camera streams
        self.COMMUNICATION_RANGE = rospy.get_param('~communication_range') #m pursuers can communicate only within this range
        self.fov_x = rospy.get_param('~horizontal_FOV') 
        self.fov_y = rospy.get_param('~vertical_FOV')

        # for debugging
        # self.num_pursuer_drones = 2
        # self.num_evader_drones = 2

        self.numdrones= self.num_pursuer_drones + self.num_evader_drones

        self.pursuer_drone_rb = {
            i: {
                j: {"range": [], "bearing": []}
                for j in range(1, self.num_pursuer_drones+1) if i != j
            }
            for i in range(1, self.num_pursuer_drones+1)
        }

        self.evader_drone_rb = {  # i here is key; index of pursuer; j is key id of evader
            i: {
                j: {"range": [], "bearing": []} 
                for j in range(self.num_pursuer_drones+1, self.numdrones+1) if i != j
            }
            for i in range(1, self.num_pursuer_drones+1)
        }

        self.evader_drone_xyz = {
            i: {
                j: {"x": [], "y": [], "z": []} 
                for j in range(self.num_pursuer_drones+1, self.numdrones+1) if i != j
            }
            for i in range(1, self.num_pursuer_drones+1)
        }

        self.evaders_in_fov = {i: [] for i in range(1, self.num_pursuer_drones+1)} # this is for pursuers' IDs as keys
        self.evader_observers_dict = {i: [] for i in range(self.num_pursuer_drones+1, self.numdrones+1)}  # this is for evaders' IDs as keys
        
        self.neighbors = {i: {"in": set(), "out": set()} for i in range(1, self.numdrones+1)}  # Initialize neighbors for drones

        self.pe_data_pub = rospy.Publisher('pursuer_evader_data', PursuerEvaderDataArray, queue_size=1)

        self.poses_received = 0 # counter for collecting poses from all drones
        self.drones_received = set()
        self.subscribe_pose_flag = True
        self.setup_subscribers()


    def setup_subscribers(self):
        n = self.numdrones
        for i in range(n):
            # posesub_topic = "/airsim_node_odom_drone" + str(i + 1) + "/Drone" + str(i + 1) + "/odom_local_ned"
            posesub_topic = "/airsim_node_odom" + "/Drone" + str(i + 1) + "/odom_local_ned"
            # posesub_topic = "/airsim_node_imagecovering_drone" + str(i + 1) + "/Drone" + str(i + 1) + "/odom_local_ned"
            # posesub_topic = "/airsim_node_IMU_RGBDSS_drone" + str(i + 1) + "/Drone" + str(i + 1) + "/odom_local_ned"
            rospy.Subscriber(posesub_topic, Odometry, self.pose_callback, [i + 1], queue_size=1)


    def pose_callback(self, msg, args):
        if self.subscribe_pose_flag:
            # saves the pose of each drone the self.drones dictionary with id as key
            drone_id = args[0]
            if drone_id <= self.num_pursuer_drones:
                self.pursuer_drone_poses[drone_id] = msg.pose.pose
            elif (drone_id > self.num_pursuer_drones) and (drone_id <= self.numdrones): #being extra clear with logic here, could just use else
                self.evader_drone_poses[drone_id] = msg.pose.pose
            
            self.drones_received.add(drone_id)

    def compute_pe_data(self):
        # first compute pursuer-pursuer range/bearings, add ids and r/b data to two arrays
        for i, pose_i in self.pursuer_drone_poses.items():
            for j, pose_j in self.pursuer_drone_poses.items():
                if i == j:
                    continue
                
                range, bearing_ij = self.compute_range_bearing(pose_i, pose_j)
                bearing_ji = -(pi - bearing_ij)

                self.pursuer_drone_rb[i][j]["range"] = range
                self.pursuer_drone_rb[i][j]["bearing"] = bearing_ij

                self.pursuer_drone_rb[j][i]["range"] = range
                self.pursuer_drone_rb[j][i]["bearing"] = bearing_ji

                if range <= self.COMMUNICATION_RANGE:
                    # NOTE: assuming bi-directional communication, adding in-communication drones as both
                    # in and out neighbors
                    self.neighbors[i]["in"].add(j)
                    self.neighbors[i]["out"].add(j)
                    
                    self.neighbors[j]["in"].add(i)
                    self.neighbors[j]["out"].add(i)
 

        # then compute pursuer-evader range/bearings, add ids and r/b data to two arrays
        for i, pose_i in self.pursuer_drone_poses.items():
            for j, pose_j in self.evader_drone_poses.items():

                range, bearing_ij = self.compute_range_bearing(pose_i, pose_j) # range from pursuer to evader
                self.evader_drone_rb[i][j]["range"] = range
                self.evader_drone_rb[i][j]["bearing"] = bearing_ij

                self.evader_drone_xyz[i][j]["x"] = pose_j.position.x
                self.evader_drone_xyz[i][j]["y"] = pose_j.position.y
                self.evader_drone_xyz[i][j]["z"] = pose_j.position.z

                # then, for each pursuer-evader set, compute which evaders are currently in that pursuer's field of view, save this in another array
                # check whether drone in FOV
                # roll, pitch, yaw = tf.transformations.euler_from_quaternion(self.pursuer_drone_poses[i].orientation)
                quaternion =Quaternion(x=self.pursuer_drone_poses[i].orientation.x, y=self.pursuer_drone_poses[i].orientation.y, z=self.pursuer_drone_poses[i].orientation.z, 
                w=self.pursuer_drone_poses[i].orientation.w)
                euler_angles = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
                _, _, yaw = euler_angles
                in_fov = self.is_evader_in_fov(pose_i, pose_j, self.DETECTION_RANGE, -yaw)
                if in_fov:
                    self.evaders_in_fov[i].append(j)
                    self.evader_observers_dict[j].append(i)
                        

    def is_evader_in_fov(self, pursuer_pose, evader_pose, max_dist, pursuer_yaw):
        """
        """
        # Calculate vector between pursuer and evader
        dx = evader_pose.position.x - pursuer_pose.position.x
        dy = evader_pose.position.y - pursuer_pose.position.y

        # Rotate vector by pursuer's yaw angle
        rotated_dx = dx * math.cos(pursuer_yaw) - dy * math.sin(pursuer_yaw)
        rotated_dy = dx * math.sin(pursuer_yaw) + dy * math.cos(pursuer_yaw)

        # Calculate distance 
        # dist = math.sqrt(rotated_dx**2 + rotated_dy**2)
        dist = rotated_dx # must only check this component because we are seeing planar distance and FOV angles

        # range must not be radial, must follow FOV trapezium
        if dist > max_dist:
            return False

        if dist > 0:
            unit_vector = (rotated_dx/dist, rotated_dy/dist)
            angle = math.atan2(unit_vector[1], unit_vector[0])

            if abs(angle) > self.fov_x/2:
                return False
        
        # Vertical FOV check
        vertical_angle = math.atan2(evader_pose.position.z - pursuer_pose.position.z, dist)

        if abs(vertical_angle) > self.fov_y/2:
            return False

        return True # within field of view


    def publish_neighbors_data(self):
        msg = PursuerEvaderDataArray()
        msg.header.stamp = rospy.Time.now()
        for pursuer_id in self.pursuer_drone_rb:
            pursuer_msg = PursuerEvaderData()
            pursuer_msg.drone_id = pursuer_id
            # pursuer_msg.header.stamp = rospy.Time.now()

            # add self drone pose
            quaternion =Quaternion(x=self.pursuer_drone_poses[pursuer_id].orientation.x, y=self.pursuer_drone_poses[pursuer_id].orientation.y, z=self.pursuer_drone_poses[pursuer_id].orientation.z, 
                w=self.pursuer_drone_poses[pursuer_id].orientation.w)
            euler_angles = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            _, _, yaw = euler_angles
            pursuer_msg.self_drone_pose = [self.pursuer_drone_poses[pursuer_id].position.x, self.pursuer_drone_poses[pursuer_id].position.y, self.pursuer_drone_poses[pursuer_id].position.z, -yaw]
            
            # Populate pursuer range bearing, ids
            pursuer_msg.pursuers_range_bearing_ids = list(self.pursuer_drone_rb[pursuer_id].keys())
            pursuer_rb_vals = []
            for pursuer_neighbor_id in self.pursuer_drone_rb[pursuer_id]:
                pursuer_rb_vals.append(self.pursuer_drone_rb[pursuer_id][pursuer_neighbor_id]["range"])
                pursuer_rb_vals.append(self.pursuer_drone_rb[pursuer_id][pursuer_neighbor_id]["bearing"])
            pursuer_msg.pursuers_range_bearing_values = pursuer_rb_vals

            # Populate evaders range bearing, ids
            pursuer_msg.evaders_range_bearing_ids = list(self.evader_drone_rb[pursuer_id].keys())
            evader_rb_vals = []
            evader_xy_vals = []
            for evader_neighbor_id in self.evader_drone_rb[pursuer_id]:
                evader_rb_vals.append(self.evader_drone_rb[pursuer_id][evader_neighbor_id]["range"])
                evader_rb_vals.append(self.evader_drone_rb[pursuer_id][evader_neighbor_id]["bearing"])

                evader_xy_vals.append(self.evader_drone_xyz[pursuer_id][evader_neighbor_id]["x"])
                evader_xy_vals.append(self.evader_drone_xyz[pursuer_id][evader_neighbor_id]["y"])
                evader_xy_vals.append(self.evader_drone_xyz[pursuer_id][evader_neighbor_id]["z"])

            pursuer_msg.evaders_range_bearing_values = evader_rb_vals
            pursuer_msg.evaders_x_y_z_values = evader_xy_vals
            
            for neighbors_data_key, _ in self.neighbors[pursuer_id].items():
                if neighbors_data_key == 'in':
                    pursuer_msg.in_neighbor_ids = list(self.neighbors[pursuer_id]["in"]) 
                elif neighbors_data_key == 'out':
                    pursuer_msg.out_neighbor_ids = list(self.neighbors[pursuer_id]["out"])   

            # Populate IDs of evaders in FOV
            pursuer_msg.evaders_in_FOV = self.evaders_in_fov[pursuer_id]

            msg.pursuer_evader_info_array.append(pursuer_msg)

        self.pe_data_pub.publish(msg)

    def compute_dist(self, pose_i, pose_j):
        return sqrt((pose_i.position.x - pose_j.position.x) ** 2 +
                                (pose_i.position.y - pose_j.position.y) ** 2 +
                                (pose_i.position.z - pose_j.position.z) ** 2)

    def compute_range_bearing(self, pose_i, pose_j):
        # NOTE bearing goes from x1,y1 of pose_i to x2,y2 of pose_j, in RADIANS
        # Calculate distance between poses 
        dx = pose_j.position.x - pose_i.position.x
        dy = pose_j.position.y - pose_i.position.y
        range = sqrt(dx**2 + dy**2)

        # Calculate bearing in RADIANS
        # bearing = atan2(dx, dy)
        bearing = atan2(dy, dx)

        return range, bearing


if __name__ == '__main__':
    mesh_network = PursuerEvaderDataSender()
    # rate = rospy.Rate(4)

    # Get the rate parameter
    rate_param = rospy.get_param('~pub_freq') 
    rate = rospy.Rate(rate_param) 

    while not rospy.is_shutdown():
        # print(len(mesh_network.drones_received))
        if len(mesh_network.drones_received) == mesh_network.numdrones:
            mesh_network.subscribe_pose_flag = False

            mesh_network.compute_pe_data()
            mesh_network.publish_neighbors_data()
            mesh_network.drones_received.clear()

            # clear values of keys in FOV
            for key in mesh_network.evaders_in_fov:
                mesh_network.evaders_in_fov[key] = []

        mesh_network.subscribe_pose_flag = True
        rate.sleep()
