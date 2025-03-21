#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

path = Path()

def odom_callback(msg):
    global path
    
    pose = PoseStamped()
    pose.header = msg.header
    pose.pose = msg.pose.pose
    
    path.header = msg.header
    path.poses.append(pose)
    
    path_pub.publish(path)

if __name__ == '__main__':
    rospy.init_node('odom_to_path', anonymous=True)

    # Adjust the rate based on your desired callback rate
    # rate = rospy.Rate(10)  # 30 Hz by default

    # Get drone name parameter
    drone_name = rospy.get_param('~drone_name')

    # Define the path publisher
    pubstr = "/" + drone_name + "path"
    path_pub = rospy.Publisher(pubstr, Path, queue_size=1)

    # Subscribe to the odometry topic
    odom_topic = "/airsim_node_odom/" + drone_name + "/odom_local_ned"
    odom_sub = rospy.Subscriber(odom_topic, Odometry, odom_callback)

    # Main loop
    while not rospy.is_shutdown():
        # Additional processing can be added here if needed
        rospy.spin()
        # rate.sleep()
