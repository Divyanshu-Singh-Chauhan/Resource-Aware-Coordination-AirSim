#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock

# USAGE:
# rosrun multi_target_tracking clock_publisher.py _replanning_freq:=20.0

def wallclock_talker():
    pub1 = rospy.Publisher('clock',Clock, queue_size=1)
    rospy.init_node('clock_talker', anonymous=True)
    replanning_freq = rospy.get_param('~replanning_freq',8.0)  # Default to 8 Hz if not specified
    rate = rospy.Rate(replanning_freq) # hz
    sim_speed_multiplier = 1    
    sim_clock = Clock()
    zero_time = rospy.get_time()

    while not rospy.is_shutdown():
       sim_clock.clock = rospy.Time.from_sec(sim_speed_multiplier*(rospy.get_time() - zero_time))
    #    rospy.loginfo(sim_clock)
       pub1.publish(sim_clock)
       rate.sleep()

if __name__ == '__main__':
    try:
        wallclock_talker()
    except rospy.ROSInterruptException:
        pass


# import rospy
# from rosgraph_msgs.msg import Clock

# class ClockTalker:
#     def __init__(self):
#         # self.replanning_freq = replanning_freq
#         self.pub1 = rospy.Publisher('clock', Clock, queue_size=10)
#         rospy.init_node('clock_talker', anonymous=True)
#         self.replanning_freq = rospy.get_param('~replanning_freq')  # Default to 8 Hz if not specified

#     def run(self):
#         rate = rospy.Rate(self.replanning_freq)
#         sim_speed_multiplier = 1
#         sim_clock = Clock()
#         zero_time = rospy.get_time()

#         while not rospy.is_shutdown():
#             sim_clock.clock = rospy.Time.from_sec(sim_speed_multiplier * (rospy.get_time() - zero_time))
#             self.pub1.publish(sim_clock)
#             rate.sleep()

# if __name__ == '__main__':
#     try:
#         # replanning_freq = rospy.get_param('~replanning_freq', 8)  # Default to 8 Hz if not specified
#         clock_talker = ClockTalker()
#         clock_talker.run()
#     except rospy.ROSInterruptException:
#         pass
