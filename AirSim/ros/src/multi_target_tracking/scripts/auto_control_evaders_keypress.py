"""NOTE!!! This script runs with python3, it does not interface with the ROS code.
Used to takeoff and automatically send commands to evader drones only.
User has to enter the number of evader drones and the IDs at which evader drone IDs begin."""

# SET THESE CONSTANTS when number of pursuers and evaders change
import math
import pygame
import time
import airsim
import numpy as np
from threading import Thread

# NUMBER_OF_PURSUERS = 5
# NUMBER_OF_EVADERS = 10

NUMBER_OF_PURSUERS = 8
NUMBER_OF_EVADERS = 12

# NUMBER_OF_PURSUERS = 10
# NUMBER_OF_EVADERS = 15

EVADER_DRONE_FIRST_ID = NUMBER_OF_PURSUERS+1

NUM_DRONES = NUMBER_OF_PURSUERS + NUMBER_OF_EVADERS
# EVADER_DRONE_LINEAR_VELOCITY = 0.75
EVADER_DRONE_LINEAR_VELOCITY = 2.5

class MoveDroneThread(Thread):
  def __init__(self, client, drone_name, move_func):
    Thread.__init__(self)
    self.client = client
    self.drone_name = drone_name
    self.move_func = move_func

  def run(self):
    self.move_func()


# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Drones to control
evader_drones = {}

# LINE PATH MOTION CHECKPOINT DISPLACEMENTS delta (x,y,theta)
linep_checkpoint_1_delta = [126, 0, 0]
# linep_checkpoint_1_delta = [-10, 0, 1]     # for testing
linep_checkpoint_2_delta = [172, 152, -0.7]


def takeoff_all(drones):
    print('Taking off evader drones to 40 m altitude...')
    for name in drones:
        client.enableApiControl(True, name)
        client.moveToZAsync(-40, 5, vehicle_name=name)  # for image covering


def land_all(drones):
    print('Landing all drones...')
    for name in drones:
        client.moveToZAsync(0, 5, vehicle_name=name)

def calculate_unit_vector(vector):
    # Calculate the magnitude of the vector
    magnitude = math.sqrt(sum(i**2 for i in vector))
    
    # Calculate the unit vector components
    unit_vector = [i / magnitude for i in vector]
    
    return unit_vector

def calculate_unit_vector_2d(vector):
    # Take only the first two elements of the vector
    vector_2d = vector[:2]
    
    # Calculate the magnitude of the 2D vector
    magnitude = math.sqrt(sum(i**2 for i in vector_2d))
    
    # Calculate the unit vector components for 2D
    unit_vector_2d = [i / magnitude for i in vector_2d]
    
    # Extend the result to a 3D unit vector by adding a zero as the third component
    unit_vector = unit_vector_2d + [0]
    
    return unit_vector

def calculate_distance_2d(point1, point2):
    # Ensure both points have at least two coordinates
    if len(point1) < 2 or len(point2) < 2:
        raise ValueError("Both points must have at least two coordinates")

    # Take only the first two coordinates for each point
    point1_2d = point1[:2]
    point2_2d = point2[:2]

    # Calculate the Euclidean distance in 2D
    distance = math.sqrt(sum((p1 - p2)**2 for p1, p2 in zip(point1_2d, point2_2d)))

    return distance


def get_box_pattern(client, drone_name, speed, distance):
    duration = distance / speed
    # position = client.getMultirotorState(vehicle_name=drone_name).kinematics_estimated.position
    altitude = -40
    multi_box_pattern = []

    if drone_name == "Drone3":  # ccw box starting forward
        multi_box_pattern = [lambda: client.moveByVelocityZAsync(vx=speed, vy=0, z=altitude, duration=duration, vehicle_name=drone_name),
                             lambda: client.moveByVelocityZAsync(
                                 vx=0, vy=-speed, z=altitude, duration=duration, vehicle_name=drone_name),
                             lambda: client.moveByVelocityZAsync(
                                 vx=-speed, vy=0, z=altitude, duration=duration, vehicle_name=drone_name),
                             lambda: client.moveByVelocityZAsync(vx=0, vy=speed, z=altitude, duration=duration, vehicle_name=drone_name)]

    elif drone_name == "Drone4":  # ccw box starting right
        multi_box_pattern = [lambda: client.moveByVelocityZAsync(vx=0, vy=speed, z=altitude, duration=duration, vehicle_name=drone_name),
                             lambda: client.moveByVelocityZAsync(
                                 vx=speed, vy=0, z=altitude, duration=duration, vehicle_name=drone_name),
                             lambda: client.moveByVelocityZAsync(
                                 vx=0, vy=-speed, z=altitude, duration=duration, vehicle_name=drone_name),
                             lambda: client.moveByVelocityZAsync(vx=-speed, vy=0, z=altitude, duration=duration, vehicle_name=drone_name)]

    return multi_box_pattern


speed = 3   # m/s
distance = 24     # m
duration = distance / speed

# Execute action sets
altitude = -39.85
evader_drone3_actionset = [lambda: client.moveByVelocityZAsync(vx=speed, vy=0, z=altitude, duration=duration, vehicle_name="Drone3"),
                           lambda: client.moveByVelocityZAsync(
                               vx=0, vy=-speed, z=altitude, duration=duration, vehicle_name="Drone3"),
                           lambda: client.moveByVelocityZAsync(
                               vx=-speed, vy=0, z=altitude, duration=duration, vehicle_name="Drone3"),
                           lambda: client.moveByVelocityZAsync(vx=0, vy=speed, z=altitude, duration=duration, vehicle_name="Drone3")]

evader_drone4_actionset = [lambda: client.moveByVelocityZAsync(vx=0, vy=speed, z=altitude, duration=duration, vehicle_name="Drone4"),
                           lambda: client.moveByVelocityZAsync(
                               vx=speed, vy=0, z=altitude, duration=duration, vehicle_name="Drone4"),
                           lambda: client.moveByVelocityZAsync(
                               vx=0, vy=-speed, z=altitude, duration=duration, vehicle_name="Drone4"),
                           lambda: client.moveByVelocityZAsync(vx=-speed, vy=0, z=altitude, duration=duration, vehicle_name="Drone4")]


# Add only evader drones to the evader_drones dict
for i in range(EVADER_DRONE_FIRST_ID, NUM_DRONES + 1):
    drone_name = f"Drone{i}"
    client.enableApiControl(True, drone_name)
    evader_drones[drone_name] = client.getMultirotorState(
        vehicle_name=drone_name)

# initialize pygame window
pygame.init()

# Set up the display, enter key inputs here
screen = pygame.display.set_mode((200, 200))

# TAKEOFF DRONES
takeoff_all(evader_drones)

# Main loop for controlling drones_________
checkpoints = [linep_checkpoint_1_delta, linep_checkpoint_2_delta]

# Store goal poses for each drone and checkpoint
goal_poses_most_recent = {drone_name: None for drone_name in evader_drones}

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_d:
                print("Starting evader drones line path motion")
                for target_checkpoint in checkpoints:
                    start_time = time.time()
                    # threads = []

                    for drone_name_key in evader_drones:
                        pose = client.getMultirotorState(
                            vehicle_name=drone_name_key).kinematics_estimated
                        position = pose.position  # has x, y, z
                        euler_angles = airsim.to_eularian_angles(pose.orientation)
                        _, _, yaw = euler_angles
                        goal_poses_most_recent[drone_name_key] = np.array([position.x_val + target_checkpoint[0],
                                                                  position.y_val + target_checkpoint[1], yaw + target_checkpoint[2]])

                        goal_yaw = yaw+target_checkpoint[2] 
                        yawrate = (target_checkpoint[2] -yaw) / 0.4     
                        unit_vec = calculate_unit_vector_2d(target_checkpoint)
                        dist = calculate_distance_2d(target_checkpoint, [0,0])
                        dur = dist / EVADER_DRONE_LINEAR_VELOCITY

                        client.moveByRollPitchYawrateZAsync(0,0, yawrate, position.z_val, 0.4, vehicle_name=drone_name_key)
                        client.moveToPositionAsync(x=(position.x_val + target_checkpoint[0]),
                                                   y=(position.y_val +
                                                      target_checkpoint[1]),
                                                   z=position.z_val, vehicle_name=drone_name_key, velocity=EVADER_DRONE_LINEAR_VELOCITY)

                        # client.moveByVelocityZBodyFrameAsync(EVADER_DRONE_LINEAR_VELOCITY * unit_vec[0], EVADER_DRONE_LINEAR_VELOCITY * unit_vec[1], z=position.z_val, vehicle_name=drone_name_key,
                        #                                             duration=dur)

                    # check if previous checkpoint was reached NOTE: CHECK IF TIME DURATION OF CHECKPOINT HAS PASSED
                    elapsed_time = time.time() - start_time
                    wait_time = dur - elapsed_time
                    if wait_time > 0:
                        time.sleep(wait_time)

                    # for time_wait_increment in range(10000):
                    #     reached_goal_flags = {drone_name: False for drone_name in evader_drones}
                            
                    #     for drone in evader_drones:
                    #         pose = client.getMultirotorState(vehicle_name=drone).kinematics_estimated
                    #         # pose = rospy.wai
                    #         position = pose.position  # has x, y, z
                    #         euler_angles = airsim.to_eularian_angles(pose.orientation)
                    #         _, _, yaw = euler_angles
                    #         curr_pose_vec = np.array([position.x_val, position.y_val, yaw])
                    #         diff_vec = np.absolute(goal_poses_most_recent[drone] - curr_pose_vec)

                    #         sum_xy = diff_vec[0] + diff_vec[1]
                    #         sum_theta = diff_vec[2]
                    #         # print("sum_xy:", sum_xy, "sum_theta: ",sum_theta, drone)

                    #         # if (sum_xy <= 0.5) and (sum_theta <= 0.35):
                    #         if (sum_xy <= 0.5):
                    #             print(drone, " reached checkpoint")
                    #             reached_goal_flags[drone] = True
                            
                    #     # if all flags are true, then break, else sleep 2.5 seconds
                    #     if all(reached_goal_flags.values()):
                    #         print("All drones reached the checkpoint")
                    #         break  # skips to next checkpoint
                    #     else:
                    #         time.sleep(2)  # seconds