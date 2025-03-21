"""NOTE!!! This script runs with python3, it does not interface with the ROS code.
Used to takeoff and automatically send commands to evader drones only.
User has to enter the number of evader drones and the IDs at which evader drone IDs begin."""

# SET THESE CONSTANTS when number of pursuers and evaders change
import math
import time
import airsim
import numpy as np
from threading import Thread
import random

NUMBER_OF_PURSUERS = 2
NUMBER_OF_EVADERS = 2

# NUMBER_OF_PURSUERS = 5
# NUMBER_OF_EVADERS = 10

# NUMBER_OF_PURSUERS = 8
# NUMBER_OF_EVADERS = 12

# NUMBER_OF_PURSUERS = 10
# NUMBER_OF_EVADERS = 15

EVADER_DRONE_FIRST_ID = NUMBER_OF_PURSUERS+1

NUM_DRONES = NUMBER_OF_PURSUERS + NUMBER_OF_EVADERS
# EVADER_DRONE_LINEAR_VELOCITY = 0.75
# EVADER_DRONE_LINEAR_VELOCITY = 3.25
EVADER_DRONE_LINEAR_VELOCITY = 1.5


# Define number of random displacements per drone
NUM_FORWARD = 1
NUM_LEFT = 3
NUM_RIGHT = 3
NUM_ALL = 3*NUM_FORWARD + 2*(NUM_LEFT+NUM_RIGHT)
x_range = (35., 45)
y_ranges = [(-45, -35), (35, 45)]
y_options = [y for ymin,ymax in y_ranges for y in range(ymin, ymax)] 
y_range = (-5, 5)

y_small_dev = [(-25, -15), (15, 25)]
y_small_opts = [y for ymin,ymax in y_small_dev for y in range(ymin, ymax)] 

x_range_leftsplit = (45., 55)
y_range_leftsplit = (-40, -25.)


x_range_rightsplit = (45., 55)
y_range_rightsplit = (25, 40.)


# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Drones to control
evader_drones = {}

# LINE PATH MOTION CHECKPOINT DISPLACEMENTS delta (x,y,theta)__
linep_checkpoint_1_delta = [126, 0, 0]
# linep_checkpoint_1_delta = [-10, 0, 1]     # for testing
linep_checkpoint_2_delta = [172, 152, -0.7]


def takeoff_all(drones):
    print('Taking off evader drones to 40 m altitude...')
    base_altitude = -40.0
    altitude_increment = [-1, -0.5, 0, 0.5, 1]

    for i in range(EVADER_DRONE_FIRST_ID, NUM_DRONES + 1):
        name = f"Drone{i}"
        # decide flight level here
        altitude_index = (i - EVADER_DRONE_FIRST_ID) // 4  # Calculate index 
        altitude_offset = altitude_increment[altitude_index] if altitude_index < len(altitude_increment) else 0

        altitude = base_altitude + altitude_offset
        client.enableApiControl(True, name)
        client.moveToZAsync(altitude, 5, vehicle_name=name)  # for image covering


def get_speed_scale():
    return random.uniform(0.9, 1.1)
#   return random.uniform(0.8, 1.2)

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

# Add only evader drones to the evader_drones dict
for i in range(EVADER_DRONE_FIRST_ID, NUM_DRONES + 1):
    drone_name = f"Drone{i}"
    client.enableApiControl(True, drone_name)
    # evader_drones[drone_name] = client.getMultirotorState(vehicle_name=drone_name)
    evader_drones[drone_name] = [] #initialize empty list to store randomly selected control actions

# Main loop for controlling drones_________
checkpoints = [linep_checkpoint_1_delta, linep_checkpoint_2_delta]

# Generate random displacements______________
# initialize drone displacements
drone_displacements = {} 
for i in range(EVADER_DRONE_FIRST_ID, NUM_DRONES + 1):
    drone_name = f"Drone{i}"
    drone_displacements[drone_name] = []

# FORWARD 1 CHECKPOINTS
for i in range(EVADER_DRONE_FIRST_ID, NUM_DRONES + 1):
    drone_name = f"Drone{i}"
    random_displacements = []
    for i in range(1):
        x = random.uniform(*x_range)
        y = random.uniform(*y_range)
        random_displacements.append((x, y))    

    drone_displacements[drone_name] = random_displacements

# HERD LEFT SPLIT, First 2 RANDOM CHECKPOINTS
for i in range(EVADER_DRONE_FIRST_ID, NUMBER_OF_PURSUERS+ int(NUMBER_OF_EVADERS/2)+1):
    drone_name = f"Drone{i}"
    for j in range(2):
        x = random.uniform(*(25,40))
        y = random.uniform(*(-30,-50))
        drone_displacements[drone_name].append((x, y))

# HERD RIGHT SPLIT, 2 RANDOM CHECKPOINTS
for i in range(NUMBER_OF_PURSUERS+ int(NUMBER_OF_EVADERS/2)+1, NUM_DRONES + 1):
    drone_name = f"Drone{i}"
    for j in range(2):
        x = random.uniform(*(25, 40))
        y = random.uniform(*(30, 50))
        drone_displacements[drone_name].append((x, y))

# FORWARD CHECKPOINTS
for i in range(EVADER_DRONE_FIRST_ID, NUM_DRONES + 1):
    drone_name = f"Drone{i}"
    for j in range(8):
        x = random.uniform(*(30,35))
        y = np.random.choice(y_small_opts)
        drone_displacements[drone_name].append((x, y))

# # RIGHT HERD GOES TO THE LEFT, 2 RANDOM CHECKPOINTS
# for i in range(NUMBER_OF_PURSUERS+ int(NUMBER_OF_EVADERS/2)+1, NUM_DRONES + 1):
#     drone_name = f"Drone{i}"
#     for j in range(2):
#         x = random.uniform(*x_range_leftsplit)
#         y = random.uniform(*y_range_leftsplit)
#         drone_displacements[drone_name].append((x, y))

# # LEFT HERD GOES TO RIGHT, 2 RANDOM CHECKPOINTS
# for i in range(EVADER_DRONE_FIRST_ID, NUMBER_OF_PURSUERS+ int(NUMBER_OF_EVADERS/2)+1):
#     drone_name = f"Drone{i}"
#     for j in range(2):
#         x = random.uniform(*x_range_rightsplit)
#         y = random.uniform(*y_range_rightsplit)
#         drone_displacements[drone_name].append((x, y))

# ALL DRONES MOVE FORWARD 3 RANDOM CHECKPOINTS FINALLY
# for i in range(EVADER_DRONE_FIRST_ID, NUM_DRONES + 1):
#     drone_name = f"Drone{i}"
#     for i in range(NUM_FORWARD):
#         x = random.uniform(*x_range)
#         y = random.uniform(*y_range)
#         drone_displacements[drone_name].append((x, y))

# ___________________________

# TAKEOFF DRONES
takeoff_all(evader_drones)
time.sleep(7.0) #sleep until takeoff: 5.0 seconds is exact time to takeoff and start moving right away

# Store goal poses for each drone and checkpoint
goal_poses_most_recent = {drone_name: None for drone_name in evader_drones}
goal_unit_vecs_most_recent = {drone_name: None for drone_name in evader_drones}

# Keep track of current displacement index for each drone
displacement_idx = {drone: 0 for drone in evader_drones}
running = True
keep_cycling = True
print("Starting evader drones line path motion")
dronestr = "Drone" + str(EVADER_DRONE_FIRST_ID)
for i in range(len(drone_displacements[dronestr])):
    start_time = time.time()
    for drone_name_key in evader_drones:
        # Get current displacement for this drone
        # displacement = drone_displacements[drone_name_key][displacement_idx[drone_name_key]]
        x_additional = random.uniform(*(-5, 5))
        y_additional = np.random.choice(y_small_opts)
        displacement = drone_displacements[drone_name_key][i]
        
        # Calculate goal pose with displacement
        pose = client.getMultirotorState(vehicle_name=drone_name_key).kinematics_estimated 
        position = pose.position
        goal_poses_most_recent[drone_name_key] = np.array([position.x_val + displacement[0]+x_additional,
                                            position.y_val + displacement[1]+y_additional])

        unit_vec = calculate_unit_vector_2d(displacement)
        goal_unit_vecs_most_recent[drone_name_key] = unit_vec

        dist = calculate_distance_2d(displacement, [0,0])
        dur = dist / EVADER_DRONE_LINEAR_VELOCITY

        scale = get_speed_scale()
        velocity_scaled = scale * EVADER_DRONE_LINEAR_VELOCITY 

        client.moveToPositionAsync(x=(position.x_val + displacement[0]+x_additional),
                                    y=(position.y_val + displacement[1]+y_additional),
                                    z=position.z_val, vehicle_name=drone_name_key, velocity=velocity_scaled)
    
    # check if previous checkpoint was reached NOTE: CHECK IF TIME DURATION OF CHECKPOINT HAS PASSED
    # elapsed_time = time.time() - start_time
    # wait_time = dur - elapsed_time
    # if wait_time > 0:
    #     time.sleep(wait_time)
    reached_goal_flags = {drone_name: False for drone_name in evader_drones}
    for time_wait_increment in range(10000):
            
        for drone in evader_drones:
            pose = client.getMultirotorState(vehicle_name=drone).kinematics_estimated
            position = pose.position  # has x, y, z
            curr_pose_vec = np.array([position.x_val, position.y_val])
            diff_vec = np.absolute(goal_poses_most_recent[drone] - curr_pose_vec)

            sum_xy = diff_vec[0] + diff_vec[1]
            if (reached_goal_flags[drone]==False):
                if (sum_xy <= 15):
                    # print(drone, " reached checkpoint")
                    reached_goal_flags[drone] = True
                    # tell this drone to continue moving in the same direction
                    scale = get_speed_scale()
                    vel_vec = goal_unit_vecs_most_recent[drone]
                    # x_comp_scaled = 0.75*EVADER_DRONE_LINEAR_VELOCITY *vel_vec[0]
                    # y_comp_scaled = 0.75*EVADER_DRONE_LINEAR_VELOCITY *vel_vec[1]
                    x_comp_scaled = EVADER_DRONE_LINEAR_VELOCITY *vel_vec[0]
                    y_comp_scaled = EVADER_DRONE_LINEAR_VELOCITY *vel_vec[1]

                    client.moveByVelocityZBodyFrameAsync(x_comp_scaled, y_comp_scaled, position.z_val, duration=15.0, vehicle_name=drone) 
            
        # if all flags are true, then break, else sleep 2.5 seconds
        if all(reached_goal_flags.values()):
            # print("All drones reached the checkpoint")
            break  # skips to next checkpoint
        else:
            time.sleep(2)  # s, wait to check next one


for i in range(EVADER_DRONE_FIRST_ID, NUM_DRONES + 1):
    drone_name = f"Drone{i}"    
    client.enableApiControl(False, drone_name) # exits cleanly
