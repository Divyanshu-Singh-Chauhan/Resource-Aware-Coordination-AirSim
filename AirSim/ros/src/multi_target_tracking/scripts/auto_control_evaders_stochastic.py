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

# NUMBER_OF_PURSUERS = 5
# NUMBER_OF_EVADERS = 10

# NUMBER_OF_PURSUERS = 8
# NUMBER_OF_EVADERS = 12

NUMBER_OF_PURSUERS = 10
NUMBER_OF_EVADERS = 15

EVADER_DRONE_FIRST_ID = NUMBER_OF_PURSUERS+1

NUM_DRONES = NUMBER_OF_PURSUERS + NUMBER_OF_EVADERS
# EVADER_DRONE_LINEAR_VELOCITY = 0.75
EVADER_DRONE_LINEAR_VELOCITY = 3.0 # 3.5, 4m/s with rbg works well; 2.5m too slow


# Define number of random displacements per drone
NUM_RANDOM = 20
x_range = (35., 40)
y_range = (-30, 30)
y_ranges = [(-30, -20), (20, 30)]
y_options = [y for ymin,ymax in y_ranges for y in range(ymin, ymax)] 

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
# Store displacements for each drone
drone_displacements = {} 
for drone_name in evader_drones:

    # Generate random displacements
    random_displacements = []
    for i in range(NUM_RANDOM):
        x = random.uniform(*x_range)
        y = random.uniform(*y_range)
        random_displacements.append((x, y))

    # Add first global checkpoint as last displacement 
    # global_checkpoint = checkpoints[0] 
    # random_displacements.append(global_checkpoint)

    # Store list of displacements for this drone
    drone_displacements[drone_name] = random_displacements
# ___________________________

# TAKEOFF DRONES
takeoff_all(evader_drones)
time.sleep(5.2) #sleep until takeoff: 5.0 seconds is exact time to takeoff and start moving right away

# Store goal poses for each drone and checkpoint
goal_poses_most_recent = {drone_name: None for drone_name in evader_drones}

# Keep track of current displacement index for each drone
displacement_idx = {drone: 0 for drone in evader_drones}
running = True
keep_cycling = False

keep_cycling = True
print("Starting evader drones line path motion")

for i in range(NUM_RANDOM):
    start_time = time.time()
    for drone_name_key in evader_drones:
        # Get current displacement for this drone
        # displacement = drone_displacements[drone_name_key][displacement_idx[drone_name_key]]
        x = random.uniform(x_range[0], x_range[1]) 
        y = np.random.choice(y_options)
        displacement = (x, y)
        # displacement = drone_displacements[drone_name_key][i]
        
        # Increment index for next iteration
        # if displacement_idx[drone_name_key] < NUM_RANDOM:
        #     displacement_idx[drone_name_key] += 1
        
        # Calculate goal pose with displacement
        pose = client.getMultirotorState(vehicle_name=drone_name_key).kinematics_estimated 
        position = pose.position
        euler_angles = airsim.to_eularian_angles(pose.orientation)
        _, _, yaw = euler_angles

        # goal_poses_most_recent[drone_name_key] = np.array([position.x_val + displacement[0],
        #                                             position.y_val + displacement[1], yaw + displacement[2]])
        unit_vec = calculate_unit_vector_2d(displacement)
        dist = calculate_distance_2d(displacement, [0,0])
        dur = dist / EVADER_DRONE_LINEAR_VELOCITY

        client.moveToPositionAsync(x=(position.x_val + displacement[0]),
                                    y=(position.y_val +
                                        displacement[1]),
                                    z=position.z_val, vehicle_name=drone_name_key, velocity=EVADER_DRONE_LINEAR_VELOCITY)
    
    # check if previous checkpoint was reached NOTE: CHECK IF TIME DURATION OF CHECKPOINT HAS PASSED
    elapsed_time = time.time() - start_time
    wait_time = dur - elapsed_time
    if wait_time > 0:
        time.sleep(wait_time-1.)


for i in range(EVADER_DRONE_FIRST_ID, NUM_DRONES + 1):
    drone_name = f"Drone{i}"    
    client.enableApiControl(False, drone_name) # exits cleanly
