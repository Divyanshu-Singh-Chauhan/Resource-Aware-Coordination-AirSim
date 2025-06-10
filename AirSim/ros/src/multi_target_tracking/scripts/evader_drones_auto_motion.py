"""NOTE!!! This script runs with python3, it does not interface with the ROS code.
Used for setting automatic motion plans for evader drones.
User has to enter the number of evader drones and the IDs at which evader drone IDs begin."""

# SET THESE CONSTANTS when number of pursuers and evaders change
EVADER_DRONE_FIRST_ID = 3
NUMBER_OF_EVADERS =2
NUMBER_OF_PURSUERS =2
NUM_DRONES= NUMBER_OF_PURSUERS + NUMBER_OF_EVADERS

import airsim
import math
import random
import time
import itertools


# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Drones to control
all_drones = {}
evader_drones = {}

def takeoff_all():
    print('Taking off all drones to 40 m altitude...')
    for name in all_drones:
        client.enableApiControl(True, name)
        # client.takeoffAsync(vehicle_name=name)
        client.moveToZAsync(-40, 8, vehicle_name=name) # for image covering

def land_all():
    print('Landing all drones...')
    for name in all_drones:
        client.moveToZAsync(0, 5, vehicle_name=name)
        # client.landAsync(vehicle_name=name)
        # client.enableApiControl(False, name)

def generate_random_position(exclude_positions=[]):
    while True:
        x = random.uniform(-100, 100)  # Adjust the range based on your environment
        y = random.uniform(-100, 100)
        z = random.uniform(-40, 40)
        position = airsim.Vector3r(x, y, z)
        
        # Check if the generated position is not too close to any other drone
        if all(position.distance(other_position) > 1 for other_position in exclude_positions):
            return position

def generate_random_control_action(client, drone_name):
    position = client.getMultirotorState(vehicle_name=drone_name).kinematics_estimated.position
    action = random.choice([
        lambda: client.moveByVelocityZAsync(vx=4, vy=0, z=position.z_val, duration=2, vehicle_name=drone_name),
        lambda: client.moveByVelocityZAsync(vx=-4, vy=0, z=position.z_val, duration=2, vehicle_name=drone_name),
        lambda: client.moveByVelocityZAsync(vx=0, vy=-4, z=position.z_val, duration=2, vehicle_name=drone_name),
        lambda: client.moveByVelocityZAsync(vx=0, vy=4, z=position.z_val, duration=2, vehicle_name=drone_name)
        # lambda: client.moveByRollPitchYawrateZAsync(roll=0, pitch=0, yaw_rate=0.785, z=position.z_val, duration=0.8, vehicle_name=drone_name),
        # lambda: client.moveByRollPitchYawrateZAsync(roll=0, pitch=0, yaw_rate=-0.785, z=position.z_val, duration=0.8, vehicle_name=drone_name)
    ])

    return action

# 2 box pattern for drones to fly in
# std::cout << "moveByVelocityZ(" << speed << ", 0, " << z << "," << duration << ")" << std::endl;
# client.moveByVelocityZAsync(speed, 0, z, duration, drivetrain, yaw_mode, drone_name);
# std::this_thread::sleep_for(std::chrono::duration<double>(duration));
# std::cout << "moveByVelocityZ(0, " << speed << "," << z << "," << duration << ")" << std::endl;
# client.moveByVelocityZAsync(0, speed, z, duration, drivetrain, yaw_mode, drone_name);
# std::this_thread::sleep_for(std::chrono::duration<double>(duration));
# std::cout << "moveByVelocityZ(" << -speed << ", 0, " << z << "," << duration << ")" << std::endl;
# client.moveByVelocityZAsync(-speed, 0, z, duration, drivetrain, yaw_mode, drone_name);
# std::this_thread::sleep_for(std::chrono::duration<double>(duration));
# std::cout << "moveByVelocityZ(0, " << -speed << "," << z << "," << duration << ")" << std::endl;
# client.moveByVelocityZAsync(0, -speed, z, duration, drivetrain, yaw_mode, drone_name);
# std::this_thread::sleep_for(std::chrono::duration<double>(duration));

def get_box_pattern(client, drone_name, speed, distance): 
    # speed = 3
    side_length = distance
    duration = side_length / speed
    position = client.getMultirotorState(vehicle_name=drone_name).kinematics_estimated.position
    multi_box_pattern = [lambda: client.moveByVelocityZAsync(vx=speed, vy=0, z=position.z_val, duration=duration, vehicle_name=drone_name), 
                        lambda: client.moveByVelocityZAsync(vx=0, vy=-speed, z=position.z_val, duration=2*duration, vehicle_name=drone_name),
                        lambda: client.moveByVelocityZAsync(vx=speed, vy=0, z=position.z_val, duration=duration, vehicle_name=drone_name), 
                        lambda: client.moveByVelocityZAsync(vx=0, vy=speed, z=position.z_val, duration=duration, vehicle_name=drone_name),
                        lambda: client.moveByVelocityZAsync(vx=-speed, vy=0, z=position.z_val, duration=2*duration, vehicle_name=drone_name),
                        lambda: client.moveByVelocityZAsync(vx=0, vy=speed, z=position.z_val, duration=duration, vehicle_name=drone_name)]

    return multi_box_pattern

# Add all drones to the all_drones dict
for i in range(1, NUM_DRONES + 1):
    drone_name = f"Drone{i}"
    client.enableApiControl(True, drone_name)
    all_drones[drone_name] = client.getMultirotorState(vehicle_name=drone_name)

# Add only evader drones to the evader_drones dict
for i in range(EVADER_DRONE_FIRST_ID, NUM_DRONES + 1):
    drone_name = f"Drone{i}"
    evader_drones[drone_name] = client.getMultirotorState(vehicle_name=drone_name)

takeoff_all()

time.sleep(11)

# Main loop for continuously sending random control actions to drones_________
speed = 3   # m/s
side_length = 15    # m

# Generate action sets
action_sets = []  
for drone_name in evader_drones:
  actions = get_box_pattern(client, drone_name, speed, side_length)
  action_sets.append(actions)

# Create generator that cycles through action sets
action_gen = itertools.cycle(action_sets)

# Track current index for each drone  
action_indices = {name: 0 for name in evader_drones}

while True:
  for drone_name in evader_drones:
    actions = next(action_gen)
    index = action_indices[drone_name]
    action = actions[index]
    action()
    time.sleep(side_length/speed)
    # Increment index for next iteration
    action_indices[drone_name] = (index + 1) % len(actions)

  time.sleep(3.5)

# while True:
#     for drone_name in evader_drones:
#         # control_action = generate_random_control_action(client, drone_name)
#         control_actions = get_box_pattern(client, drone_name, speed, side_length)
#         control_action()
#         time.sleep(1)

#     # Add a delay to control the rate at which actions are sent (adjust as needed)
#     time.sleep(3.5)