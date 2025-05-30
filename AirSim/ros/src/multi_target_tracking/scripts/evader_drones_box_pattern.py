"""NOTE!!! This script runs with python3, it does not interface with the ROS code.
Used for setting automatic motion plans for evader drones.
User has to enter the number of evader drones and the IDs at which evader drone IDs begin."""

# SET THESE CONSTANTS when number of pursuers and evaders change
EVADER_DRONE_FIRST_ID = 3
NUMBER_OF_EVADERS =2
NUMBER_OF_PURSUERS =2
NUM_DRONES= NUMBER_OF_PURSUERS + NUMBER_OF_EVADERS

import airsim
import time

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
        client.moveToZAsync(-40, 8, vehicle_name=name) # for image covering

def land_all():
    print('Landing all drones...')
    for name in all_drones:
        client.moveToZAsync(0, 5, vehicle_name=name)


def get_box_pattern(client, drone_name, speed, distance): 
    duration = distance / speed
    # position = client.getMultirotorState(vehicle_name=drone_name).kinematics_estimated.position
    altitude = -40
    multi_box_pattern=[]

    if drone_name == "Drone3": #ccw box starting forward
        multi_box_pattern = [lambda: client.moveByVelocityZAsync(vx=speed, vy=0, z=altitude, duration=duration, vehicle_name=drone_name), 
                            lambda: client.moveByVelocityZAsync(vx=0, vy=-speed, z=altitude, duration=duration, vehicle_name=drone_name),
                            lambda: client.moveByVelocityZAsync(vx=-speed, vy=0, z=altitude, duration=duration, vehicle_name=drone_name), 
                            lambda: client.moveByVelocityZAsync(vx=0, vy=speed, z=altitude, duration=duration, vehicle_name=drone_name)]

    elif drone_name == "Drone4": #ccw box starting right
        multi_box_pattern = [lambda: client.moveByVelocityZAsync(vx=0, vy=speed, z=altitude, duration=duration, vehicle_name=drone_name), 
                            lambda: client.moveByVelocityZAsync(vx=speed, vy=0, z=altitude, duration=duration, vehicle_name=drone_name),
                            lambda: client.moveByVelocityZAsync(vx=0, vy=-speed, z=altitude, duration=duration, vehicle_name=drone_name), 
                            lambda: client.moveByVelocityZAsync(vx=-speed, vy=0, z=altitude, duration=duration, vehicle_name=drone_name)]

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
distance = 24     # m
duration = distance / speed

# Execute action sets
altitude=-39.85
evader_drone3_actionset = [lambda: client.moveByVelocityZAsync(vx=speed, vy=0, z=altitude, duration=duration, vehicle_name="Drone3"), 
                            lambda: client.moveByVelocityZAsync(vx=0, vy=-speed, z=altitude, duration=duration, vehicle_name="Drone3"),
                            lambda: client.moveByVelocityZAsync(vx=-speed, vy=0, z=altitude, duration=duration, vehicle_name="Drone3"), 
                            lambda: client.moveByVelocityZAsync(vx=0, vy=speed, z=altitude, duration=duration, vehicle_name="Drone3")]

evader_drone4_actionset = [lambda: client.moveByVelocityZAsync(vx=0, vy=speed, z=altitude, duration=duration, vehicle_name="Drone4"), 
                            lambda: client.moveByVelocityZAsync(vx=speed, vy=0, z=altitude, duration=duration, vehicle_name="Drone4"),
                            lambda: client.moveByVelocityZAsync(vx=0, vy=-speed, z=altitude, duration=duration, vehicle_name="Drone4"), 
                            lambda: client.moveByVelocityZAsync(vx=-speed, vy=0, z=altitude, duration=duration, vehicle_name="Drone4")]

# iterate through actions
while True:
    for i in range(4):
        evader_drone3_action = evader_drone3_actionset[i]
        evader_drone3_action()

        evader_drone4_action = evader_drone4_actionset[i]
        evader_drone4_action()

        time.sleep(duration+1.5)