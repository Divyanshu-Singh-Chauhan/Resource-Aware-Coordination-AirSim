# NOTE!!! This script is not used anymore because multiple Airsim client connections results 
# in simulation latency. See "evader_drones_auto_motion.py" for takeoff + random motion
import airsim
import math
import time

NUM_DRONES = 4

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Drones to control
drones = {}

def takeoff_all():
    print('Taking off all drones to 40 m altitude...')
    for name in drones:
        client.enableApiControl(True, name)
        # client.takeoffAsync(vehicle_name=name)
        client.moveToZAsync(-40, 8, vehicle_name=name) # for image covering
        # client.moveToZAsync(-8, 8, vehicle_name=name) # for active SLAM with kimera-multi

def land_all():
    print('Landing all drones...')
    for name in drones:
        client.moveToZAsync(0, 5, vehicle_name=name)

# Main loop for controlling drones
for i in range(1, NUM_DRONES + 1):
    drone_name = f"Drone{i}"
    drones[drone_name] = client.getMultirotorState(vehicle_name=drone_name) 

takeoff_all()
time.sleep(10)
airsim.wait_key('Press any key to disable/switch off api control only after enabling auto motion of evaders')

# for i in range(1, NUM_DRONES+1):
#     drone_name = f"Drone{i}"    
#     client.enableApiControl(False, drone_name) # exits Unity cleanly


# Clean up
# land_all()
