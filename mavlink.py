from pymavlink import mavutil
import time

# Establish connection with the vehicle
connection_string = "udp:127.0.0.1:14551"
print(f"Connecting to vehicle on: {connection_string}")
vehicle = mavutil.mavlink_connection(connection_string)

# Wait for vehicle heartbeat to confirm connection
print("Waiting for vehicle heartbeat...")
vehicle.wait_heartbeat()
print("Heartbeat received")

def set_relative_yaw(target_yaw):
    print(f"Setting yaw: {target_yaw}, relative: True")
    
    # Send command for relative yaw adjustment
    vehicle.mav.command_long_send(
        vehicle.target_system, 
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,  # Confirmation
        target_yaw,  # Target yaw angle
        0,  # Yaw speed (not used here)
        1,  # Direction (clockwise)
        1,  # Relative offset
        0, 0, 0  # Unused parameters
    )
    print("Command sent")

# Continuous yaw adjustment example
try:
    while True:
        set_relative_yaw(10)  # Rotate 10 degrees clockwise relative to current heading
        time.sleep(1)  # Adjust rotation speed by modifying sleep duration
except KeyboardInterrupt:
    print("Operation stopped by user")