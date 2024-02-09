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
    vehicle.mav.command_long_send(
        vehicle.target_system, 
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,  # Confirmation
        target_yaw,  # Target yaw angle
        0,  # Yaw speed (not used)
        1,  # Direction (clockwise)
        1,  # Relative offset
        0, 0, 0  # Unused parameters
    )
    print("Command sent")

def set_guided_mode():
    while not is_guided_mode():
        vehicle.mav.set_mode_send(
            vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            vehicle.mode_mapping()['GUIDED'])
        print("Attempting to set vehicle to GUIDED mode")
        time.sleep(1)
    print("Vehicle set to GUIDED mode")

def get_current_mode():
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
    return mavutil.mode_string_v10(msg)

def is_guided_mode():
    if get_current_mode() == 'GUIDED':
        print("Vehicle is in GUIDED mode")
        return True
    print("Vehicle is not in GUIDED mode")
    return False

def ensure_guided_mode():
    if not is_guided_mode():
        set_guided_mode()

def check_arm_status():
    # Fetch the latest heartbeat message to check armed status
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
    # MAV_STATE_STANDBY (4) indicates the vehicle is armed but not flying.
    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    return is_armed

def arm_and_takeoff(drone, target_altitude):
    # Request to set the drone to GUIDED mode
    ensure_guided_mode()

    # Arm the drone
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # Arm
        0, 0, 0, 0, 0, 0, 0  # 7 parameters, with the first one being meaningful here
    )
    # Wait for the drone to be armed
    while True:
        msg = drone.recv_match(type='HEARTBEAT', blocking=True)
        is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if is_armed:
            print("Drone is armed")
            break
        time.sleep(0.5)

    # Take off to target altitude
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # Confirmation
        0,  # Pitch
        0,  # Empty
        0,  # Yaw angle
        0,  # Latitude (not used)
        0,  # Longitude (not used)
        0,  # Param 6 (not used)
        target_altitude  # Altitude (Param 7)
    )
    print(f"Taking off to {target_altitude}m altitude")

    # Wait until the drone reaches the target altitude
    while True:
        msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0  # Convert from mm to m
        print(f"Current altitude: {current_altitude}m")
        if current_altitude >= target_altitude * 0.95:  # Close to target altitude
            print("Target altitude reached")
            break
        time.sleep(1)

def goto_altitude(down):
    msg = vehicle.mav.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        0, 0, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.mav.send(msg)

def init_adjustment(target_altitude):

    #check relative altitude
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_altitude = msg.relative_alt / 1000.0

    if not check_arm_status():
        print("Vehicle is not armed")

        arm_and_takeoff(vehicle, target_altitude)
    
    ensure_guided_mode()
    print("Vehicle is armed and in GUIDED mode")

    if target_altitude/current_altitude > 1.05 or target_altitude/current_altitude < 0.95:
        goto_altitude(-target_altitude)
        print("Adjusting altitude")
    
    print("Vehicle is about to be at the correct altitude")
        

if __name__ == "__main__":
    init_adjustment(5)

    try:
        while True:
            set_relative_yaw(10)  # Adjust yaw by 10 degrees clockwise
            time.sleep(1)
    except KeyboardInterrupt:
        print("Operation stopped by user")