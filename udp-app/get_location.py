import dronekit
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative 
from pymavlink import mavutil # Needed for command message definitions
import time
import math

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
#Set up velocity vector to map to each direction.
# vx > 0 => fly North
# vx < 0 => fly South
NORTH = 2
SOUTH = -2

# Note for vy:
# vy > 0 => fly East
# vy < 0 => fly West
EAST = 2
WEST = -2

# Note for vz: 
# vz < 0 => ascend
# vz > 0 => descend
UP = -0.5
DOWN = 0.5
    
connection_string = "udp:127.0.0.1:5760"
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = dronekit.connect(connection_string, wait_ready=True)
vehicle.mode = dronekit.VehicleMode("GUIDED")
vehicle.airspeed = .001
vehicle.groundspeed = .2;
if vehicle.is_armable:
    vehicle.armed = True
    time.sleep(1)	
    vehicle.simple_takeoff(20)
    while vehicle.location.global_relative_frame.alt <= 20 * .95:
	time.sleep(1)
	print vehicle.location.global_relative_frame.alt
    #send_ned_velocity(0,0,0,1)
    #a_location = LocationGlobal(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,vehicle.location.global_frame.alt+3)
    send_ned_velocity(0,0,1,1)
    #vehicle.simple_goto(a_location)
    time.sleep(1)
    condition_yaw(0)
    time.sleep(4)

    #condition_yaw(90)
    #time.sleep(10)
    angle_count = 0
    while True:
        condition_yaw(1,True)
        angle = vehicle.attitude.yaw*57.2958
        print "altitude: {}".format(angle)
        angle_count = angle_count + 1
        if(angle < 0 and angle > -1 and angle_count > 2):
            break
        time.sleep(.4)
    print angle_count
    time.sleep(10)
    vehicle.mode = dronekit.VehicleMode("LAND")
    while vehicle.armed == True:
        time.sleep(1)
vehicle.close()

