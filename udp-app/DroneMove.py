from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import sys
import tty
import termios
import os

class Drone:
    """ Drone Class"""


    def __init__(self):
        """Basic global varbles"""
        #print "Creating Drone Object"
        #connection_string = "udp:127.0.0.1:5760"
        #self.vehicle = connect(connection_string, wait_ready=True)
        #self.add_callback()
        #self.mode = "GUIDED"
        #self.vehicle.mode = VehicleMode("GUIDED")
        #self.vehicle.groundspeed = 5;
        #self.waypoints = []
        try:
            print "Creating Drone Object"
            connection_string = "udp:127.0.0.1:5760"
            self.vehicle = connect(connection_string, wait_ready=True)
            self.mode = "GUIDED"
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.groundspeed = 2;
            self.waypoints = []
            self.run()
        except KeyboardInterrupt:
            self.clean_exit()

    def __del__(self):
        """Clean up"""
        print "Exiting Dorne Object"
        self.vehicle.close()

    def move_to_altitude(self, altitude):
        #a_location = LocationGlobal(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,altitude)
        a_location = LocationGlobalRelative(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,altitude)
        self.vehicle.simple_goto(a_location)
        #loc = self.vehicle.location
        #loc.alt = altitude
        #self.vehicle.simple_goto(loc)
        self.vehicle.flush()
        while self.vehicle.location.global_relative_frame.alt <= altitude * .95:
            time.sleep(1)
            print self.vehicle.location.global_relative_frame.alt
        time.sleep(1)
        
    def condition_chang_alt(self, altitude):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT, #command
            0, #confirmation
            5,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            0,          # param 3, direction -1 ccw, 1 cw
            0, # param 4, relative offset 1, absolute angle 0
            0, 0, altitude)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        
        
    def take_off(self, altitude):
        if self.mode != "guided":
            self.vehicle.mode = VehicleMode("GUIDED")
        if self.vehicle.is_armable:
            self.vehicle.armed = True
            time.sleep(4)	
            self.vehicle.simple_takeoff(altitude)
            while self.vehicle.location.global_relative_frame.alt <= (altitude * .85):
                time.sleep(.1)
                print self.vehicle.location.global_relative_frame.alt
            #time.sleep(1)

    def land(self):
        self.vehicle.mode = VehicleMode("LAND")
        while True:
            print "Waiting for program to end"
            time.sleep(20)

    def condition_yaw(self, heading, relative=False):
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
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            1,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def roate_on_pos(self):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, #command
            0, #confirmation
            0,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            0,          # param 3, direction -1 ccw, 1 cw
            0, # param 4, relative offset 1, absolute angle 0
            0, 0, 10)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    def clean_exit(self):
         self.vehicle.mode = VehicleMode("LAND")
         time.sleep(5)
         print "Program interrupted, switching to LAND mode."
         self.vehicle.close()
         os._exit(0)


    def run(self):
        self.take_off(5)
        time.sleep(5)
        print "mode down"
        self.move_to_altitude(4)
        time.sleep(5)
        print "roating on pos"
        self.roate_on_pos()
        time.sleep(10)
        #self.rotate_on_pos()
        #while True:
            #print self.vehicle.location.global_relative_frame.alt
            #time.sleep(.1)



    


if __name__ == '__main__':
    print "running from command line"
    Drone()






