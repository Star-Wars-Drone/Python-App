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
            self.add_callback()
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

    def mode_callback(self,attr_name,value):
        """Check Mode"""
        self.mode = value.name
        print " CALLBACK: (%s)" % (value.name)
        if value.name == 'LAND':
            os._exit(1)

    def add_callback(self):
        """Add all callback function"""
        self.vehicle.add_attribute_listener('mode.name', self.mode_callback)
        
    def move_to_altitude(self, altitude):
        if self.mode != "GUIDED":
            self.vehicle.mode = VehicleMode("GUIDED")
        #a_location = LocationGlobal(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,altitude)
        a_location = LocationGlobalRelative(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,altitude)
        self.vehicle.simple_goto(a_location)
        time.sleep(1)
        #loc = self.vehicle.location
        #loc.alt = altitude
        #self.vehicle.simple_goto(loc)
        self.vehicle.flush()
        while self.vehicle.location.global_relative_frame.alt <= altitude * .85:
            time.sleep(1)
            print self.vehicle.location.global_relative_frame.alt
        time.sleep(1)
        
    def rotate_on_pos(self):
        if self.mode != "GUIDED":
            self.vehicle.mode = VehicleMode("GUIDED")
        self.condition_yaw(0)
        time.sleep(10)
        angle_count = 0
        while True:
            self.condition_yaw(1,True)
            angle = self.vehicle.attitude.yaw*57.2958
            print "altitude: {}".format(angle)
            angle_count = angle_count + 1
            if(angle < 0.5 and angle > -1 and angle_count > 2):
                break
            time.sleep(.4)
        print angle_count
        time.sleep(1)
        
    def take_off(self, altitude):
        if self.mode != "GUIDED":
            self.vehicle.mode = VehicleMode("GUIDED")
        if self.vehicle.is_armable:
            self.vehicle.armed = True
            time.sleep(1)	
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
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
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
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def take_off_and_land(self):
        self.take_off(5)
        time.sleep(1)
        #self.send_ned_velocity(0,0,1,1)
        self.move_to_altitude(25)
        time.sleep(1)
        self.rotate_on_pos()
        time.sleep(1)
        self.land()

    def read_waypoints(self,filename):
        """Gets waypoints from a file"""
        with open(filename,"r") as filestream:
            num_waypoints = 0
            for line in filestream:
                currentline = line.rstrip("\n").split(",")
                #self.waypoints.extend(LocationGlobal(float(currentline[0]),float(currentline[1]),float(currentline[2])))
                self.waypoints.append(LocationGlobalRelative(float(currentline[0]),float(currentline[1]),float(currentline[2])))
                num_waypoints = num_waypoints + 1
            return num_waypoints

    def get_distance_metres(self,aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def fly_to_waypoint(self,waypoint):
        distance_from_target = 100
        self.vehicle.simple_goto(waypoint)
        while distance_from_target > 0.5:
            current_location = LocationGlobalRelative(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,self.vehicle.location.global_frame.alt)
            distance_from_target = self.get_distance_metres(current_location,waypoint)
            print distance_from_target
            time.sleep(1)
        return

    def clean_exit(self):
         self.vehicle.mode = VehicleMode("LAND")
         time.sleep(5)
         print "Program interrupted, switching to LAND mode."
         self.vehicle.close()
         os._exit(0)


    def run(self):
        self.take_off(5)
        self.move_to_altitude(6)
        self.rotate_on_pos()
        #while True:
            #print self.vehicle.location.global_relative_frame.alt
            #time.sleep(.1)



    


if __name__ == '__main__':
    print "running from command line"
    Drone()






