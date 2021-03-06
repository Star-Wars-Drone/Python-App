from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
from LidarLiteV3 import LidarLiteV3
from smbus import SMBus
import time
import math
import sys
import tty
import termios
import os
from laserctl import Laser
from balloon_finder import BalloonFinder

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
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.groundspeed = 2;
            self.waypoints = []
            print "Entering run"
            #self.test_check_center()
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
        #a_location = LocationGlobal(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,altitude+96)
        #a_location = LocationGlobalRelative(35.768052,-78.662367,altitude)
        a_location = LocationGlobalRelative(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,altitude)
        self.vehicle.simple_goto(a_location)
        #self.vehicle.commands.goto(a_location)
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
        
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print "Basic pre-arm checks"
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)

        print "Arming motors"
        # Copter should arm in GUIDED mode
        self.vehicle.mode    = VehicleMode("GUIDED")
        self.vehicle.armed   = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print " Waiting for arming..."
            time.sleep(1)

        print "Taking off!"
        self.vehicle.simple_takeoff(altitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt
            #Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt>=altitude*0.95:
                print "Reached target altitude"
                break
            time.sleep(1)
            
            #if self.vehicle.is_armable:
            #    self.vehicle.armed = True
            #    time.sleep(1)	
            #    self.vehicle.simple_takeoff(altitude)
            #    while self.vehicle.location.global_relative_frame.alt <= (altitude * .85):
            #        time.sleep(1)
            #        print self.vehicle.location.global_relative_frame.alt
                #time.sleep(1)

    def land(self):
        """ Switches the vehicle to LAND mode."""
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
            .01,          # param 2, yaw speed deg/s
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
        #self.rotate_on_pos()
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

    def goto_position_target_local_ned(self,north, east, down):
        """ 
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
        location in the North, East, Down frame.

        It is important to remember that in this frame, positive altitudes are entered as negative 
        "Down" values. So if down is "10", this will be 10 metres below the home altitude.

        Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
        ignored. For more information see: 
        http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.

        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def set_attitude_target(self,roll,pitch,yaw):
        array = [1,0,0,0]
        msg = self.vehicle.message_factory.set_attitude_target_encode(
                0, #time_boot_ms (not used)
                0, 0, #target system, target component
                0b00000000,
                array, # Quaternion set for no rotation
                roll,pitch,yaw, # roll,pitch,yaw rates
                1)
        self.vehicle.send_mavlink(msg)
                

    def clean_exit(self):
         self.vehicle.mode = VehicleMode("LAND")
         time.sleep(5)
         print "Program interrupted, switching to LAND mode."
         self.vehicle.close()
         os._exit(0)

    def roate_360_div_by_n(self,n):
        if n == 0:
            return
        print "Moving to North"
        self.condition_yaw(0,False)
        time.sleep(5)
        detla_deg = 360/n
        print "detla is: {}".format(detla_deg)
        time.sleep(1)
        for i in range(n):
            pos = i*detla_deg
            print "Moivne to {}".format(pos)
            self.condition_yaw(pos,False)
            time.sleep(5)
        print "Done wiht roate"
        time.sleep(1)

    def rotate_and_check_360_div_by_n(self,n):
        if n == 0:
            return
        print "Moving to North"
        self.condition_yaw(0,False)
        time.sleep(5)
        detla_deg = 360/n
        print "detla is: {}".format(detla_deg)
        time.sleep(1)
        for i in range(n):
            pos = i*detla_deg
            print "Moiving to {}".format(pos)
            self.condition_yaw(pos,False)
            while True:
                angle = self.vehicle.attitude.yaw*57.2958
                if(angle < 0):
                    angle = 360 + angle
                print "angle is: {}".format(angle)
                if pos + 1.5 > angle and pos -1.5 < angle:
                    time.sleep(2)
                    if pos + 1.5 > angle and pos -1.5 < angle:
                        time.sleep(2)
                        break
                time.sleep(.1)
            time.sleep(.5)
        print "Done with rotate"
        time.sleep(1)


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

    def kill(self,bf):
        bus = SMBus(1)
        range_sensor = LidarLiteV3(bus)
        range_sensor.begin(0,0x62)
        dest = range_sensor.get_distance(False,0x62)
        print "#########################Distance: {}############################".format(dest)
        if dest < 160:
            print "Killing"
            laser = Laser()
            laser.trigger(1)
            time.sleep(2) #MAKE SURE THE switch_off gets called!! either by timeout or manually calling switch_off()
            print("Laser test done")
            bus.close()
            time.sleep(5)
            im, balloon_list = bf.find_balloons()
            if len(balloon_list)>0:
                return False
            return True
        return False
            #time.sleep(.33)
            #return True
        #bus.close()
        #return False
        #time.sleep(10)

    def init_checks(self):
        print "Basic pre-arm checks"
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)


        #if self.vehicle.mode.name == "INITIALISING":
            #print "Waiting for vehicle to initialise"
            #time.sleep(1)
        while self.vehicle.gps_0.fix_type < 2:
            print "Waiting for GPS...:", self.vehicle.gps_0.fix_type
            time.sleep(1)


    def update_vector(self,bf): 
        for i in range(20):
            print "looking for Balloon"
            im, balloon_list = bf.find_balloons()
            if len(balloon_list)>0 and balloon_list != None:
                # if multiple, find one most likely to be true.
                true_balloon = bf.pick_best_balloon(balloon_list)
                if true_balloon == None:
                    continue
                # find the vector to that balloon
                tvec = bf.find_vector(true_balloon)
                x = tvec[2] * .0254
                y = tvec[0] * .0254
                z = tvec[1] * .0254
                print "====Vector==================="
                print tvec
                print "============================="
                self.vehicle.mode = VehicleMode("GUIDED")
                time.sleep(2)
                print "Flying to way point"
                #self.goto_position_target_local_ned(x,y,z)
                #time.sleep(10)
                vec = [x,y,z]
                rotated_angle=0
                return vec
            #time.sleep(.033)
        return []


    def rotate_and_check_for_balloon(self,n,bf):
        if n == 0:
            return
        self.vehicle.mode = VehicleMode("AUTO")
        time.sleep(2)
        print "Moving to North"
        self.condition_yaw(0,False)
        time.sleep(5)
        detla_deg = 360/n
        print "detla is: {}".format(detla_deg)
        time.sleep(1)
        for i in range(n):
            pos = i*detla_deg
            print "Moiving to {}".format(pos)
            self.condition_yaw(pos,False)
            while True:
                angle = self.vehicle.attitude.yaw*57.2958
                if(angle < 0):
                    angle = 360 + angle
                print "angle is: {}".format(angle)
                if pos + 5 > angle and pos -5 < angle:
                    time.sleep(2)
                    if pos + 5 > angle and pos -5 < angle:
                        time.sleep(2)
                        break
                time.sleep(1)
            time.sleep(1)
            print "Checking for Balloon"
            vec = self.update_vector(bf)
            if vec != []:
                print "Found Balloon"
                return vec
            print "Didn't find Balloon"
#            for i in range(60):
#                im, balloon_list = bf.find_balloons()
#                if len(balloon_list)>0:
#                    # if multiple, find one most likely to be true.
#                    true_balloon = bf.pick_best_balloon(balloon_list)
#                    # find the vector to that balloon
#                    tvec = bf.find_vector(true_balloon)
#                    x = tvec[2] * .0254
#                    y = tvec[0] * .0254
#                    z = tvec[1] * .0254
#                    print "====Vector==================="
#                    print tvec
#                    print "============================="
#                    self.vehicle.mode = VehicleMode("GUIDED")
#                    time.sleep(2)
#                    print "Flying to way point"
#                    #self.goto_position_target_local_ned(x,y,z)
#                    #time.sleep(10)
#                    vec = [x,y,z]
#                    rotated_angle=0
 #                   return vec
 #               time.sleep(.033)
        print "Done with rotate"
        time.sleep(1)
        return []
   

    def test_check_center(self,bf):
        #bf = BalloonFinder()
        count = 0
        while count < 5: 
            print "Checking for Balloon"
            center_count = 0
            for i in range(30):
                im, balloon_list = bf.find_balloons()

                if len(balloon_list)>0:
                    # if multiple, find one most likely to be true.
                    true_balloon = bf.pick_best_balloon(balloon_list)
                    # find the vector to that balloon
                    tvec = bf.find_vector(true_balloon)
                    print "====Vector==================="
                    print tvec
                    print "============================="
                    x = tvec[2] * .0254
                    y = tvec[0] * .0254
                    z = tvec[1] * .0254
                    print "x: {},y: {},z {}".format(x,y,z)
                    is_center = self.check_center(y,z)
                    print "ballon is the Center: {}".format(is_center)
                    if(is_center):
                        center_count = center_count + 1
                        print "center_count: {}".format(center_count)        
                time.sleep(.033)
            count = count + 1
            if center_count > 5:
                print "###################### Balloon is centered #########################"
                return True
        return False

    

    def check_center(self,x,y):
        dst = math.sqrt(x*x+y*y)
        if dst < .4:
            return True
        else:
            return False

    def run(self):
        bf = BalloonFinder()
        timeout = 0
        filename = "waypoints.txt"
        num_waypoints = self.read_waypoints(filename)
        current_waypoint = 0
        self.take_off(6)
        time.sleep(2)
        home_location = LocationGlobalRelative(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,6)
        print "Home set as: {}".format(home_location)
        #time.sleep(3) 
        self.fly_to_waypoint(self.waypoints[current_waypoint])
        self.vehicle.mode = VehicleMode("AUTO")
        time.sleep(4)
        print "Rotating..."
        vec = self.rotate_and_check_for_balloon(15,bf)
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(2)
        if vec == []:
            self.fly_to_waypoint(home_location)
            print "############# Didn't find balloon!!! Flying Home. ###############"
            time.sleep(2)
            self.land()
        killed = False
        while not killed:
            timeout = 0
            start_position =  LocationGlobalRelative(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,self.vehicle.location.global_frame.alt)
            self.goto_position_target_local_ned(vec[0],vec[1],vec[2])
            distance = math.sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2]))
            distance_traveled = 0
            while distance > distance_traveled:
                if distance < 0.5:
                    print "Too close to track by gps, flying blind"
                    time.sleep(5)
                    break
                current_waypoint =  LocationGlobalRelative(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,self.vehicle.location.global_frame.alt)
                distance_traveled = self.get_distance_metres(start_position,current_waypoint)
                time.sleep(1)
            print "At new position, Checking for Balloon"
            vec = self.update_vector(bf)
            while vec == []:
                self.goto_position_target_local_ned(-1,0,0)
                print "Can't see balloon. Backing up...."
                time.sleep(5)
                vec = self.update_vector(bf)
                timeout = timeout + 1
                if timeout >= 5:
                    break
            if timeout >= 5:
                print "Can't find balloon. Flying home..."
                break
        self.fly_to_waypoint(home_location)
        time.sleep(1)
        self.land()



    def take_off_init(self):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print "Basic pre-arm checks"
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)

        print "Arming motors"
        # Copter should arm in GUIDED mode
        self.vehicle.mode    = VehicleMode("GUIDED")
        self.vehicle.armed   = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print " Waiting for arming..."
            time.sleep(1)

        print "Taking off!"
        self.vehicle.simple_takeoff(altitude) # Take off to target altitude


    def take_off_run(self):
        print " Altitude: ", self.vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if self.vehicle.location.global_relative_frame.alt>=altitude*0.95:
            print "Reached target altitude"

    


if __name__ == '__main__':
    print "running from command line"
    Drone()





