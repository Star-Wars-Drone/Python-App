import dronekit
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import os


connection_string = "udp:127.0.0.1:5760"
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = dronekit.connect(connection_string, wait_ready=True)
@vehicle.on_attribute('mode')
def mode_callback(self,attr_name,value):
    #attr_name not used here.
    print " CALLBACK: (%s)" % (value.name)
    if value.name == 'LAND':
        os._exit(1)
vehicle.add_attribute_listener('mode.name', mode_callback)
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
    time.sleep(10)
    vehicle.mode = dronekit.VehicleMode("LAND")
    while vehicle.armed == True:
        time.sleep(1)
vehicle.close()

