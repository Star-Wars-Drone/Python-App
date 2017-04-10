from threading import Thread
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import sys
import tty
import termios
import os

class DroneRun(Thread):
    """ Sensor Class for external sensors i2c"""
    def __init__(self):
        Thread.__init__(self)
        print "Creating Drone Object"

    def run(self):
        print "Drone is live"
        isRunning = True
        while isRunning:
           
           
