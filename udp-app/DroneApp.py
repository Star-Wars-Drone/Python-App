
from threading import Thread
from logging import debug  
from Sensor import Sensor
import time
from smbus import SMBus
import atexit
from DroneRun import DroneRun

class DroneApp:
    """ Sensor Class for external sensors i2c"""

    def __init__(self):
        bus = SMBus(1)
        sensor = Sensor(bus)
        drone = DroneRun()
        sensor.start()
        drone.start()
        while True:
            try:
                keypress = raw_input("> ")
            except EOFError:
                print "bye"
                sensor.terminate()
                drone.land()
                drone.terminate()
                break
            except (KeyboardInterrupt, SystemExit):
                print "\nExiting program\n"
                sensor.terminate()
                drone.land()
                drone.terminate()
                break

    def __del_(self):
        sensor.stop()
        pass

if __name__ == '__main__':
    print "running from command line"
    DroneApp()
