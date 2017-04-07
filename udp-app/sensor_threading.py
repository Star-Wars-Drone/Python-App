from threading import Thread
from logging import debug  
from Sensor import Sensor
import time
from smbus import SMBus

class SensorThreading:
    """ Sensor Class for external sensors i2c"""

    def __init__(self):
        bus = SMBus(1)
        sensor = Sensor(bus)
        sensor.start()
        

    def __del_(self):
        pass

if __name__ == '__main__':
    print "running from command line"
    SensorThreading()


