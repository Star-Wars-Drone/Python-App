from threading import Thread
from logging import debug  
from Sensor import Sensor
import time

class SensorThreading:
    """ Sensor Class for external sensors i2c"""

    def __init__(self):
        sensor = Sensor()
        sensor.start()
        time.sleep(2)

    def __del_(self):
        pass

if __name__ == '__main__':
    print "running from command line"
    SensorThreading()


