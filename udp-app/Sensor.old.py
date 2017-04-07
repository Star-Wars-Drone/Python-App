from threading import Thread
from logging import debug
from LidarLiteV3 import LidarLiteV3
import time
class Sensor(Thread):
    """ Sensor Class for external sensors i2c"""
    def __init__(self,bus):
        super(Sensor,self).__init__()
        self.bus = bus
        self.range_sensor = LidarLiteV3(bus)
        self.range_sensor.begin(4,0x62)

        pass

    def run(self):
        while True:
            self.range_sensor.get_distance(False,0x62)
            time.sleep(.05)
