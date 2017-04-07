from multiprocessing import Process
from LidarLiteV3 import LidarLiteV3
import time
class Sensor(Process):
    """ Sensor Class for external sensors i2c"""
    def __init__(self,bus):
        Process.__init__(self)
        self.bus = bus
        self.range_sensor = LidarLiteV3(bus)
        self.range_sensor.begin(4,0x62)

    def run(self):
        while True:
            self.range_sensor.get_distance(False,0x62)
            time.sleep(.05)
