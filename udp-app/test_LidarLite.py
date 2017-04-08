from LidarLiteV3 import LidarLiteV3
from smbus import SMBus
import time
bus = SMBus(1)
range_sensor = LidarLiteV3(bus)
range_sensor.begin(0,0x62)
while True:
    dest = range_sensor.get_distance(False,0x62)
    print dest 
    time.sleep(.05)

