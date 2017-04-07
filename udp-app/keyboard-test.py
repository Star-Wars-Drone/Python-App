from threading import Thread
from logging import debug  
from Sensor import Sensor
import time
from smbus import SMBus
import atexit

while True:
    try:
        keypress = raw_input("> ")
    except EOFError:
        print "bye"
    except (KeyboardInterrupt, SystemExit):
        print "\nExiting program\n"
        break
