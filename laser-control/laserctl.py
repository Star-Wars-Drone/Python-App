import RPi.GPIO as GPIO
import time
from threading import Timer

class Laser:
    def __init__(self,pin=12,timeout=1):
        self.pin = pin #which board pin the laser control is wired to
        self.timeout = timeout #how long to keep laser on after trigger()
        GPIO.setmode(GPIO.BOARD) #use board-layout numbering
        GPIO.setup(self.pin,GPIO.OUT)
        GPIO.output(self.pin,False) #make sure it is off by default
        self.on = False #indicate state to ourselves
        self.timer = None
        print("Laser setup - pin: "+str(self.pin)+"; timeout: "+str(self.timeout))

    def trigger(self,timeout=None):
        if timeout == None: #no override
            timeout = self.timeout #use preset
        self.timer = Timer(timeout,self.switch_off,())
        self.timer.start()
        print("Laser on for " + str(timeout) + " seconds")
        GPIO.output(self.pin,True)
        self.on = True

    def switch_off(self):
#        while not self.sched.empty():
#           self.sched.
		print("Laser: switching off")
		if self.timer:
			self.timer.cancel()
			self.timer = None
		GPIO.output(self.pin,False)
		self.on = False


#    def timeout_callback(self):
#        print("Laser timeout")
#        self.switch_off()


if __name__ == "__main__":
    zap = Laser()
    zap.trigger(5)
    time.sleep(10)
    zap.trigger(1)
    time.sleep(5)
    print("Laser test done")
