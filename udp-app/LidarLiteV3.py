import smbus
import time

class LidarLiteV3():

    def __init__(self,bus):
        self.bus = bus

    def __del__(self):
        pass

    def begin(self,config,lladdress):
        self.configure(config,lladdress)

    def configure(self,config,lladdress):
        if(config == 0):
            self.bus.write_byte_data(lladdress,0x02,0x80)
            self.bus.write_byte_data(lladdress,0x04,0x08)
            self.bus.write_byte_data(lladdress,0x1c,0x00)
            print "Configuation 0: Default"
        elif(config == 1):
            self.bus.write_byte_data(lladdress,0x02,0x1d)
            self.bus.write_byte_data(lladdress,0x04,0x08)
            self.bus.write_byte_data(lladdress,0x1c,0x00)
        elif(config == 2):
            self.bus.write_byte_data(lladdress,0x02,0x80)
            self.bus.write_byte_data(lladdress,0x04,0x00)
            self.bus.write_byte_data(lladdress,0x1c,0x00)
        elif(config == 3):
            self.bus.write_byte_data(lladdress,0x02,0xff)
            self.bus.write_byte_data(lladdress,0x04,0x08)
            self.bus.write_byte_data(lladdress,0x1c,0x00)
        elif(config == 4):
            self.bus.write_byte_data(lladdress,0x02,0x80)
            self.bus.write_byte_data(lladdress,0x04,0x08)
            self.bus.write_byte_data(lladdress,0x1c,0x80)
        elif(config == 5):
            self.bus.write_byte_data(lladdress,0x02,0x80)
            self.bus.write_byte_data(lladdress,0x04,0x08)
            self.bus.write_byte_data(lladdress,0x1c,0xb0)

    def reset(self,lladdress):
        self.bus.write_byte_data(lladdress,0x00,0x00)

    def get_distance(self,bias_correction,lladdress):
        busy_flag = 0
        if(bias_correction):
            self.bus.write_byte_data(lladdress,0x00,0x04)   
        else:
            self.bus.write_byte_data(lladdress,0x00,0x03)
        #while busy_flag != 0:
        #    busy_flag = self.bus.read_byte_data(lladdress,0x01)
        #    print busy_flag
        #    time.sleep(.1)
        if(busy_flag == 0):
            distance = self.bus.read_word_data(lladdress,0x8f)
            temp_low = distance & 0xff
            temp_high = distance & 0xff00
            temp_low = temp_low << 8
            temp_high = (temp_high >> 8) & 0xff;
            distance = temp_high | temp_low
            print distance






