#!/usr/bin/env python3

#mostly taken from https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/blob/master/Code/Server/ADC.py

import rospy
from sensor_msgs.msg import BatteryState
import smbus
import time

pub = None

class Adc:
    def __init__(self):
        # Get I2C bus
        self.bus = smbus.SMBus(1)
        
        # I2C address of the device
        self.ADDRESS            = 0x48
        
        # PCF8591 Command
        self.PCF8591_CMD                        =0x40  #Command
        
        # ADS7830 Command 
        self.ADS7830_CMD                        = 0x84 # Single-Ended Inputs
        
        for i in range(3):
            aa=self.bus.read_byte_data(self.ADDRESS,0xf4)
            if aa < 150:
                self.Index="PCF8591"
            else:
                self.Index="ADS7830" 
    def analogReadPCF8591(self,chn):#PCF8591 read ADC value,chn:0,1,2,3
        value=[0,0,0,0,0,0,0,0,0]
        for i in range(9):
            value[i] = self.bus.read_byte_data(self.ADDRESS,self.PCF8591_CMD+chn)
        value=sorted(value)
        return value[4]   
        
    def analogWritePCF8591(self,value):#PCF8591 write DAC value
        self.bus.write_byte_data(self.ADDRESS,cmd,value)
        
    def recvPCF8591(self,channel):#PCF8591 write DAC value
        while(1):
            value1 = self.analogReadPCF8591(channel)   #read the ADC value of channel 0,1,2,
            value2 = self.analogReadPCF8591(channel)
            if value1==value2:
                break
        voltage = value1 / 256.0 * 3.3  #calculate the voltage value
        voltage = round(voltage,2)
        return voltage
    def recvADS7830(self,channel):
        """Select the Command data from the given provided value above"""
        COMMAND_SET = self.ADS7830_CMD | ((((channel<<2)|(channel>>1))&0x07)<<4)
        self.bus.write_byte(self.ADDRESS,COMMAND_SET)
        while(1):
            value1 = self.bus.read_byte(self.ADDRESS)
            value2 = self.bus.read_byte(self.ADDRESS)
            if value1==value2:
                break;
        voltage = value1 / 255.0 * 3.3  #calculate the voltage value
        voltage = round(voltage,2)
        return voltage
        
    def recvADC(self,channel):
        if self.Index=="PCF8591":
            data=self.recvPCF8591(channel)
        elif self.Index=="ADS7830":
            data=self.recvADS7830(channel)
        return data
    def i2cClose(self):
        self.bus.close()

def loop():
    global pub
    adc=Adc()
    while True:
        voltage=adc.recvADC(2)*3
        battery_state = BatteryState()
        battery_state.header.stamp = rospy.Time.now()
        battery_state.voltage = voltage
        pub.publish(battery_state)
        
        
def destroy():
    pass


# Main program logic follows:
if __name__ == '__main__':
    rospy.init_node('voltage_monitor', anonymous=True)
    rospy.Rate(10)
    
    pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
    
    try:
        loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()
    except rospy.ROSInterruptException:
        pass