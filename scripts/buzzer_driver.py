#!/usr/bin/env python3

#taken mostly from https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/blob/master/Code/Server/Buzzer.py

import time
import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO


buzzer = None
GPIO.setwarnings(False)
Buzzer_Pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_Pin,GPIO.OUT)

class Buzzer:
    def run(self,command):
        if command!="0":
            GPIO.output(Buzzer_Pin,True)
        else:
            GPIO.output(Buzzer_Pin,False)
            
def callback(msg):
    global buzzer
    if msg.data:
        buzzer.run("1")
    else:
        buzzer.run("0")
            
if __name__=='__main__':
    rospy.init_node('buzzer_driver', anonymous=True)
    buzzer=Buzzer()
    rospy.Subscriber('/buzzer', Bool, callback)
    rospy.spin()
