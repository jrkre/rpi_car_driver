#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import ColorRGBA
import Adc, Ultrasonic, Motor, Buzzer, Led


cmd_vel_sub = None
voltage_pub = None
ultrasonic_pub = None
buzzer_sub = None
led_sub = None


def cmd_vel_callback(msg):
    #do motor stuff
    print("cmd_vel_callback")
    

def buzzer_callback(msg):
    #do buzzer stuff
    buzzer = Buzzer()
    
    if (msg.data):
        buzzer.run("1")
    else:
        buzzer.run("0")

def led_callback(msg):
    #do led stuff
    led = Led()
    led.colorWipe(msg.r, msg.g, msg.b, msg.a)
    
def update_sensors():
    adc = Adc()
    ultrasonic = Ultrasonic()
    
    voltage = adc.recvADC(2)*3
    ultrasonic_m = ultrasonic.get_distance()
    

if "___main__" == __name__:
    rospy.init_node('robot_driver', anonymous=True)
    
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    voltage_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
    ultrasonic_pub = rospy.Publisher('/ultrasonic', Range, queue_size=10)
    buzzer_sub = rospy.Subscriber('/buzzer', Bool, buzzer_callback)
    led_sub = rospy.Subscriber('/led', ColorRGBA, led_callback)
    
    update_sensors()
    
    rospy.spin()