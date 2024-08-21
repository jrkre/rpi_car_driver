#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32, Bool, ColorRGBA
from sensor_msgs.msg import Range, BatteryState
from geometry_msgs.msg import Twist
from Motor import Motor
from Buzzer import Buzzer
from Adc import Adc
from Servo import Servo
from Ultrasonic import Ultrasonic

#subscibers
cmd_vel_sub = None
servo_sub_x = None
servo_sub_y = None

#publishers
v_pub = None
range_pub = None

#controllers
motor_controller = None
servo_controller = None
ultrasonic_controller = None
buzzer_controller = None


def cmd_vel_callback(msg):
    rospy.loginfo("cmd_vel_callback")
    motor_controller.twist(msg)

def buzzer_callback(msg):
    buzzer_controller.callback(msg)
    
    
def voltage_check():
    voltage=voltage_controller.recvADC(2)*3
    msg = BatteryState()
    msg.voltage = voltage
    msg.percentage = voltage/8.4
    v_pub.publish(msg)


def range_check():
    distance = ultrasonic_controller.get_distance()
    msg = Range()
    msg.header.stamp = rospy.Time.now()
    msg.range = distance
    range_pub.publish(msg)

    
def loop():
    while not rospy.is_shutdown():
        voltage_check()
        range_check()
        rospy.sleep(.1)

if __name__ == '__main__':
    rospy.init_node('robot_driver', anonymous=True)
    rospy.Rate(10)
    
    rospy.logdebug ('Program is starting ... ')
    
    #init controllers
    motor_controller = Motor()
    servo_controller = Servo()
    voltage_controller = Adc()
    ultrasonic_controller = Ultrasonic()
    buzzer_controller = Buzzer()
    
    
    #init publishers
    v_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
    range_pub = rospy.Publisher('/ultrasonic', Range, queue_size=10)
    
    
    
    #init subscribers
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    
    #servo_sub_x = rospy.Subscriber('/servo/x', Int32, servo_x_callback)
    #servo_sub_y = rospy.Subscriber('/servo/y', Int32, servo_y_callback)
    buzzer_sub = rospy.Subscriber('/buzzer', Bool, buzzer_callback)
   
    loop()