#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32, Bool, ColorRGBA
from sensor_msgs.msg import Range, BatteryState
import Servo, Buzzer, Led, Adc

cmd_vel_sub = None
v_pub = None



motor_controller = None
servo_controller = None


def cmd_vel_calllback(msg):
    rospy.loginfo("cmd_vel_callback")
    motor_controller.twist(msg)
    
    
def voltage_check():
    voltage=voltage_controller.recvADC(2)*3
    msg = BatteryState()
    msg.voltage = voltage
    msg.percentage = voltage/8.4
    v_pub.publish(msg)


    
def loop():
    while not rospy.is_shutdown():
        voltage_check()
        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('robot_driver', anonymous=True)
    rospy.Rate(10)
    
    rospy.logdebug ('Program is starting ... ')
    
    #init controllers
    #motor_controller = Motor() this doesnt compile
    servo_controller = Servo()
    voltage_controller = Adc()
    
    
    #init publishers
    v_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
    range_pub = rospy.Publisher('/ultrasonic', Range, queue_size=10)
    
    
    
    #init subscribers
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Int32, cmd_vel_callback)
    
    servo_sub_x = rospy.Subscriber('/servo/x', Int32, servo_x_callback)
    servo_sub_y = rospy.Subscriber('/servo/y', Int32, servo_y_callback)
    buzzer_sub = rospy.Subscriber('/buzzer', Bool, buzzer_callback)
   
    # led_sub = rospy.Subscriber('/led', ColorRGBA, led_callback)
    
    loop()