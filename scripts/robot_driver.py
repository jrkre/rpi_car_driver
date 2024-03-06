#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import ColorRGBA
# import Adc, Ultrasonic, Buzzer, Servo #, Led
from Adc import Adc
from Ultrasonic import Ultrasonic
from Buzzer import Buzzer
from Servo import Servo
# from Led import Led implementation is broken


servo, ultrasonic, adc, buzzer = None, None, None, None

voltage_pub = None
ultrasonic_pub = None

servo_sub_x = None
servo_sub_y = None
buzzer_sub = None
cmd_vel_sub = None
# led_sub = None



    

def buzzer_callback(msg):
    global buzzer
    #do buzzer stuff
    
    
    if (msg.data):
        buzzer.run("1")
    else:
        buzzer.run("0")

# def led_callback(msg):
#     #do led stuff
#     led = Led()
#     led.colorWipe(msg.r, msg.g, msg.b, msg.a)
    
    
def main():
    while not rospy.is_shutdown():
        update_sensors()
        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('robot_driver', anonymous=True)
    rospy.Rate(10)
    

    rospy.logdebug ('Program is starting ... ')
    
    voltage_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
    ultrasonic_pub = rospy.Publisher('/ultrasonic', Range, queue_size=10)
    
    
    servo_sub_x = rospy.Subscriber('/servo/x', Int32, servo_x_callback)
    servo_sub_y = rospy.Subscriber('/servo/y', Int32, servo_y_callback)
    buzzer_sub = rospy.Subscriber('/buzzer', Bool, buzzer_callback)
   
    # led_sub = rospy.Subscriber('/led', ColorRGBA, led_callback)
    
    main()