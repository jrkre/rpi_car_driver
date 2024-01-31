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


voltage_pub = None
ultrasonic_pub = None

servo_sub = None
buzzer_sub = None
cmd_vel_sub = None
# led_sub = None


def servo_callback(msg):
    #do servo stuff
    print("servo_callback")
    servo = Servo()
    
    servo.setServoPwm('0', msg.data)


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

# def led_callback(msg):
#     #do led stuff
#     led = Led()
#     led.colorWipe(msg.r, msg.g, msg.b, msg.a)
    
def update_sensors():
    global voltage_pub, ultrasonic_pub
    adc = Adc()
    ultrasonic = Ultrasonic()
    
    voltage = adc.recvADC(2)*3
    ultrasonic_m = ultrasonic.get_distance() * 0.01
    
    voltage_msg = BatteryState()
    voltage_msg.voltage = voltage
    voltage_msg.percentage = voltage/8.4
    voltage_msg.header.stamp = rospy.Time.now()
    voltage_pub.publish(voltage_msg)
    
    ultrasonic_msg = Range()
    ultrasonic_msg.range = ultrasonic_m
    ultrasonic_msg.header.stamp = rospy.Time.now()
    ultrasonic_pub.publish(ultrasonic_msg)
    
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
    
    servo_sub = rospy.Subscriber('/servo', Int32, servo_callback)
    buzzer_sub = rospy.Subscriber('/buzzer', Bool, buzzer_callback)
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    # led_sub = rospy.Subscriber('/led', ColorRGBA, led_callback)
    
    main()