#!/usr/bin/env python3

#taken mostly from https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/blob/master/Code/Server/Motor.py

import math
import rospy
from PCA9685 import PCA9685
from geometry_msgs.msg import Twist
import Adc
import time


class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.Twist = Twist()
        self.time_proportion = 2.5  # Depend on your own car,If you want to get the best out of the rotation mode,
        # change the value by experimenting.
        self.adc = Adc()

    @staticmethod
    def duty_range(duty1, duty2, duty3, duty4):
        if duty1 > 4095:
            duty1 = 4095
        elif duty1 < -4095:
            duty1 = -4095

        if duty2 > 4095:
            duty2 = 4095
        elif duty2 < -4095:
            duty2 = -4095

        if duty3 > 4095:
            duty3 = 4095
        elif duty3 < -4095:
            duty3 = -4095

        if duty4 > 4095:
            duty4 = 4095
        elif duty4 < -4095:
            duty4 = -4095
        return duty1, duty2, duty3, duty4

    def left_Upper_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(0, 0)
            self.pwm.setMotorPwm(1, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(1, 0)
            self.pwm.setMotorPwm(0, abs(duty))
        else:
            self.pwm.setMotorPwm(0, 4095)
            self.pwm.setMotorPwm(1, 4095)

    def left_Lower_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(3, 0)
            self.pwm.setMotorPwm(2, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(2, 0)
            self.pwm.setMotorPwm(3, abs(duty))
        else:
            self.pwm.setMotorPwm(2, 4095)
            self.pwm.setMotorPwm(3, 4095)

    def right_Upper_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(6, 0)
            self.pwm.setMotorPwm(7, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(7, 0)
            self.pwm.setMotorPwm(6, abs(duty))
        else:
            self.pwm.setMotorPwm(6, 4095)
            self.pwm.setMotorPwm(7, 4095)

    def right_Lower_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(4, 0)
            self.pwm.setMotorPwm(5, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(5, 0)
            self.pwm.setMotorPwm(4, abs(duty))
        else:
            self.pwm.setMotorPwm(4, 4095)
            self.pwm.setMotorPwm(5, 4095)

    def setMotorModel(self, duty1, duty2, duty3, duty4):
        duty1, duty2, duty3, duty4 = self.duty_range(duty1, duty2, duty3, duty4)
        self.left_Upper_Wheel(duty1)
        self.left_Lower_Wheel(duty2)
        self.right_Upper_Wheel(duty3)
        self.right_Lower_Wheel(duty4)

    def Rotate(self, n):
        angle = n
        bat_compensate = 7.5 / (self.adc.recvADC(2) * 3)
        while True:
            W = 2000

            VY = int(2000 * math.cos(math.radians(angle)))
            VX = -int(2000 * math.sin(math.radians(angle)))

            FR = VY - VX + W
            FL = VY + VX - W
            BL = VY - VX - W
            BR = VY + VX + W

            PWM.setMotorModel(FL, BL, FR, BR)
            print("rotating")
            time.sleep(5 * self.time_proportion * bat_compensate / 1000)
            angle -= 5



PWM = None
WHEEL_GEOMETRY = None
WHEEL_RADIUS = None





def destroy():
    PWM.setMotorModel(0, 0, 0, 0)
    
def cmd_vel_callback(msg):
    global PWM
    #do motor stuff
    print("motor_callback")
    x = msg.linear.x
    y = msg.linear.y
    rot = msg.angular.z

    front_left = (x - y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    front_right = (x + y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_left = (x + y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_right = (x - y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    
    PWM.setMotorModel(front_left, back_left, front_right, back_right)
    
    

def loop():
    global WHEEL_RADIUS, WHEEL_GEOMETRY, PWM
    rospy.init_node('motor_controller', anonymous=True)
    rospy.Rate(10)
    
    PWM = Motor()
    
    WHEEL_GEOMETRY = (rospy.param('/robot/wheel/separation/horizontal') + rospy.param('/robot/wheel/separation/vertical')) / 2
    WHEEL_RADIUS = rospy.param('/robot/wheel/diameter') / 2

    while True:
        try:
            cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
        except KeyboardInterrupt:
            print("\nEnd of program")
            

if __name__ == '__main__':
    try:
        loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()
    except rospy.ROSInterruptException:
        destroy()
        pass