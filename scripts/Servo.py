#!/usr/bin/env python3

from PCA9685 import PCA9685
import rospy
from std_msgs.msg import Int32


class Servo:
    def __init__(self):
        self.PwmServo = PCA9685(0x40, debug=True)
        self.PwmServo.setPWMFreq(50)
        self.PwmServo.setServoPulse(8, 1500)
        self.PwmServo.setServoPulse(9, 1500)

    def setServoPwm(self, channel, angle, error=10):
        angle = int(angle)
        if channel == '0':
            self.PwmServo.setServoPulse(8, 2500 - int((angle + error) / 0.09))
        elif channel == '1':
            self.PwmServo.setServoPulse(9, 500 + int((angle + error) / 0.09))
        elif channel == '2':
            self.PwmServo.setServoPulse(10, 500 + int((angle + error) / 0.09))
        elif channel == '3':
            self.PwmServo.setServoPulse(11, 500 + int((angle + error) / 0.09))
        elif channel == '4':
            self.PwmServo.setServoPulse(12, 500 + int((angle + error) / 0.09))
        elif channel == '5':
            self.PwmServo.setServoPulse(13, 500 + int((angle + error) / 0.09))
        elif channel == '6':
            self.PwmServo.setServoPulse(14, 500 + int((angle + error) / 0.09))
        elif channel == '7':
            self.PwmServo.setServoPulse(15, 500 + int((angle + error) / 0.09))


servo = Servo()

def servo_x_callback(msg):
    global servo
    #do servo stuff
    print("servo_callback")
    
    servo.setServoPwm('0', msg.data)

def servo_y_callback(msg):
    global servo
    print("servo_callback")
    
    servo.setServoPwm('1', msg.data)



# Main program logic follows:
if __name__ == '__main__':
    rospy.init_node('servo_controller', anonymous=True)
    rospy.Rate(10)
    
    while True:
        try:
            servo_sub_x = rospy.Subscriber('/servo/x', Int32, servo_x_callback)
            servo_sub_y = rospy.Subscriber('/servo/y', Int32, servo_y_callback)
        except KeyboardInterrupt:
            print("\nEnd of program")
            break
        except rospy.ROSInterruptException:
            pass