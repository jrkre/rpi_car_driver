#!/usr/bin/env python3

import rospy
import math
from tf2_ros import tf2 as tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry



class RobotBase:
    linear_velocity_x, linear_velocity_y, angualar_velocity_z = 0, 0, 0
    delta_v_time = rospy.Time()
    vel_dt = 0
    x_pos, y_pos = 0, 0
    
    steering_angle = 0
    heading = 0
    
    odom_pub = None
    velocity_sub = None
    
    odom_quaternoion = Quaternion()
    odom_transform = TransformStamped()
    
    def __init__(self):
        odom_pub = rospy.Publisher('/raw_odom', Odometry, queue_size=10)
        velocity_sub = rospy.Subscriber('/raw_vel', Twist, self.vel_callback)
        pass
    
    def update(self):
        
        pass
    
    def vel_callback(self, msg):
        # a lot of this code is heavily based on www.github.com/linorobot/linorobot
        current_time = rospy.Time.now()
        
        self.linear_velocity_x = msg.linear.x
        self.linear_velocity_y = msg.linear.y
        self.angualar_velocity_z = msg.angular.z
        
        self.vel_dt = (current_time.secs - self.delta_v_time.secs)
        print("vel_dt:" + self.vel_dt)
        self.delta_v_time = current_time
        
        delta_heading = self.angualar_velocity_z * self.vel_dt
        
        delta_x = (self.linear_velocity_x * math.cos(self.heading) - self.linear_velocity_y * math.sin(self.heading)) * self.vel_dt
        delta_y = (self.linear_velocity_x * math.cos(self.heading) + self.linear_velocity_y * math.cos(self.heading)) * self.vel_dt
        
        self.x_pos += delta_x
        self.y_pos += delta_y
        self.heading += delta_heading
        
        self.odom_quaternoion = tf.transformations.quaternion_from_euler(0, 0, self.heading)
        
        #publish odom
        self.odom_transoform.header.stamp = current_time
        self.odom_transform.header.frame_id = "odom"
        self.odom_transform.child_frame_id = "base_link"
        
        self.odom_transform.transform.translation.x = self.x_pos
        self.odom_transform.transform.translation.y = self.y_pos
        self.odom_transform.transform.translation.z = 0
        
        self.odom_transform.transform.rotation = self.odom_quaternoion
        #self.odom_pub.publish(self.odom_transform)
        
        odom=Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        #x,y,z coords
        odom.pose.pose.position.x = self.x_pos
        odom.pose.pose.position.y = self.y_pos
        odom.pose.pose.position.z = 0
        
        #heading
        odom.pose.pose.orientation = self.odom_quaternoion
        odom.poes.covariance[9] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01
        
        #linear speed from encoders
        odom.twist.twist.linear.x = self.linear_velocity_x
        odom.twist.twist.linear.y = self.linear_velocity_y
        odom.twist.twist.linear.z = 0
        
        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.y = 0
        
        #angular speed from encoders
        odom.twist.twist.angular.z = self.angualar_velocity_z
        
        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[7] = 0.0001
        odom.twist.covariance[35] = 0.0001

        self.odom_pub.publish(odom)
    
robotBase = None

def main():
    global robotBase
    while not rospy.is_shutdown():
        
        rospy.spin()

if __name__ == '__main__':
    global robotBase
    rospy.init_node('robot_driver', anonymous=True)
    print('\'/robotbase\' is initializing ... ')
    rospy.Rate(10)
    

    rospy.logdebug ('Program is starting ... ')
    
    robotBase = RobotBase()
    # voltage_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
    # ultrasonic_pub = rospy.Publisher('/ultrasonic', Range, queue_size=10)
    
    
    # servo_sub_x = rospy.Subscriber('/servo/x', Int32, servo_x_callback)
    # servo_sub_y = rospy.Subscriber('/servo/y', Int32, servo_y_callback)
    # buzzer_sub = rospy.Subscriber('/buzzer', Bool, buzzer_callback)
   
    # led_sub = rospy.Subscriber('/led', ColorRGBA, led_callback)
    
    main()
