

    
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