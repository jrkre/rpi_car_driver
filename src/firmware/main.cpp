#include "ros/ros.h"
#include <pigpiod_if2.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "Encoder.h"


/** front
 * 1 ==== 2
 *  |    |
 *  |    |
 *  |    |
 * 3 ==== 4
 *   back
 */




//Encoder motor2_encoder(18, [](int i){std::cout << "Encoder:" << i << std::endl;});
//Encoder motor3_encoder(27, [](int i){std::cout << "Encoder:" << i << std::endl;});
//Encoder motor4_encoder(22, [](int i){std::cout << "Encoder:" << i << std::endl;});

int main(int argc, char **argv)
{
    int pi;
    if (pi = pigpio_start(NULL, NULL) >= 0)
    {
        ROS_INFO("GPIO Initialized\n  pi: %i", pi);
    }
    else
    {
        ROS_ERROR("GPIO FAILED TO INITIALISE");
        ROS_ERROR("%i", pi);
    }

    ros::init(argc, argv, "rpi_car_firmware");

    ROS_INFO("Starting rpi_car_firmware node");

    usleep(5000);

    Encoder motor1_encoder(pi, 13);

    motor1_encoder.setPosition(0);

    ros::NodeHandle nh;

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
