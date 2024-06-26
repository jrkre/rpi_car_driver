#include "ros/ros.h"


#include "Encoder.h"


/** front
 * 1 ==== 2
 *  |    |
 *  |    |
 *  |    |
 * 3 ==== 4
 *   back
 */



Encoder motor1_encoder(22);
//Encoder motor2_encoder(18, [](int i){std::cout << "Encoder:" << i << std::endl;});
//Encoder motor3_encoder(27, [](int i){std::cout << "Encoder:" << i << std::endl;});
//Encoder motor4_encoder(22, [](int i){std::cout << "Encoder:" << i << std::endl;});

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpi_car_firmware");

    ROS_INFO("Starting rpi_car_firmware node");

    ros::NodeHandle nh;

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}