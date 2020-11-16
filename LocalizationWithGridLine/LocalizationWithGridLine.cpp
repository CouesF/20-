/*****************************************************
Name: LocalizationWithGridLine.cpp
Function: Localize the robot from the beginning of map. output in ros

Node Name: RobotPositionPublisher
Create Topic: RobotPositionInfo
******************************************************/
#include<opencv>//TODO:correct the format

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    //data type initialize
    //TODO:

    //Ros configuration_publisher
    ros::init(argc,argv, "RobotPositionPublisher");
    ros::NodeHandle n;

    ros::Publisher PositionPublisher = 
        n.advertise<std_msgs::String>("RobotPositionInfo", 500);
    ros::Rate loop_rate (30);//max rate is 30 Hz. ImageProcess may slower than it.

    //set camera parameter: 
    //      1. black white
    //      2. exposure time
    //      3. image resolution
    //TODO:
    


    while (ros::ok())
    {



    }

}
