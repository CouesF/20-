/*****************************************************
Name: LocalizationWithGridLine.cpp
Function: Localize the robot from the beginning of map. output in ros

Node Name: RobotPositionPublisher
Create Topic: RobotPositionInfo
******************************************************/
#include"opencv2/opencv.hpp"//TODO:correct the format

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    //data type initialize
    //TODO:
    /*
     * Mat cameraMatrix = (Mat1d(3,3) << fx, 0, cx, fy, cy, 0, 0, 1);
     * Mat distortionCoefficients = (Mat1d(1,4) << k1, k2, p1, p2);
     * undistort(image,undistortedImg,cameraMatrix,distortionCoefficients);
     */ 
    cv::Mat image;
    cv::VideoCapture cap;
    

    // cam initialization
    cap.open(0);
    if(!cap.isOpened()){
        std::cout << "cam openning failed" << std::endl;
	return -1;
    }
    
    cap >> image;
    if(image.empty())
    {}
    cv::imshow("test",image);
    cv::waitKey(0);






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
    


    //while (ros::ok())
    //{



    //}
    return 0;
}
