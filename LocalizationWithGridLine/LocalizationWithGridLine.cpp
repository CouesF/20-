/*****************************************************
Name: LocalizationWithGridLine.cpp
Function: Localize the robot from the beginning of map. output in ros

Node Name: RobotPositionPublisher
Create Topic: RobotPositionInfo
******************************************************/
#include"opencv2/opencv.hpp"//TODO:correct the format
#include"opencv2/highgui/highgui.hpp"
//#include"opencv2/core/types.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <stdbool.h>

#define imgWidth 640
#define imgHeight 480
#define imgWidthCut 640
#define imgHeightCut 240
#define loopRate 30 // whole loop rate
#define imgPartitionSize 60//(pixels) divide the img into areas. make sure each part has only one cross

#define currentFrame 1
#define previousFrame 0
#define x 0
#define y 1
#define areaXCount imgWidth/imgPartitionSize+3
#define areaYCount imgWidth/imgPartitionSize+3


int main(int argc, char **argv)
{
    //data type initialize
    //TODO:
    /*
     * Mat cameraMatrix = (Mat1d(3,3) << fx, 0, cx, fy, cy, 0, 0, 1);
     * Mat distortionCoefficients = (Mat1d(1,4) << k1, k2, p1, p2);
     */
    cv::Mat image,greyImage;
    cv::VideoCapture cap;
    
    //Localization data
    //    The first parameter represents the frame order. 0-previous 1-current
    //    The last parameter represents x or y values
    bool isCrossExists[2][areaXCount][areaYCount];
    int localizationData[2][areaXCount][areaYCount][2];
    int coord[2][areaXCount][areaYCount][2];


    // cap setting
    // cap.set(cv::CV_CAP_PROP_FRAME_WIDTH,imgWidth);
    // cap.set(cv::CV_CAP_PROP_FRAME_HEIGHT,imgHeight);
    // cap.set(cv::CV_CAP_PROP_EXPOSURE,-5);//-5 means 40ms
    //

    // cam initialization
    cap.open(0);
    if(!cap.isOpened()){ 
        std::cout << "cam openning failed" << std::endl;
	return -1;
    }
    
    cap >> image;
    
    if(image.empty())
    {}
    
    cv::cvtColor(image,greyImage,cv::COLOR_RGB2GREY);
    //undistort(image,undistortedImg,cameraMatrix,distortionCoefficients);
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
