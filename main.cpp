/*****************************************************
Name: LocalizationWithGridLine.cpp
Function: Localize the robot from the beginning of map. output in ros

Node Name: RobotPositionPublisher
Create Topic: RobotPositionInfo
******************************************************/
#include"opencv2/opencv.hpp"
#include"opencv2/highgui/highgui.hpp"
//#include"opencv2/core/types.hpp"
#include"opencv2/imgproc.hpp"//cvtColor

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <stdbool.h>
#include <vector>
#include <math.h>
#include <fstream> //for debug. output to a file
#include <string.h> // memset


#define imgWidth 640
#define imgHeight 360
#define imgWidthCut 640
#define imgHeightCut 210
#define warpedWidth 382
#define warpedHeight 315
#define loopRate 30 // whole loop rate
#define imgPartitionSize 60//(pixels) divide the img into areas. make sure each part has only one cross
#define gaussianBlurSize 11
#define pixelsCntPerCentimeter 3.33//DONE: 100p = 30cm
#define houghLineThreshold pixelsCntPerCentimeter*20.0
#define rotationThreshold (double)(25.0/180.0*CV_PI)
#define maxLensInImg (double)(sqrt(pow(warpedWidth,2)+pow(warpedHeight,2)) + 4.0)
#define currentFrame 1
#define previousFrame 0
#define areaXCount imgWidth/imgPartitionSize+3
#define areaYCount imgWidth/imgPartitionSize+3
#define gaussianSumMax 800//used for debug draw on canvas
int cutX1 = 10,
    cutY1 = 10, 
    cutX2 = 300, 
    cutY2 = 300;

Rect imgResize(cutX1,cutY1,cutX2,cutY2);//TODO: 

using namespace cv;

//TODO: IMPORTANT : 100pixels = 30CM (height 60cm, Direction 30 degrees)


Point circleCentralPointDetection(Mat img)
{
    Mat croppedImg = img(imgResize);
    Mat grayImg,blurredImg,mThres_Gray;
    cvtColor(croppedImg,grayImg,COLOR_RGB2GRAY);
    medianBlur(grayImg,blurredImg,3);
    std::vector<Vec3f>circles;
    cannyMaxThreshold = min(200,(int)(threshold(blurredImg,mThres_Gray,0,255,THRESH_OTSU)));
    HoughCircles(blurredImg,circles,HOUGH_GRADIENT,1,0,cannyMaxThreshold,80,120)//TODO: the last 2 paras is the min & max radius of circle
    Mat centerCanvas(cutX2 - cutX1 , cutY2 - cutY1,CV_8UC1,Scalar(0));
    for(size_t i = 0; i < circles.size(); i++)
    {
        Vec3i c = circles[i];
        Point center = Point(c[0],c[1]);
        uchar *rowData = centerCanvas.ptr<uchar>(c[0]);
        rowData[c[1]] = Scalar(255);
        //int x = circles[i]
    }
    imshow("centerCanvas",centerCanvas);
    GaussianBlur(centerCanvas,centerCanvas,Size(5,5),0);
    imshow("blur",centerCanvas);
    double minVal;
    double maxVal;
    Point minIdx;
    Point maxIdx;
    minMaxLoc(centerCanvas,&minVal, &maxVal, &minIdx, &maxIdx);
    circle(croppedImg,maxIdx,3,(0,0,255),0)
    imshow("circleDetect",croppedImg);
    waitKey(0);

}


int main(int argc, char **argv)
{
    VideoCapture cap;

    // cap initialization and setting
    cap.open(0);
    cap.set(CAP_PROP_FRAME_WIDTH,imgWidth);
    cap.set(CAP_PROP_FRAME_HEIGHT,imgHeight);
    if(!cap.isOpened()){ 
        std::cout << "cam openning failed" << std::endl;
	return -1;
    }
    system("v4l2-ctl -d /dev/video0 -c exposure_auto=1");
    system("v4l2-ctl -d /dev/video0 -c exposure_absolute=78");//minimum is 78
    system("v4l2-ctl -d /dev/video0 -c brightness=60");//minimum is 78
    system("v4l2-ctl -d /dev/video0 -c contrast=30");//minimum is 78
    std::cout<<"default exposure: "<<cap.get(CAP_PROP_EXPOSURE)<<std::endl;
    std::cout<<"default contrast: "<<cap.get(CAP_PROP_CONTRAST)<<std::endl;
    std::cout<<"default brightne: "<<cap.get(CAP_PROP_BRIGHTNESS)<<std::endl;
    //cv::cvtColor(img,grayImg,cv::COLOR_RGB2GRAY);
    //imshow("gray",grayImg);
    if(img.empty()){}
    //undistort(image,undistortedImg,cameraMatrix,distortionCoefficients);
    
    calculateGaussianPara();
   



    //Ros configuration_publisher
    ros::init(argc,argv, "RobotPositionPublisher");
    ros::NodeHandle n;

    ros::Publisher PositionPublisher = 
        n.advertise<std_msgs::String>("RobotPositionInfo", 500);
    ros::Rate loop_rate (loopRate);//max rate is 30 Hz. ImageProcess may slower than it.

    

    

    while (ros::ok())
    {
	    //get the stream and cut the im
        //for(int i = 1;i <= 1;i++)
            cap.grab();
        //while(cap.grab()){}
        cap >> originImg;


    }

}
