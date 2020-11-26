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

using namespace cv;

//TODO IMPORTANT : 100pixels = 30CM (height 60cm, Direction 30 degrees)

Point2f perspectiveTransformOriginPoint[4] = {Point2f(155,0),Point2f(485,0),Point2f(0,320),Point2f(640,320)};
Point2f perspectiveTransformWarpedPointa[4] = {Point2f(0,0),Point2f(warpedWidth,0),Point2f(0,warpedHeight),Point2f(warpedWidth,warpedHeight)};
int xGridLinesFitting[int(maxLensInImg *2 + 20)];
int yGridLinesFitting[int(maxLensInImg *2 + 20)];
float gaussianPara[int(pixelsCntPerCentimeter*3+2)];
void calculateGaussianPara()
{
    double para = 1.0/sqrt(2.0*CV_PI);
    double gap = 4.0 / pixelsCntPerCentimeter / 3.0;
    for(int i = 1;i <= pixelsCntPerCentimeter * 3;i++)
    {
        gaussianPara[i] = para * exp(-pow((double)(-2+(double)i*gap),2)/2.0)*30.0;
    }
}


void gaussianSum(int pixelPosition,int k)// TODO:rewrite using array
{
    pixelPosition += maxLensInImg;
    int pixelTempPos;
    for(int i = 1;i <= pixelsCntPerCentimeter * 3 ;i++)
    {
        pixelTempPos = int(pixelPosition + i - pixelsCntPerCentimeter * 3.0 / 2.0 );
        //if(pixelTempPos > maxLensInImg || pixelTempPos < 1) continue;
        if(k == 0)
        {
            xGridLinesFitting[pixelTempPos] += double(gaussianPara[i]);
        }
        if(k == 1)
        {
            yGridLinesFitting[pixelTempPos] += double(gaussianPara[i]);
        }
    }
}

float getAngelOfTwoVector(Point2f &pt1, Point2f &pt2, Point2f &c)
{
	float theta = atan2(pt1.x - c.x, pt1.y - c.y) - atan2(pt2.x - c.x, pt2.y - c.y);
	if (theta > CV_PI)
		theta -= 2 * CV_PI;
	if (theta < -CV_PI)
		theta += 2 * CV_PI;
 
	//theta = theta * 180.0 / CV_PI;
	return theta;
}


int main(int argc, char **argv)
{
    //data type initialize
        //TODO:Mat cameraMatrix = (Mat1d(3,3) << fx, 0, cx, fy, cy, 0, 0, 1);
        //Mat distortionCoefficients = (Mat1d(1,4) << k1, k2, p1, p2);
    Mat img,originImg;
    Mat warpedImg,warpedGrayImg;

    short thresholdCnt = 151 ;//used for estimating the threshold value of canny edge detection. to save the resources of calculating while looping
    int cannyMinThreshold,cannyMaxThreshold;
    VideoCapture cap;
    
    //Localization data
        //    The first parameter represents the frame order. 0-previous 1-current
        //    The last parameter represents x or y values
        //bool isCrossExists[2][areaXCount][areaYCount];
        //int localizationData[2][areaXCount][areaYCount][2];
        //int coord[2][areaXCount][areaYCount][2];
    double xDirectionOfPreviousFrame = 0,yDirectionOfPreviousFrame = CV_PI/2;//theta in img.
    double robotGlobalDirection =  -CV_PI / 2; //robot direction in global map. Initial direction is -pi/2
    double robotGlobalX = 0,robotGlobalY = 0;
    int xGlobal = 0, yGlobal = 0;

    int xRho,yRho,xPreviousRho = 0,yPreviousRho = 0;

    //TODO: initial the point
    int previousCrossExists[8][8];//6*6means devide the img into 6*6 cells
    memset(previousCrossExists,0,sizeof(previousCrossExists));
    double previousCrossPosition[8][8][2];//0-x 1-y
    previousCrossExists[1][1] = 1;
    previousCrossExists[2][1] = 1;
    previousCrossExists[4][1] = 1;
    previousCrossExists[1][2] = 1;
    previousCrossExists[1][4] = 1;
    previousCrossExists[2][2] = 1;
    previousCrossExists[4][4] = 1;
    previousCrossPosition[1][1][0] = 10;
    previousCrossPosition[1][1][1] = 10;
    previousCrossPosition[2][1][0] = 110;
    previousCrossPosition[2][1][1] = 10;
    previousCrossPosition[4][1][0] = 210;
    previousCrossPosition[4][1][1] = 10;
    previousCrossPosition[1][2][0] = 10;
    previousCrossPosition[1][2][1] = 110;
    previousCrossPosition[1][4][0] = 10;
    previousCrossPosition[1][4][1] = 210;
    previousCrossPosition[2][2][0] = 110;
    previousCrossPosition[2][2][1] = 110;
    previousCrossPosition[4][4][0] = 210;
    previousCrossPosition[4][4][1] = 210;
    


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
