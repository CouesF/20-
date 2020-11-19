/*****************************************************
Name: LocalizationWithGridLine.cpp
Function: Localize the robot from the beginning of map. output in ros

Node Name: RobotPositionPublisher
Create Topic: RobotPositionInfo
******************************************************/
#include"opencv2/opencv.hpp"//TODO:correct the format
#include"opencv2/highgui/highgui.hpp"
//#include"opencv2/core/types.hpp"
#include"opencv2/imgproc.hpp"//cvtColor

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <stdbool.h>
#include <vector>

#define imgWidth 640
#define imgHeight 360
#define imgWidthCut 640
#define imgHeightCut 210
#define loopRate 30 // whole loop rate
#define imgPartitionSize 60//(pixels) divide the img into areas. make sure each part has only one cross
#define gaussianBlurSize 11
#define pixelsCntPerCentimeter 15
#define houghLineThreshold pixelsCntPerCentimeter*7

#define currentFrame 1
#define previousFrame 0
#define areaXCount imgWidth/imgPartitionSize+3
#define areaYCount imgWidth/imgPartitionSize+3

using namespace cv;
void drawLine(Vec2f _line, Mat &img)
{
    if(_line[1]!=0)
    {
        float m = -1/tan(_line[1]);

        float c = _line[0]/sin(_line[1]);

        line(img, Point(0, c), Point(img.size().width, m*img.size().width+c),CV_RGB(0,0,255));
    }
    else
    {
        line(img, Point(_line[0], 0), Point(_line[0], img.size().height), CV_RGB(0,0,255));
    }

}

int main(int argc, char **argv)
{
    //data type initialize
    //TODO:
    /*
     * Mat cameraMatrix = (Mat1d(3,3) << fx, 0, cx, fy, cy, 0, 0, 1);
     * Mat distortionCoefficients = (Mat1d(1,4) << k1, k2, p1, p2);
     */
    Mat img,originImg;
    short thresholdCnt = 151 ;//used for estimating the threshold value of canny edge detection. to save the resources of calculating while looping
    int cannyMinThreshold,cannyMaxThreshold;
    VideoCapture cap;

    
    //Localization data
    //    The first parameter represents the frame order. 0-previous 1-current
    //    The last parameter represents x or y values
    bool isCrossExists[2][areaXCount][areaYCount];
    int localizationData[2][areaXCount][areaYCount][2];
    int coord[2][areaXCount][areaYCount][2];



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
    
   



    //Ros configuration_publisher
    ros::init(argc,argv, "RobotPositionPublisher");
    ros::NodeHandle n;

    ros::Publisher PositionPublisher = 
        n.advertise<std_msgs::String>("RobotPositionInfo", 500);
    ros::Rate loop_rate (loopRate);//max rate is 30 Hz. ImageProcess may slower than it.

    

    

    while (ros::ok())
    {
	    
	//get the stream and cut the img
	Mat tempCompleteImg; 
	cap >> tempCompleteImg;
	Rect rect(0,0,imgWidthCut,imgHeightCut);
	originImg = tempCompleteImg(rect);


        cvtColor(originImg,img,COLOR_RGB2GRAY);

	
	
	//calculate the canny threshold automatically
	thresholdCnt++;
        if(thresholdCnt > 150)//every 150 times loop finished, recalculate the threshold
	{
            thresholdCnt = 0;
            //meanStdDev(img,meanValueOfImg,stdDev);
            //double avg = meanValueOfImg.ptr<double>(0)[0];
	    Mat mThres_Gray;
            cannyMaxThreshold = min(200,(int)(threshold(img,mThres_Gray,0,255,THRESH_OTSU)));
            cannyMinThreshold = max(70,(int)(0.3*cannyMaxThreshold));
	}


	//Img pre-processing and line detection
	GaussianBlur(img,img,Size(gaussianBlurSize,gaussianBlurSize),0);
        imshow("GaussialBlur",img);
        Canny(img,img,cannyMinThreshold,cannyMaxThreshold,3);
        imshow("canny",img);
	std::vector<Vec2f>lines;
	HoughLines(img,lines,3,CV_PI/180,houghLineThreshold,0,0);
	
	
	for(int i = 0;i < lines.size(); i++)
        {
	    //std::cout<<"jjjj=\n";
	    double rho = lines[i][0],theta = lines[i][1];
	    Point pt1,pt2;
	    double a = cos(theta),b = sin(theta);
	    double x0 = rho/a,y0 = rho/b;
	    pt1.x = cvRound(x0 + 1000*(-b));
	    pt1.y = cvRound(y0 + 1000*(a));
	    pt2.x = cvRound(x0 + 1000*(-b));
	    pt2.y = cvRound(y0 + 1000*(a));
	    pt1.x = x0;
	    pt1.y = 0;	    
            pt2.x = 0;
	    pt2.y = y0;
	    line(originImg,pt1,pt2,Scalar(0,0,255),3,LINE_AA);
	}
        imshow("lines",originImg);

	waitKey(15);

    
    }
    return 0;
}
