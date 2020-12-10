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
#include <string> // memset


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
using namespace cv;
Rect imgResize(cutX1,cutY1,cutX2,cutY2);//TODO: 

std::string templateCam("/dev/video0"); 
std::string blueTemplatePath = "/home/coues/template/blueTemplate.png";
std::string greenTemplatePath = "/home/coues/template/greenTemplate.png";
std::string redTemplatePath = "/home/coues/template/redTemplate.png";
int blueLowH = 100, blueHighH = 124, 
    greenLowH = 35, greenHighH = 77, 
    redLowH1 = 0,   redHighH1 = 10, 
    redLowH2 = 156, redHighH2 = 180;
int LowS = 43, HighS = 255, LowV = 46, HighV = 255;
Scalar blueLow = Scalar(blueLowH,LowS,LowV);
Scalar greenLow = Scalar(greenLowH,LowS,LowV);
Scalar redLow1 = Scalar(redLowH1,LowS,LowV);
Scalar redLow2 = Scalar(redLowH2,LowS,LowV);
Scalar blueHigh = Scalar(blueHighH,HighS,HighV);
Scalar greenHigh = Scalar(greenHighH,HighS,HighV);
Scalar redHigh1 = Scalar(redHighH1,HighS,HighV);
Scalar redHigh2 = Scalar(redHighH2,HighS,HighV);
Mat originTemplate;
Mat temPlates[200];

//TODO: IMPORTANT : 100pixels = 30CM (height 60cm, Direction 30 degrees)
Mat convertBGR2HSV(Mat originImg)
{
    Mat hsvImg;
    cvtColor(originImg, hsvImg,CV_BGR2HSV);
    return hsvImg;
}

Mat getHueChanel(Mat hsvImg)
{
    Mat hueChannel;
    std::vector<Mat> channels;
    split(hsvImg,channels);
    return channels[0];//0,1,2 h,s,v
}
Point circleCentralPointDetection(Mat img)
{
    Mat croppedImg = img;// = img(imgResize);
    Mat grayImg,blurredImg;//,mThres_Gray;
    //cvtColor(croppedImg,grayImg,COLOR_RGB2GRAY);
    blurredImg = croppedImg;
    //medianBlur(grayImg,blurredImg,3);
    std::vector<Vec3f>circles;
    //unsigned int cannyMaxThreshold = min(200,(int)(threshold(blurredImg,mThres_Gray,0,255,THRESH_OTSU)));
    HoughCircles(blurredImg,circles,HOUGH_GRADIENT,1,100,150,40,60);//TODO: the last 2 paras is the min & max radius of circle
    //Mat centerCanvas(cutX2 - cutX1 , cutY2 - cutY1,CV_8UC1,Scalar(0));
    std::cout << circles.size() << std::endl;
    for(size_t i = 0; i < circles.size(); i++)
    {
        Vec3i c = circles[i];
        Point center = Point(c[0],c[1]);
	std::cout << "x  " << c[0] <<"  y " << c[1]<<std::endl;
    	circle(croppedImg,center,3,(255),-1);
    	//circle(centerCanvas,center,3,(255,255,255),-1);
        //centerCanvas.at<int>(c[1],c[0]) = 255;
	//uchar *rowData = centerCanvas.ptr<uchar>(c[0]);
        //rowData[c[1]] = 255;
        //int x = circles[i]
    }
    //circle(croppedImg,{100,100},40,(255,0,255),0);
    //imshow("centerCanvas",centerCanvas);
    //GaussianBlur(centerCanvas,centerCanvas,Size(9,9),0);
    //imshow("blur",centerCanvas);
    double minVal;
    double maxVal;
    Point minIdx;
    Point maxIdx;
    //minMaxLoc(centerCanvas,&minVal, &maxVal, &minIdx, &maxIdx);
    //circle(croppedImg,maxIdx,3,(0,0,255),0);
    imshow("circleDetect",croppedImg);
    waitKey(5);
}

// Point 
// {
//     medianBlur(src,dst,ksize );		
//     inRange(img,Scalar());
//     img.copyTo( img_display );
//     inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

// }
int setCamera(VideoCapture *cap, unsigned int _width, unsigned int _height, unsigned int _buffersize)
{
    (*cap).set(CAP_PROP_FRAME_WIDTH,_width);
    (*cap).set(CAP_PROP_FRAME_HEIGHT,_height);
    (*cap).set(CV_CAP_PROP_BUFFERSIZE, _buffersize);//only stores (n) frames in cache;
    std::string cmd1("v4l2-ctl -d ");
    std::string cmd11(" -c exposure_auto=1");
    std::string cmd12(" exposure_absolute=78");
    std::string cmd13(" brightness=60");
    std::string cmd14(" contrast=30");
    system((cmd1 + templateCam + cmd11).c_str());
    system((cmd1 + templateCam + cmd12).c_str());
    system((cmd1 + templateCam + cmd13).c_str());
    system((cmd1 + templateCam + cmd14).c_str());

    std::cout<<"default exposure: "<<(*cap).get(CAP_PROP_EXPOSURE)<<std::endl;
    std::cout<<"default contrast: "<<(*cap).get(CAP_PROP_CONTRAST)<<std::endl;
    std::cout<<"default brightne: "<<(*cap).get(CAP_PROP_BRIGHTNESS)<<std::endl;
}
Point templateMatching(Mat src)//return the value of x y for robot to move
{
    

    imshow("origin",src);
    Mat hsv,hue;
    //Mat croppedImg = img(imgResize);
    medianBlur(src,src,3);
    imshow("medianBlur",src);
    hsv = convertBGR2HSV(src);
    imshow("HSV",hsv);
    hue = getHueChanel(hsv);
    imshow("HueChannel",hue);

    Mat blueImg,redImg1,redImg2,redImg,greenImg;

    inRange(hsv, blueLow, blueHigh, blueImg);
    inRange(hsv, greenLow, greenHigh, greenImg);
    inRange(hsv, redLow1, redHigh1, redImg1);
    inRange(hsv, redLow2, redHigh2, redImg2);
    redImg = redImg1 | redImg2;
   
    
    circleCentralPointDetection(redImg);

    imshow("greenTreshold",greenImg);
    medianBlur(greenImg,greenImg,7);
    imshow("greenBlurTreshold",greenImg);
    
    imshow("redThreshold",redImg);
    medianBlur(redImg,redImg,7);
    imshow("redBlurThreshold",redImg);
    
    imshow("blueTreshold",blueImg);
    medianBlur(blueImg,blueImg,7);
    imshow("blueBlurTreshold",blueImg);
    
    waitKey(5);
}

int main(int argc, char **argv)
{
        //cv::cvtColor(img,grayImg,cv::COLOR_RGB2GRAY);
    //imshow("gray",grayImg);
    //undistort(image,undistortedImg,cameraMatrix,distortionCoefficients);
    
   



    //Ros configuration_publisher
    ros::init(argc,argv, "main");
    ros::NodeHandle n;

    ros::Publisher PositionPublisher = 
        n.advertise<std_msgs::String>("mainStatus", 500);
    ros::Rate loop_rate (loopRate);//max rate is 30 Hz. ImageProcess may slower than it.

    

    VideoCapture cap;
    // cap initialization and setting
    cap.open(templateCam);
    if(!cap.isOpened()){ 
        std::cout << "cam openning failed" << std::endl;
	    return -1;
    }
    setCamera(&cap, imgWidth, imgHeight, 2);
    while (ros::ok())
    {
        Mat img;
        cap >> img;
	    templateMatching(img);
	    //get the stream and cut the im
        //for(int i = 1;i <= 1;i++)
        //while(cap.grab()){}


    }

}
