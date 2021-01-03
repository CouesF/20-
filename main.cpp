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
#include "opencv2/objdetect.hpp"//qrcode detector
#include "opencv2/imgcodecs.hpp"
#include "zbar.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <sstream>
#include <stdbool.h>
#include <vector>
#include <math.h>
#include <fstream> //for debug. output to a file
#include <string> // memset

// C library headers
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <string.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close(), sleep()
#include <sys/ioctl.h>
#include <getopt.h>

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



//for the order of material
#define red 0
#define green 1
#define blue 2
short colorOrder[3];

int cutX1 = 10,
    cutY1 = 10,     
    cutX2 = 300, 
    cutY2 = 300;

int MKSGENfd,MKSDLCfd;
using namespace cv;
Rect imgResize(cutX1,cutY1,cutX2,cutY2);//DONE: 
Rect templateArea(15,15,150,150);
std::string templateCam("/dev/cam4TemplateMatching"); 
std::string qrCodeCam("/dev/cam4QRCode"); 
std::string MKSGEN("/dev/MKSGEN");
std::string MKSDLC("/dev/MKSDLC");
std::string blueTemplatePath = "/home/coues/template/blueTemplate.png";
std::string greenTemplatePath = "/home/coues/template/greenTemplate.png";
std::string redTemplatePath = "/home/coues/template/redTemplate.png";

//Rect templateArea(templateX,templateY,templateWidth,templateHeight);
Mat blueTemplate, greenTemplate, redTemplate;
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

bool positionDataUsed;
double maxSpeed = 100.0;
#define ratioFromPixel2Steps 20 //TODO:

Mat theMap(500,500,CV_8UC3,Scalar(255,255,255,0.5));
float _robotCurrentGlobalPosition[3];
Point3f robotCurrentGlobalPosition;
Point templateMatching(Mat src,int color);//return the value of x y of the center position for template
void sleepTime(float seconds);
void setMovement(int x, int y, float rotation);
void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    int i = 0;
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
		_robotCurrentGlobalPosition[i] = *it;
		i++;
	}
	positionDataUsed = 0;
	robotCurrentGlobalPosition.x = _robotCurrentGlobalPosition[0];
        robotCurrentGlobalPosition.y = _robotCurrentGlobalPosition[1];
        robotCurrentGlobalPosition.z = _robotCurrentGlobalPosition[2];
	
    	circle(theMap,
	       Point(robotCurrentGlobalPosition.x * 50 + 50,
		     robotCurrentGlobalPosition.y * 50 + 50),
	       4,
	       (0,0,0),
	       -1);
	return;
}

//DONE: IMPORTANT : 100pixels = 30CM (height 60cm, Direction 30 degrees)
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
Point circleCentralPointDetectionBGR(Mat img)
{
    Mat croppedImg = img;// = img(imgResize);
    Mat grayImg,blurredImg,mThres_Gray;
    cvtColor(croppedImg,grayImg,COLOR_RGB2GRAY);
    blurredImg = grayImg;//croppedImg;
    //medianBlur(grayImg,blurredImg,3);
    std::vector<Vec3f>circles;
    unsigned int cannyMaxThreshold = 100;//min(200,(int)(threshold(blurredImg,mThres_Gray,0,255,THRESH_OTSU)));
    HoughCircles(blurredImg,circles,HOUGH_GRADIENT,2,
		    30,
		    cannyMaxThreshold,90,90,150);//TODO: the last 2 paras is the min & max radius of circle
    //Mat centerCanvas(cutX2 - cutX1 , cutY2 - cutY1,CV_8UC1,Scalar(0));
    std::cout << circles.size() << std::endl;
    for(size_t i = 0; i < circles.size(); i++)
    {
        Vec3i c = circles[i];
        Point center = Point(c[0],c[1]);
	std::cout << "x  " << c[0] <<"  y " << c[1]<<std::endl;
    	circle(croppedImg,center,c[2],(255,0,255),1);
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
float dist(Point2f point1,Point2f point2)
{
    return sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
}
void setSpeed(float dir, int speed, int rotationSpeed)
{
    double speedA, speedB, speedC, speedD;
    speedA = - speed * sin(dir);
    speedB = speed * cos(dir);
    speedC = speed * sin(dir);
    speedD = - speed * cos(dir);
    speedA += rotationSpeed;
    speedB += rotationSpeed;
    speedC += rotationSpeed;
    speedD += rotationSpeed;
    std::string msg = "S A" + std::to_string((int)speedA) + " B" + std::to_string((int)speedB) + " C" + std::to_string((int)speedC) +" D" + std::to_string((int)speedD) + " @";
    //unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\n' };
    std::cout<<msg.c_str()<<std::endl<<std::endl;
    write(MKSGENfd, msg.c_str(), sizeof(msg.c_str()));
}
int moveToGlobalPosition(Point3f target)
{
    int returnValue = 0;
    double deltaX = target.x - robotCurrentGlobalPosition.x;
    double deltaY = target.y - robotCurrentGlobalPosition.y;
    double moveDir = atan2(deltaY,deltaX);
    moveDir += CV_PI / 4;
    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    double movingSpeed = 1.000;
    double deltaRotation = target.z - robotCurrentGlobalPosition.z;
    int rotationSpeed = int(deltaRotation * 15.0);
    
    printf("cur:: x:%.3lf,y:%.3lf,dir:%.3lf\n",robotCurrentGlobalPosition.x,robotCurrentGlobalPosition.y,robotCurrentGlobalPosition.z);

    printf("delta: x:%.3lf, y:%.3lf, dir:%.3lf\n",deltaX,deltaY,deltaRotation);
    printf("distance:%lf,delta:%lf\n",distance,rotationSpeed);
    
    if(distance > 1.2)
    {
        movingSpeed = maxSpeed * 0.8;
        returnValue += 1;
    }
    else if(distance > 0.17)
    {
        movingSpeed = distance/1.2 * 0.8 * maxSpeed;
        returnValue += 2;
    }
    else if(distance > 0.08)
    {
        movingSpeed = maxSpeed * 0.15;
        returnValue += 3;
    }
    else
    {
        returnValue +=100;
        movingSpeed = 0;
    }
    
    
    
    if(deltaRotation > 1.2)
    {
        rotationSpeed = maxSpeed * 0.3;
	returnValue += 10;
    }
    else if(deltaRotation > 0.35)
    {
        rotationSpeed = rotationSpeed/1.2 * 0.3 * maxSpeed;
        returnValue += 20;
    }
    else if(deltaRotation > 0.07)
    {
        rotationSpeed = maxSpeed * 0.15;
        returnValue += 30;
    }
    else
    {
        returnValue += 100;
        rotationSpeed = 0;
    }
    printf("moveDir:%.3lf speed:%d rotateV:%d\n",moveDir,(int)movingSpeed,(int)rotationSpeed);
    setSpeed(moveDir,(int)movingSpeed,(int)rotationSpeed);
    return returnValue;
    //https://blog.csdn.net/qq_38410730/article/details/103272396
}

// Point 
// {
//     medianBlur(src,dst,ksize );		
//     inRange(img,Scalar());
//     img.copyTo( img_display );
//     inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

// }
int setCamera(VideoCapture *cap, unsigned int _width, unsigned int _height, unsigned int _buffersize,std::string Cam);
void accurateLocalization(Mat src,int color)//used after run to the right position
{
    Point templatePoint = templateMatching(src,color);
    int deltaX,deltaY;
    deltaX = templatePoint.x - 320;
    deltaY = templatePoint.y - 240;
    float stepX = deltaX * ratioFromPixel2Steps;
    float stepY = deltaY * ratioFromPixel2Steps;
    double distance = sqrt(pow(deltaX,2)+pow(deltaY,2));
    setMovement((int)stepX,(int)stepY,0);
    sleepTime(distance * 0.1);

    printf("moveStoped!!!!!!!!!!!!!!!!!!!!!!!\n\n");
    sleepTime(3);
    
    
}
bool getQRcodeInfo(Mat img);//print data and process the data to the array
Point3f targetPoint(float x, float y, float dir);
void getTemplate();

int main(int argc, char **argv)
{
        //cv::cvtColor(img,grayImg,cv::COLOR_RGB2GRAY);
    //imshow("gray",grayImg);
    //undistort(image,undistortedImg,cameraMatrix,distortionCoefficients);
    //Ros configuration_publisher
    ros::init(argc,argv, "main");
    ros::NodeHandle n;

    ros::Subscriber  PositionSubscriber = 
        n.subscribe<std_msgs::Float32MultiArray>("RobotPositionInfo", 24,arrayCallback);
    ros::Rate loop_rate (loopRate);//max rate is 30 Hz. ImageProcess may slower than it.

    
    
        // cap initialization and setting
        VideoCapture templateCap;
	printf("prepare to open templateCam\n");
        templateCap.open(templateCam.c_str());
        if(!templateCap.isOpened()){
            std::cout << "template cam openning failed" << std::endl;
	        return -1;
        }
        setCamera(&templateCap, imgWidth, imgHeight, 2, templateCam);
       

	VideoCapture qrCodeCap;
        qrCodeCap.open(qrCodeCam);
        if(!qrCodeCap.isOpened()){
            std::cout << "qrcode cam openning failed" << std::endl;
	        return -1;
        }
        setCamera(&qrCodeCap, imgWidth, imgHeight, 2,qrCodeCam);
    
      //------------------------------------------------------// 
       	//Serial Open & settings
        MKSGENfd = open(MKSGEN.c_str(),O_RDWR | O_NOCTTY | O_SYNC);
        struct termios tty;
        if(MKSGENfd == -1)
	{
            printf("MKSGEN open failed\n");
            return -1;
	}
        if(tcgetattr(MKSGENfd, &tty) < 0) 
	{
            return -1;//printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }
        cfsetispeed(&tty,B115200);
	cfsetospeed(&tty,B115200);
        tty.c_iflag &= ~IGNBRK;                 // no break processing
        tty.c_lflag = 0;                        // no signal characters
        tty.c_oflag = 0;                        // no remapping
        tty.c_cc[VMIN] = 0;                     // no blocking
        tty.c_cc[VTIME] = 0;                    // no read timeout
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no xon / xoff control
        tty.c_cflag |= CLOCAL | CREAD;          // ignore modem control
        tty.c_cflag &= ~CRTSCTS;                // no RTS/CTS flow control
// data bits
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // parity bits
    tty.c_cflag &= ~(PARENB | PARODD);

    // stop bits
    tty.c_cflag &= ~CSTOPB;
    if(tcsetattr(MKSGENfd,TCSAFLUSH, &tty) < 0) {
        return -1;//printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    write(MKSGENfd,"S A20 B-20 C-20 D20 @",sizeof("S A20 B-20 C-20 D20 @"));
    printf("test\n\n");
    sleepTime(5);
    printf("test\n\n");
      //------------------------------------------------------// 

        
/*	
	MKSDLCfd = open(MKSDLC.c_str(),O_RDWR|O_NOCTTY);
        struct termios MKSDLCtermios;
        if(MKSDLCfd == -1)
	{
            printf("MKSDLC open failed\n");
            return -1;
	}
        if(tcgetattr(MKSDLCfd, &MKSDLCtermios) != 0) {
            return -1;//printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }
        setSerialMKSGEN(MKSDLCfd,&MKSDLCtermios);
*/

        printf("templateCam & qrCodeCam & MKSDLC & MKSGEN \n opened successfully\n");
    
    setSpeed(0,0,0);
    printf("iiiiiooooo\n\n\n");
    sleepTime(5);
    
    setSpeed(0,40,0);
    printf("oooohjgffgbo\n\n\n");
    sleepTime(5);
    
    setSpeed(0,0,0);
    printf("ooooo\n\n\n");
    sleepTime(5);
    

    while (ros::ok())
    {
        ros::spinOnce();
        if(!positionDataUsed)
        {
            positionDataUsed = !positionDataUsed;
        }
        else
        {
            printf("No Global Data\n");
        }
        
	//{
        double x,y,dir;
	scanf("%lf %lf %lf",&x,&y,&dir);
        dir = dir / 180 * CV_PI;
        
	double flag;
	do
	{
            ros::spinOnce();
            Point3f _target = targetPoint(x,y,dir);
	    printf("target::  x:%.3lf y:%.3lf dir:%.3lf\n",x,y,dir);

            flag = moveToGlobalPosition(_target);
	    printf("moving Flag:%d\n",flag);

	}
	while(flag<200);
        printf("succussesfully moved!\n");
		
    //    Mat img;
    //      cap >> img;
	//    templateMatching(img);
	    //get the stream and cut the im
        //for(int i = 1;i <= 1;i++)
        //while(cap.grab()){}

    //	circleCentralPointDetectionBGR(img);
        //imshow("mappppp",theMap);	
   	//waitKey(100);
    }

}


int setCamera(VideoCapture *cap, unsigned int _width, unsigned int _height, unsigned int _buffersize,std::string Cam)
{
    (*cap).set(CAP_PROP_FRAME_WIDTH,_width);
    (*cap).set(CAP_PROP_FRAME_HEIGHT,_height);
   // (*cap).set(CV_CAP_PROP_BUFFERSIZE, _buffersize);//only stores (n) frames in cache;
    std::string cmd1("v4l2-ctl -d ");
    std::string cmd11(" -c exposure_auto=1");
    std::string cmd12(" exposure_absolute=78");
    std::string cmd13(" brightness=60");
    std::string cmd14(" contrast=30");
    system((cmd1 + Cam + cmd11).c_str());
    system((cmd1 + Cam + cmd12).c_str());
    system((cmd1 + Cam + cmd13).c_str());
    system((cmd1 + Cam + cmd14).c_str());

    std::cout<<"default exposure: "<<(*cap).get(CAP_PROP_EXPOSURE)<<std::endl;
   std::cout<<"default contrast: "<<(*cap).get(CAP_PROP_CONTRAST)<<std::endl;
    std::cout<<"default brightne: "<<(*cap).get(CAP_PROP_BRIGHTNESS)<<std::endl;
}
void setMovement(int x, int y, float rotation)
{
    std::string msg = "P X" + std::to_string((int)x) + " Y" + std::to_string((int)y) + " R" + std::to_string((int)rotation) +" @";
    //unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\n' };
    int _flag = write(MKSGENfd, msg.c_str(), sizeof(msg.c_str()));
    if(_flag == -1)printf("MKSGEN write failed\n");
    std::cout << msg << std::endl;
}

void sleepTime(float seconds)
{
    unsigned int microsecond = 1000000;
    seconds *= microsecond;
    usleep(seconds);//sleeps for 3 second
}
Point templateMatching(Mat src,int color)//return the value of x y of the center position for template
{
    Mat _template,matchingResult;
    switch (color)
    {
    case red:
        _template = redTemplate;
        break;
    case green:
        _template = greenTemplate;
        break;
    case blue:
        _template = blueTemplate;
        break;
    default:
        break;
    }
    matchTemplate(src,_template,matchingResult,TM_CCOEFF);
    double minVal;
    double maxVal;
    Point minIdx;
    Point maxIdx;
    minMaxLoc(matchingResult,&minVal, &maxVal, &minIdx, &maxIdx);
    circle(src,maxIdx,5,(0,0,255),-1);
    imshow("templateMatching",src);
    waitKey(0);
    return maxIdx;
    // Mat hsv,hue;
    // //Mat croppedImg = img(imgResize);
    // medianBlur(src,src,3);
    // imshow("medianBlur",src);
    // hsv = convertBGR2HSV(src);
    // imshow("HSV",hsv);
    // hue = getHueChanel(hsv);
    // imshow("HueChannel",hue);

    // Mat blueImg,redImg1,redImg2,redImg,greenImg;

    // inRange(hsv, blueLow, blueHigh, blueImg);
    // inRange(hsv, greenLow, greenHigh, greenImg);
    // inRange(hsv, redLow1, redHigh1, redImg1);
    // inRange(hsv, redLow2, redHigh2, redImg2);
    // redImg = redImg1 | redImg2;
   
    
    // circleCentralPointDetection(redImg);

    // imshow("greenTreshold",greenImg);
    // medianBlur(greenImg,greenImg,7);
    // imshow("greenBlurTreshold",greenImg);
    
    // imshow("redThreshold",redImg);
    // medianBlur(redImg,redImg,7);
    // imshow("redBlurThreshold",redImg);
    
    // imshow("blueTreshold",blueImg);
    // medianBlur(blueImg,blueImg,7);
    // imshow("blueBlurTreshold",blueImg);
    
    // waitKey(5);
}
/*
bool getQRcodeInfo(Mat img)
{
    cv::QRCodeDetector qrDecoder = QRCodeDetector::QRCodeDetector();
    Mat bbox, rectifiedImage;
    std::string data = qrDecoder.detectAndDecode(img, bbox, rectifiedImage);
    if(data.length()>0)//TODO:
    {
        std::cout << "Decoded Data : " << data <<std::endl;
        rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
        imshow("Rectified QRCode", rectifiedImage);
        waitKey(0);
        return 1;
    }
    else
    {
        cout << "QR Code not detected" << endl;
        return 0;
    }
}
*/
Point3f targetPoint(float x, float y, float dir)
{
    Point3f target;
    target.x = x;
    target.y = y;
    target.z = dir;
    return target;
}
void getTemplate()
{
    //template initialize. Read & crop
    blueTemplate = (imread(blueTemplatePath.c_str()))(templateArea),
    greenTemplate = (imread(greenTemplatePath.c_str()))(templateArea), 
    redTemplate = (imread(redTemplatePath.c_str()))(templateArea);

}
