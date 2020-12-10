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
int cutX1 = 10,  cutY1 = 10,     
    cutX2 = 300, cutY2 = 300;
Rect templateSize(cutX1,cutY1,cutX2,cutY2);//TODO: 

std::string blueTemplatePath = "/home/coues/template/blueTemplate.png";
std::string greenTemplatePath = "/home/coues/template/greenTemplate.png";
std::string redTemplatePath = "/home/coues/template/redTemplate.png";
using namespace cv;
std::string templateCam("/dev/templateCam"); 
Mat convertBGR2HSV(originImg)
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
void camSettings(VideoCapture *capture)
{
    *capture.set(CAP_PROP_FRAME_WIDTH,imgWidth);
    *capture.set(CAP_PROP_FRAME_HEIGHT,imgHeight);
    *capture.set(CV_CAP_PROP_BUFFERSIZE, 2);//only stores (n) frames in cache;
        std::string cmd1("v4l2-ctl -d ");
        std::string cmd11(" -c exposure_auto=1");
        std::string cmd12(" exposure_absolute=78");
        std::string cmd13(" brightness=60");
        std::string cmd14(" contrast=30");
        system(cmd1 + templateCam + cmd11);
        system(cmd1 + templateCam + cmd12);
        system(cmd1 + templateCam + cmd13);
        system(cmd1 + templateCam + cmd14);

        std::cout<<"default exposure: "<<*capture.get(CAP_PROP_EXPOSURE)<<std::endl;
        std::cout<<"default contrast: "<<*capture.get(CAP_PROP_CONTRAST)<<std::endl;
        std::cout<<"default brightne: "<<*capture.get(CAP_PROP_BRIGHTNESS)<<std::endl;
}
void getTemplate(int color)
{
    VideoCapture cap;
    // cap initialization and setting
    cap.open(templateCam);
    if(!cap.isOpened()){ 
        std::cout << "cam openning failed" << std::endl;
	    return -1;
    }
    camSettings(&cap);

    Mat src;
    cap >> src;
    Mat croppedImg = img(templateSize);
    switch(color)
    {
        case 0:
            imwrite(blueTemplatePath, croppedImg);
            break;
        case 1:
            imwrite(greenTemplatePath, croppedImg);
            break;
        case 2:
            imwrite(redTemplatePath, croppedImg);
            break;
    }
    cap.release();
}

int main(int argc, char **argv)
{
    int color;
    switch(argv[0])
    {
        case "0":
            color = 0;
            break;
        case "1":
            color = 1;
            break;
        case "2":
            color = 2;
            break;
        default:
            std::cout<<"Color input ERROR\n";
    }
    getTemplate(color);
}
