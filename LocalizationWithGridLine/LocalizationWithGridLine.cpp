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
#include <math.h>
#include <fstream> //for debug. output to a file

#define imgWidth 640
#define imgHeight 360
#define imgWidthCut 640
#define imgHeightCut 210
#define warpedWidth 382
#define warpedHeight 315
#define loopRate 30 // whole loop rate
#define imgPartitionSize 60//(pixels) divide the img into areas. make sure each part has only one cross
#define gaussianBlurSize 11
#define pixelsCntPerCentimeter 3.33//TODO: 100p = 30cm
#define houghLineThreshold pixelsCntPerCentimeter*15.0
#define rotationThreshold (double)(25.0/180.0*CV_PI)
#define maxLensInImg (double)(sqrt(pow(warpedWidth,2)+pow(warpedHeight,2)) + 4.0)
#define currentFrame 1
#define previousFrame 0
#define areaXCount imgWidth/imgPartitionSize+3
#define areaYCount imgWidth/imgPartitionSize+3
using namespace cv;

//TODO IMPORTANT : 100pixels = 30CM (height 60cm, Direction 30 degrees)

Point2f perspectiveTransformOriginPoint[4] = {Point2f(155,0),Point2f(485,0),Point2f(0,320),Point2f(640,320)};
Point2f perspectiveTransformWarpedPointa[4] = {Point2f(0,0),Point2f(warpedWidth,0),Point2f(0,warpedHeight),Point2f(warpedWidth,warpedHeight)};

float gaussianPara[int(pixelsCntPerCentimeter*3+2)];
void calculateGaussianPara()
{
    double para = 1/sqrt(2*CV_PI);
    for(int i = 1;i <= pixelsCntPerCentimeter * 3;i++)
    {
        double gap = 4.0 / pixelsCntPerCentimeter / 3.0;
        gaussianPara[i] = para * exp(-pow((double)(-2+(double)i*gap),2)/2.0);
    }
}


void gaussianSum(Mat *gridLineFitting,int pixelPosition)
{

    int cnt = 1;
    int pixelTempPos;
    for(int i = 1;i <= pixelsCntPerCentimeter * 3 ;i++)
    {
        pixelTempPos = int(pixelPosition + i - pixelsCntPerCentimeter * 3.0 / 2.0 );
        if(pixelTempPos > maxLensInImg || pixelTempPos < 1) continue;
        ((float*)gridLineFitting->data)[pixelTempPos] += double(gaussianPara[i] * 20.0);
    }
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
    double xDirectionOfPreviousFrame = 0.0,yDirectionOfPreviousFrame = CV_PI/2;//theta in img.
    

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
	    //get the stream and cut the img
	    cap >> originImg;
	    
        //Transform perspective
        Mat lambda = getPerspectiveTransform(perspectiveTransformOriginPoint,perspectiveTransformWarpedPointa);
        warpPerspective(originImg,warpedImg,lambda, Size(320,300),INTER_LINEAR);

	    line(warpedImg,Point(30,30),Point(130,30),Scalar(0,0,255),2,LINE_AA);
        imshow("warpedImg",warpedImg);
        
        
        //Rect rect(0,0,imgWidthCut,imgHeightCut);
	    //originImg = tempCompleteImg(rect);


        cvtColor(warpedImg,warpedGrayImg,COLOR_RGB2GRAY);

	
	
	    //calculate the canny threshold automatically
	    thresholdCnt++;
        if(thresholdCnt > 150)//every 150 times loop finished, recalculate the threshold
	    {
            thresholdCnt = 0;
            //meanStdDev(img,meanValueOfImg,stdDev);
            //double avg = meanValueOfImg.ptr<double>(0)[0];
	        Mat mThres_Gray;
            cannyMaxThreshold = min(200,(int)(threshold(warpedGrayImg,mThres_Gray,0,255,THRESH_OTSU)));
            cannyMinThreshold = max(70,(int)(0.3*cannyMaxThreshold));
	    }


	    //Img pre-processing and line detection
	    GaussianBlur(warpedGrayImg,warpedGrayImg,Size(gaussianBlurSize,gaussianBlurSize),0);
        imshow("GaussialBlur",warpedGrayImg);
        Canny(warpedGrayImg,warpedGrayImg,cannyMinThreshold,cannyMaxThreshold,3);
        imshow("canny",warpedGrayImg);
	    std::vector<Vec2f>lines;
	    HoughLines(warpedGrayImg,lines,2,CV_PI/180,houghLineThreshold,0,0);
	
	

        //filter the parallel lines of x and y. then add them in to 1D mat using gaussing func
        Mat xGridLinesFitting = Mat::zeros( maxLensInImg + 5,1,CV_32F);//to calculate the fitest grid lines
        Mat yGridLinesFitting = Mat::zeros( maxLensInImg + 5,1,CV_32F);//to calculate the fitest grid lines
	    double xAverageDirection = 0,yAverageDirection = 0, xCountOfAverage = 0, yCountOfAverage = 0;
        
        
        std::cout<<lines.size()<<std::endl;
        
        //debug data
        ofstream outfile;
        outfile.open("degreeData",ios::out | ios::app);





        for(int i = 0;i < lines.size(); i++)
        {
	        float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            
            
            //line(warpedImg,Point(150+a*100,150+b*100),Point(100,100),Scalar(255,0,0),2,LINE_AA);


	        //filter of the parallel lines of x and y
	        //TODO: direction changed from pi to 0
            if(abs(theta - xDirectionOfPreviousFrame) < rotationThreshold)
	        {
                gaussianSum(&xGridLinesFitting,rho);
                xCountOfAverage++ ;
                xAverageDirection += rho;
            }
            else if(abs(theta - yDirectionOfPreviousFrame) < rotationThreshold)
            {
                gaussianSum(&yGridLinesFitting,rho);
                yCountOfAverage++;
                yAverageDirection += rho;
            }

	        line(warpedImg,pt1,pt2,Scalar(0,0,255),2,LINE_AA);
            
            outfile << "rho: "<< rho << " Theta: " << theta << std::endl
	    
        }

        outfile.close();




        imshow("hough lines",warpedImg);
        xAverageDirection = xAverageDirection / double(xCountOfAverage);
        yAverageDirection = yAverageDirection / double(yCountOfAverage);
        xDirectionOfPreviousFrame = xAverageDirection;
        yDirectionOfPreviousFrame = yAverageDirection;

        //trying to find the fitest grid by using traversal
        int xRho,yRho,xMax = 0, yMax = 0; //rho theta stores the value of fitest gridline
        for(int i = 1; i <= pixelsCntPerCentimeter * 29 ; i += 2)
        {
            int sumValue = 0;
            for(int k = i; k <= maxLensInImg ; k+= 100)//100 pixels = 30cm = 1 square
            {
                sumValue += ((float*)xGridLinesFitting.data)[k];
            } 
            if(sumValue > xMax) 
            {
                xMax = sumValue;
                xRho = i;
            }
        }
        for(int i = 1; i <= pixelsCntPerCentimeter * 29 ; i += 2)
        {
            int sumValue = 0;
            for(int k = i; k <= maxLensInImg ; k+= 100)
            {
                sumValue += ((float*)yGridLinesFitting.data)[k];
            } 
            if(sumValue > yMax) 
            {
                yMax = sumValue;
                yRho = i;
            }
        }

        //draw grid lines

       /* 
            int distance[4] = {0, 100, 200, 300};
            if(xAverageDirection > CV_PI / 2.0)
            {
                double tempTheta = CV_PI - xAverageDirection;
                for(int i = 0; i < 4; i++)
                {
                    int rho = xRho + distance[i];
                    line(warpedImg,
                        Point(warpedWidth - rho / cos(tempTheta),0),
                        Point(warpedWidth, rho / sin(tempTheta)),
                        Scalar(0,0,255),2,LINE_AA);
                }
            }
            else
            {
                double tempTheta = xAverageDirection;
                for(int i = 0; i < 4; i++)
                {
                    int rho = xRho + distance[i];
                    line(warpedImg,
                        Point(rho / cos(tempTheta),0),
                        Point(0, rho / sin(tempTheta)),
                        Scalar(0,0,255),2,LINE_AA);
                }
            }
            if(yAverageDirection > CV_PI / 2.0)
            {
                double tempTheta = CV_PI - yAverageDirection;
                for(int i = 0; i < 4; i++)
                {
                    int rho = yRho + distance[i];
                    line(warpedImg,
                        Point(warpedWidth - rho / cos(tempTheta),0),
                        Point(warpedWidth, rho / sin(tempTheta)),
                        Scalar(0,0,255),2,LINE_AA);
                }
            }
             else
            {
                double tempTheta = yAverageDirection;
                for(int i = 0; i < 4; i++)
                {
                    int rho = yRho + distance[i];
                    line(warpedImg,
                        Point(rho / cos(tempTheta),0),
                        Point(0, rho / sin(tempTheta)),
                        Scalar(0,0,255),2,LINE_AA);
                }
            }
        
        */





        //imshow("gridLines",warpedImg);

	    waitKey(10);

    
    }
    return 0;
}
