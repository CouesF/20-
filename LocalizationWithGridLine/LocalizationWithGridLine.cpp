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
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

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
#define warpedWidth 382 * photoReductionScale
#define warpedHeight 315 * photoReductionScale
#define loopRate 30 // whole loop rate
#define imgPartitionSize 60 * photoReductionScale//(pixels) divide the img into areas. make sure each part has only one cross
#define gaussianBlurSize 11
#define pixelsCntPerCentimeter 3.33  * photoReductionScale//DONE: 100p = 30cm
#define houghLineThreshold pixelsCntPerCentimeter*25.0
#define rotationThreshold (double)(10.0/180.0*CV_PI)
#define maxLensInImg (double)(sqrt(pow(warpedWidth ,2)+pow(warpedHeight,2)) + 4.0)
#define currentFrame 1
#define previousFrame 0
#define areaXCount imgWidth/imgPartitionSize+3//TODO:?????????
#define areaYCount imgWidth/imgPartitionSize+3
#define gaussianSumMax 800//used for debug draw on canvas
#define photoReductionScale 0.5 
#define camBuffersize 10

using namespace cv;

std::string localizationCam("/dev/video0"); 
//TODO IMPORTANT : 100pixels = 30CM (height 60cm, Direction 30 degrees)

Point2f perspectiveTransformOriginPoint[4] = {Point2f(155,0),Point2f(485,0),Point2f(0,320),Point2f(640,320)};
Point2f perspectiveTransformWarpedPointa[4] = {Point2f(0,0),Point2f(warpedWidth,0),Point2f(0,warpedHeight),Point2f(warpedWidth,warpedHeight)};
int xGridLinesFitting[int(maxLensInImg *2 + 20)];
int yGridLinesFitting[int(maxLensInImg *2 + 20)];
float gaussianPara[32];//center is set to 15

#define tempTheta atan(2.0/3.25)
#define lenth  sqrt(3.25*3.25 + 2.0*2.0)
//debug
Mat theMap(500,500,CV_8UC3,Scalar(255,255,255,0.5));



void calculateGaussianPara()
{
    double para = 1.0/sqrt(35.0*2.0*CV_PI);
    //sigma^2 = 5
    for(int i = -9;i <= 9;i++)
    {
	    int k = 15+i;
        gaussianPara[k] = para * exp(-pow((double)(i/3.0),2)/35.0/2)    *45.0;
    }
}


void gaussianSum(int pixelPosition,int k)// TODO:rewrite using array
{
    pixelPosition += maxLensInImg;
    int pixelTempPos;
    for(int i = -9;i <= 9 ;i++)
    {
        pixelTempPos = int(pixelPosition + i);
        //if(pixelTempPos > maxLensInImg || pixelTempPos < 1) continue;
        if(k == 0)
        {
            xGridLinesFitting[pixelTempPos] += double(gaussianPara[i+15]);
        }
        if(k == 1)
        {
            yGridLinesFitting[pixelTempPos] += double(gaussianPara[i+15]);
        }
    }
}

unsigned int robotDirectionClassification(double dir)
{
    if(dir == CV_PI / 2.0) return 5;
    if(dir == -CV_PI / 2.0) return 6;
    if(dir > 0)
    {
        if(dir < CV_PI && dir > CV_PI / 2.0)
        {
            return 4;
        }
        else if(dir < CV_PI /2.0)
        {
            return 3;
        }
    }
    else
    {
        if(dir < -CV_PI / 2.0 && dir > -CV_PI)
        {
            return 1;
        }
        else if(dir > -CV_PI / 2.0)
        {
            return 2;
        }
        
    }
}
void captureInitializaition(VideoCapture *capture)
{
    (*capture).open(localizationCam);
    (*capture).set(CAP_PROP_FRAME_WIDTH,imgWidth);
    (*capture).set(CAP_PROP_FRAME_HEIGHT,imgHeight);
    (*capture).set(CV_CAP_PROP_BUFFERSIZE, camBuffersize);
    if(!(*capture).isOpened()){ 
        std::cout << "cam openning failed" << std::endl;
	return ;
    }
    std::string cmd1("v4l2-ctl -d ");
    std::string cmd11(" -c exposure_auto=1");
    std::string cmd12(" exposure_absolute=78");
    std::string cmd13(" brightness=60");
    std::string cmd14(" contrast=30");
    system((cmd1 + localizationCam + cmd11).c_str());
    system((cmd1 + localizationCam + cmd12).c_str());
    system((cmd1 + localizationCam + cmd13).c_str());
    system((cmd1 + localizationCam + cmd14).c_str());
     std::cout<<"default exposure: "<<(*capture).get(CAP_PROP_EXPOSURE)<<std::endl;
    std::cout<<"default contrast: "<<(*capture).get(CAP_PROP_CONTRAST)<<std::endl;
    std::cout<<"default brightne: "<<(*capture).get(CAP_PROP_BRIGHTNESS)<<std::endl;
    
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
    captureInitializaition(&cap);
    
    //Localization data
        //    The first parameter represents the frame order. 0-previous 1-current
        //    The last parameter represents x or y values
        //bool isCrossExists[2][areaXCount][areaYCount];
        //int localizationData[2][areaXCount][areaYCount][2];
        //int coord[2][areaXCount][areaYCount][2];
    double xDirectionOfPreviousFrame = 0,yDirectionOfPreviousFrame = CV_PI/2;//theta in img.
    double robotGlobalDirection =  -CV_PI / 2; //robot direction in global map. Initial direction is -pi/2
    double robotGlobalX = 0,robotGlobalY = 0;
    int xGlobal = 0, yGlobal = 3;

    int xRho,yRho,xPreviousRho = 0,yPreviousRho = 0;
    std_msgs::Float32MultiArray positionDataToSend;

    //undistort(image,undistortedImg,cameraMatrix,distortionCoefficients);
    
    calculateGaussianPara();
   



    //Ros configuration_publisher
    ros::init(argc,argv, "RobotPositionPublisher");
    ros::NodeHandle n;

    ros::Publisher PositionPublisher = 
        n.advertise<std_msgs::Float32MultiArray>("RobotPositionInfo", 100);
    ros::Rate loop_rate (loopRate);//max rate is 30 Hz. ImageProcess may slower than it.

    

    

    while (ros::ok())
    {

        cap >> originImg;
	    
        //Transform perspective
        Mat lambda = getPerspectiveTransform(perspectiveTransformOriginPoint,perspectiveTransformWarpedPointa);
        warpPerspective(originImg,warpedImg,lambda, Size(320 * photoReductionScale,300 * photoReductionScale),INTER_LINEAR);

	    //line(warpedImg,Point(30,30),Point(80,30),Scalar(0,0,255),2,LINE_AA);
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


        //test Median blur
        Mat testForMedianBlur = warpedGrayImg;
        medianBlur(testForMedianBlur, testForMedianBlur, 5);
        Canny(testForMedianBlur,testForMedianBlur,cannyMinThreshold,cannyMaxThreshold,3);
	    imshow("medianCanny",testForMedianBlur);

        //Img pre-processing and 
	//    GaussianBlur(warpedGrayImg,warpedGrayImg,Size(gaussianBlurSize,gaussianBlurSize),0);
        //Canny(warpedGrayImg,warpedGrayImg,cannyMinThreshold,cannyMaxThreshold,3);
        //imshow("gaussianCanny",warpedGrayImg);
	 
	warpedGrayImg = testForMedianBlur;


        //line detection
        std::vector<Vec2f>lines;
	    HoughLines(warpedGrayImg,lines,1,CV_PI/180,houghLineThreshold,0,0);
	
	

        //filter the parallel lines of x and y. then add them in to 1D mat using gaussing func
	    memset(xGridLinesFitting,0,sizeof(xGridLinesFitting));
        memset(yGridLinesFitting,0,sizeof(yGridLinesFitting));
        double xAverageDirection = 0,yAverageDirection = 0, xCountOfAverage = 0, yCountOfAverage = 0;
        
        
        

        //debug data
        std::ofstream outfile;
        outfile.open("/home/pi/data",std::ios::out | std::ios::app);




        
        double deltaThetaXSum = 0;
        double deltaThetaYSum = 0;
        int rhoOfX[500];
        int rhoOfY[500];
        int ambiguousX[3] = {0};
        int ambiguousY[3] ={0};
        int xDirCnt = 0, yDirCnt = 0;
        double averageDeltaX, averageDeltaY;
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
            int originTheta = theta; 
            //line(warpedImg,Point(150+a*100,150+b*100),Point(100,100),Scalar(255,0,0),2,LINE_AA);


            //filter of the parallel lines of x and y
            //DONE: direction changed from pi to 0
            // calculate current angle of x and y in img [0,pi)
            {
                theta = theta + CV_PI/2.0;
                if(theta > CV_PI)theta-=CV_PI;
                double deltaThetaX = theta - xDirectionOfPreviousFrame;
                double deltaThetaY = theta - yDirectionOfPreviousFrame;

                if(deltaThetaX > CV_PI / 2.0) deltaThetaX -= CV_PI;
                else if (deltaThetaX < - CV_PI / 2.0) deltaThetaX += CV_PI;
                if(deltaThetaY > CV_PI / 2.0) deltaThetaY -= CV_PI;
                else if (deltaThetaY < - CV_PI / 2.0) deltaThetaY += CV_PI;
                
                //check whether it is parellel to x or y
                if(abs(deltaThetaX) < rotationThreshold)//it is x line
                {

                    deltaThetaXSum += deltaThetaX;
                    rhoOfX[xDirCnt] = rho;
                    xDirCnt++;
                    if(originTheta < rotationThreshold/1.5 || originTheta > CV_PI - rotationThreshold / 1.5) ambiguousX[2]=1;
                    if(rho < 0)ambiguousX[0] = 1;// std::cout<<"rhoooooooo   \n"<<rho<<"   --"<<theta<<"\n";
                    else ambiguousX[1] = 1;
                }
                else if(abs(deltaThetaY) < rotationThreshold)//it is x line
                {
                    deltaThetaYSum += deltaThetaY;
                    rhoOfY[yDirCnt] = rho;
                    yDirCnt++;
                    if(originTheta < rotationThreshold/1.5 || originTheta > CV_PI - rotationThreshold / 1.5) ambiguousY[2]=1;
                    if(rho < 0)ambiguousY[0] = 1;// std::cout<<"rhoooooooo   \n"<<rho<<"   --"<<theta<<"\n";
                    else ambiguousY[1] = 1;
                }
            }
	        line(warpedImg,pt1,pt2,Scalar(0,0,255),2,LINE_AA);
        }

        //(with former part)calculate current angle of x and y in img [0,pi)
        averageDeltaX = deltaThetaXSum / double(xDirCnt);
        averageDeltaY = deltaThetaYSum / double(yDirCnt);
        xAverageDirection = xDirectionOfPreviousFrame + averageDeltaX;
        yAverageDirection = yDirectionOfPreviousFrame + averageDeltaY;
        if(xAverageDirection > CV_PI) xAverageDirection -= CV_PI;
        else if(xAverageDirection < 0) xAverageDirection += CV_PI;
        if(yAverageDirection > CV_PI) yAverageDirection -= CV_PI;
        else if(yAverageDirection < 0) yAverageDirection += CV_PI;
        //get robot direction---------> robotGlobalDirection
        if(abs(xAverageDirection - xDirectionOfPreviousFrame) < rotationThreshold) 
            robotGlobalDirection += ((double)xAverageDirection- (double)xDirectionOfPreviousFrame);
        else 
            robotGlobalDirection += ((double)yAverageDirection - (double)yDirectionOfPreviousFrame);
        if(robotGlobalDirection > CV_PI)robotGlobalDirection -= 2*CV_PI;
        else if( robotGlobalDirection <-CV_PI)robotGlobalDirection += 2*CV_PI;
        
        //process the rho using robot Dir
        double Dir = robotGlobalDirection;
        int dirStatus = robotDirectionClassification(Dir);
        if(ambiguousX[0]==1&&ambiguousX[1]==1&&ambiguousX[2]==1)
            dirStatus = 5;
        if(ambiguousY[0]==1&&ambiguousY[1]==1&&ambiguousY[2]==1)
            dirStatus = 6;
     //   printf("status:%d\n",dirStatus); 
	for(int i = 0; i < xDirCnt; i++)
        {
            int rho = rhoOfX[i];
            switch (dirStatus)
            {
            case 1:break;
            case 2:break;
            case 3:rho = -rho;break;
            case 4:rho = - rho;break;
            case 5:if((abs(robotGlobalDirection) < CV_PI / 10.0 )&& rho>0)rho = -rho;break;
            case 6:if(robotGlobalDirection > 0)rho = -rho;else ;
            default:break;
            }
            gaussianSum(rho,0);
        }
        for(int i = 0; i < yDirCnt; i++)
        {
            int rho = rhoOfY[i];
            switch (dirStatus)
            {
            case 1:break;
            case 2:rho = -rho;break;
            case 3:rho = -rho;break;
            case 4:break;
            case 5:if(abs(robotGlobalDirection) < CV_PI / 10.0 && rho > 0)rho = -rho;break;
            case 6:
		if(robotGlobalDirection > 0)
		{
		    if(rho<0)rho = - rho;
		}
		else if(rho > 0)
		{
		    rho = -rho;
		}
		break;
            default:break;
            }
            gaussianSum(rho,1);
        }


        imshow("hough lines",warpedImg);



        //trying to find the fitest grid by using traversal
        int xMax = 0, yMax = 0; //rho theta stores the value of fitest gridline
        for(int i = maxLensInImg; i <= maxLensInImg + pixelsCntPerCentimeter * 29 ; i += 3)
        {
            //int tempPos = i + maxLensInImg;
            int xSumValue = 0, ySumValue = 0;
            for(int k = i; k <= maxLensInImg * 2 + 9 ; k+= pixelsCntPerCentimeter * 30)//100 pixels = 30cm = 1 square
            {
                for( int j = 1; j <= 9 * photoReductionScale; j++)
                {
                    xSumValue += xGridLinesFitting[k + j - 5];
                    xSumValue += abs(xGridLinesFitting[2 * i - k + j - 5]);
                    ySumValue += yGridLinesFitting[k + j - 5];
                    ySumValue += yGridLinesFitting[2 * i - k + j - 5];
                }
                
            } 
            if(xSumValue > xMax) 
            {
                xMax = xSumValue;
                xRho = i - maxLensInImg;
            }
            if(ySumValue > yMax) 
            {
                yMax = ySumValue;
                yRho = i - maxLensInImg;
            }
        }
        
        //calculate robot global position
        double deltaX = xRho - xPreviousRho,deltaY = yRho - yPreviousRho;
        if(deltaX < -pixelsCntPerCentimeter * 17) yGlobal++;
        else if(deltaX > pixelsCntPerCentimeter * 17) yGlobal--;
        if(deltaY < -pixelsCntPerCentimeter * 17) xGlobal++;
        else if(deltaY > pixelsCntPerCentimeter * 17) xGlobal--;
        
        //draw grid lines
        Point pt1, pt2;
        double a1,b1,x0,y0,a2,b2;
            a1 = cos(xAverageDirection), b1 = sin(xAverageDirection);
            x0 = a1*xRho, y0 = b1*xRho;
            pt1.x = cvRound(x0 + 1000*(-b1));pt1.y = cvRound(y0 + 1000*(a1));
            pt2.x = cvRound(x0 - 1000*(-b1));pt2.y = cvRound(y0 - 1000*(a1));
        line(warpedImg,pt1,pt2,Scalar(0,255,255),4,LINE_AA);
            a2 = cos(yAverageDirection), b2 = sin(yAverageDirection);
            x0 = a2*yRho, y0 = b2*yRho;
            pt1.x = cvRound(x0 + 1000*(-b2));pt1.y = cvRound(y0 + 1000*(a2));
            pt2.x = cvRound(x0 - 1000*(-b2));pt2.y = cvRound(y0 - 1000*(a2));
        line(warpedImg,pt1,pt2,Scalar(255,255,255),4,LINE_AA);





        //draw debug data of gaussion sum 
         Mat xCanvas(int(maxLensInImg *2 + 20) , gaussianSumMax,CV_8UC3,Scalar(255,255,255,0.5));
         Mat yCanvas(int(maxLensInImg *2 + 20) , gaussianSumMax,CV_8UC3,Scalar(255,255,255,0.5));
         line(xCanvas,Point(maxLensInImg,0),Point(maxLensInImg,gaussianSumMax),Scalar(0,0,0),2,LINE_AA);//y axis
         line(yCanvas,Point(maxLensInImg,0),Point(maxLensInImg,gaussianSumMax),Scalar(0,0,0),2,LINE_AA);//y axis
         for(int i = 0;i<maxLensInImg *2 + 20;i++)
         {
             line(xCanvas,Point(i,0),Point(i,xGridLinesFitting[i]),Scalar(255,0,0),1,LINE_8);
             line(yCanvas,Point(i,0),Point(i,yGridLinesFitting[i]),Scalar(255,0,0),1,LINE_8);
         }
         line(xCanvas,Point(xRho + maxLensInImg,0),Point(xRho + maxLensInImg,300),Scalar(255,0,0),1,LINE_8);
         line(yCanvas,Point(yRho + maxLensInImg,0),Point(yRho + maxLensInImg,300),Scalar(255,0,0),1,LINE_8);
         imshow("xxx",xCanvas);
         imshow("yyy",yCanvas);


         
        

        
        
        
        
        
        
        //reset
        xDirectionOfPreviousFrame = xAverageDirection;
        yDirectionOfPreviousFrame = yAverageDirection;
        xPreviousRho = xRho;
        yPreviousRho = yRho;
        

        //calculate precise data
        double  preciseXGlobal = xGlobal + yRho/48.0,
                preciseYGlobal = yGlobal + xRho/48.0;
	
        volatile double robotPreciseX = preciseXGlobal + lenth * cos(robotGlobalDirection+tempTheta); 
	volatile double robotPreciseY = preciseYGlobal + lenth * sin(robotGlobalDirection+tempTheta);

        //output final data
        std::cout<< "x  " << xGlobal <<"  y  "<<yGlobal<<std::endl;
        std::cout<< "xp  " << preciseXGlobal <<"  yp  "<<preciseYGlobal;
        std::cout<<std::endl;
        std::cout<< "  bot x " << robotPreciseX <<" bot y  "<<robotPreciseY;
        std::cout<<"  bot dir  " << robotGlobalDirection ;
        std::cout<<std::endl;

        //debug data of mapping
        circle(theMap,
	           Point(robotPreciseX * 50 + 50,
		             500 - (robotPreciseY * 50 + 50)),
	       3,
	       (0,0,0),
	       -1); 
        circle(theMap,
	           Point(preciseXGlobal * 50 + 50,
		             500 - (preciseYGlobal * 50 + 50)),
	       3,
	       (0,0,200),
	       1);   
        imshow("mapppppp",theMap);


        //publish position
        positionDataToSend.data.clear();
        positionDataToSend.data.push_back(robotPreciseX );
        positionDataToSend.data.push_back(robotPreciseY );
        positionDataToSend.data.push_back(robotGlobalDirection );
        PositionPublisher.publish(positionDataToSend);
        ROS_INFO("I published something!");
        //ros::spinOnce();// do not need it if have no callback

        imshow("gridLines",warpedImg);


	    waitKey(1);

    
    
    }
    //outfile.close();
    return 0;
}

