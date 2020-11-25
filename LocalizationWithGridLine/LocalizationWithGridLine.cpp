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
    double xDirectionOfPreviousFrame = CV_PI/2,yDirectionOfPreviousFrame = 0;//theta in img.
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
	    
        //Transform perspective
        Mat lambda = getPerspectiveTransform(perspectiveTransformOriginPoint,perspectiveTransformWarpedPointa);
        warpPerspective(originImg,warpedImg,lambda, Size(320,300),INTER_LINEAR);

	    //1line(warpedImg,Point(30,30),Point(130,30),Scalar(0,0,255),2,LINE_AA);
        //imshow("warpedImg",warpedImg);
        
        
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
        //imshow("GaussialBlur",warpedGrayImg);
        Canny(warpedGrayImg,warpedGrayImg,cannyMinThreshold,cannyMaxThreshold,3);
        //imshow("canny",warpedGrayImg);
	    std::vector<Vec2f>lines;
	    HoughLines(warpedGrayImg,lines,1,CV_PI/180,houghLineThreshold,0,0);
	
	

        //filter the parallel lines of x and y. then add them in to 1D mat using gaussing func
        // Mat xGridLinesFitting = Mat::zeros( maxLensInImg + 5,1,CV_32F);//to calculate the fitest grid lines
        // Mat yGridLinesFitting = Mat::zeros( maxLensInImg + 5,1,CV_32F);//to calculate the fitest grid lines
	    memset(xGridLinesFitting,0,sizeof(xGridLinesFitting));
        memset(yGridLinesFitting,0,sizeof(yGridLinesFitting));
        double xAverageDirection = 0,yAverageDirection = 0, xCountOfAverage = 0, yCountOfAverage = 0;
        
        
        //std::cout<<lines.size()<<std::endl;
        
        //debug data
        std::ofstream outfile;
        outfile.open("/home/pi/degreeData",std::ios::out | std::ios::app);




        
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
            //DONE: direction changed from pi to 0
            if(abs(sin(theta - xDirectionOfPreviousFrame)) < sin(rotationThreshold))//delta from previous frame 
	        {
                if((xDirectionOfPreviousFrame < rotationThreshold || xDirectionOfPreviousFrame > CV_PI - rotationThreshold) && theta > CV_PI / 1.5)
                {
                    theta -= CV_PI;
                    rho = - rho;
                }
                gaussianSum(rho,0);
                xCountOfAverage++;
                xAverageDirection += theta;
            }
            else if(abs(sin(theta - yDirectionOfPreviousFrame)) < sin(rotationThreshold))//delta from previous frame 
	        {
                if((yDirectionOfPreviousFrame < rotationThreshold || yDirectionOfPreviousFrame > CV_PI - rotationThreshold) && theta > CV_PI / 1.5)
                {
                    theta -= CV_PI;
                    rho = - rho;
                }
                gaussianSum(rho,1);
                yCountOfAverage++;
                yAverageDirection += theta;
            }

	        line(warpedImg,pt1,pt2,Scalar(0,0,255),2,LINE_AA);

        }

        
        imshow("hough lines",warpedImg);

        xAverageDirection = xAverageDirection / double(xCountOfAverage);// from -15 deg to 170 deg (using radian)
        yAverageDirection = yAverageDirection / double(yCountOfAverage);
        

        //trying to find the fitest grid by using traversal
        int xMax = 0, yMax = 0; //rho theta stores the value of fitest gridline
        for(int i = maxLensInImg; i <= maxLensInImg + pixelsCntPerCentimeter * 29 ; i += 2)
        {
            //int tempPos = i + maxLensInImg;
            int xSumValue = 0, ySumValue = 0;
            for(int k = i; k <= maxLensInImg * 2 + 9 ; k+= 100)//100 pixels = 30cm = 1 square
            {
                for( int j = 1; j <= 7; j++)
                {
                    xSumValue += xGridLinesFitting[k + j - 5];
                    //xSumValue += xGridLinesFitting[i - k + j - 5];
                    ySumValue += yGridLinesFitting[k + j - 5];
                    //ySumValue += yGridLinesFitting[int(2*maxLensInImg - k) + j - 5];
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
       

        //draw grid lines

        Point pt1, pt2;
        double a1,b1,x0,y0,a2,b2;
        a1 = cos(xAverageDirection), b1 = sin(xAverageDirection);
        x0 = a1*xRho, y0 = b1*xRho;
        pt1.x = cvRound(x0 + 1000*(-b1));
        pt1.y = cvRound(y0 + 1000*(a1));
        pt2.x = cvRound(x0 - 1000*(-b1));
        pt2.y = cvRound(y0 - 1000*(a1));
        line(warpedImg,pt1,pt2,Scalar(0,255,255),4,LINE_AA);
        
        a2 = cos(yAverageDirection), b2 = sin(yAverageDirection);
        x0 = a2*yRho, y0 = b2*yRho;
        pt1.x = cvRound(x0 + 1000*(-b2));
        pt1.y = cvRound(y0 + 1000*(a2));
        pt2.x = cvRound(x0 - 1000*(-b2));
        pt2.y = cvRound(y0 - 1000*(a2));
        line(warpedImg,pt1,pt2,Scalar(0,255,255),4,LINE_AA);





        //draw debug data of gaussion sum 

        // Mat xCanvas(int(maxLensInImg *2 + 20) , gaussianSumMax,CV_8UC3,Scalar(255,255,255,0.5));
        // Mat yCanvas(int(maxLensInImg *2 + 20) , gaussianSumMax,CV_8UC3,Scalar(255,255,255,0.5));
        // line(xCanvas,Point(maxLensInImg,0),Point(maxLensInImg,gaussianSumMax),Scalar(0,0,0),2,LINE_AA);//y axis
        // line(yCanvas,Point(maxLensInImg,0),Point(maxLensInImg,gaussianSumMax),Scalar(0,0,0),2,LINE_AA);//y axis
        // for(int i = 0;i<maxLensInImg *2 + 20;i++)
        // {
        //     line(xCanvas,Point(i,0),Point(i,xGridLinesFitting[i]),Scalar(255,0,0),1,LINE_8);
        //     line(yCanvas,Point(i,0),Point(i,yGridLinesFitting[i]),Scalar(255,0,0),1,LINE_8);
        // }
        // line(xCanvas,Point(xRho,0),Point(xRho,300),Scalar(255,0,0),1,LINE_8);
        // line(yCanvas,Point(yRho,0),Point(yRho,300),Scalar(255,0,0),1,LINE_8);
        // imshow("xxx",xCanvas);
        // imshow("yyy",yCanvas);


        //localization data calculation
        //robotGlobalDirection += -(xAverageDirection - xDirectionOfPreviousFrame);//i shall test the range of output
        //int xDeltaRho = xRho - xPreviousRho, yDeltaRho = yRho - yPreviousRho;
        // if(abs(xDeltaRho) >= pixelsCntPerCentimeter * 21)
        // {
        //    // if(sin(robotDirection) < 0)
        // }




        //here, i have the rho and theta of one x and one y axis
        // cos(xtheta) = a1 sin(xtheta) = b1 
        // cos y         a2 sin y         b2
        //xAverageDirection yAverageDirection
        // xRho yRho
        //get cross point
        int crossExists[8][8];
        int crossPosition[8][8][2];//0-x 1-y
        vector<Point>cellFlag;
        memset(crossExists,0,sizeof(crossExists));
        memset(crossPosition,0,sizeof(crossPosition));

        for(int i = -2; i <= 2; i++) //traversal different parellel line of x / y to get all the crosses
        {
            int xTempRho = xRho + i * 30 * pixelsCntPerCentimeter ;
            for(int k = -2; k <= 2; k++)
            {
                int yTempRho = yRho + i * 30 * pixelsCntPerCentimeter ;
                float crossX,crossY;
                crossX = (yTempRho / b2 - xTempRho / b1) / (a2 / b2 - a1 / b1);
                crossY = ( -a1 / b1) * crossX + xTempRho / b1;
                if(crossX <= 300 && crossX >= 0 && crossY <= 300 && crossY >= 0)//cross in img
                {
                    circle(warpedImg, Point(crossX,crossY), 5, Scalar(0, 255, 0), -1);
                    crossExists[int(crossX/60)][int(crossY/60)] = 1;
                    crossPosition[int(crossX/60)][int(crossY/60)][0] = crossX;
                    cellFlag.push_back(Point(int(crossX/60),int(crossY/60)));
                    crossPosition[int(crossX/60)][int(crossY/60)][1] = crossY;

                }
                


            }
        }
        //most of this is written late in the night. forgive me for the sh*t-like code
        int crossCnt = 0;
        double previousLine[2][2]; 
        double currentLine[2][2];
        int pointCnt = 0;
        for(int i = 0;i < crossFlag.size();i++)
        {
            int tempX = crossFlag[i].x;
            int tempY = crossFlag[i].y;
            for(int i2 = -1 ; i2 <= 1; i2++)
            {
                for(int i3 = -1;i3 <= 1;i3++)
                {
                    if(previousCrossExists[tempX + i2][tempY + i3] == 1
                        && (pow(previousCrossPosition[tempX + i2][tempY + i3][0] 
                        - crossPosition[tempX][tempY][0],2) 
                        + pow(previousCrossPosition[tempX + i2][tempY + i3][1] 
                        - crossPosition[tempX][tempY][1],2))
                        <= pow(10 * pixelsCntPerCentimeter, 2) )//if two point matches(between previous frame and current)
                    {
                        i2 = 2; i3 = 2;
                        previousLine[pointCnt][0] = previousCrossPosition[tempX + i2][tempY + i3][0];
                        previousLine[pointCnt][1] = previousCrossPosition[tempX + i2][tempY + i3][1];
                        currentLine[pointCnt][0] = crossPosition[tempX + i2][tempY + i3][0];
                        currentLine[pointCnt][1] = crossPosition[tempX + i2][tempY + i3][1];
                        pointCnt++;
                        if(pointCnt >= 2) i = crossFlag.size()+1;
                    }

                }
            }
        }
        //now i have two point in two frame respectively
        // i have to calculate the change between two frame.
        // i can see i m gonna to success!
        double previousTheta = atan2(previousLine[0][1]-previousLine[1][1],
                                    previousLine[0][0]-previousLine[1][0]);
        double currentTheta = atan2(currentLine[0][1]-currentLine[1][1],
                                    currentLine[0][0]-currentLine[1][0]);
        double deltaTheta_ = currentTheta - previousTheta;
        robotGlobalDirection += deltaTheta_;
        double rCurrentPoint1 = sqrt(pow(currentLine[0][0],2) + pow(currentLine[0][1],2);
        double point1Theta = - deltaTheta_ + atan2(currentLine[0][1],currentLine[0][0]);
        double transformedX = cos(point1Theta) * rCurrentPoint1;
        double transformedY = sin(point1Theta) * rCurrentPoint1;
        robotGlobalX += - (transformedX - previousLine[0][0]);
        robotGlobalY += - (transformedY - previousLine[0][1]);
        
        //reset
        //std::cout << "xDir: "<< xDirectionOfPreviousFrame << "  xRho: " << xRho << " yDir: " << yDirectionOfPreviousFrame 
         //        <<"  yRho: "<< yRho <<" bot dir: " << robotGlobalDirection << std::endl;
        std::cout << "x::" << robotGlobalX << "y::" << robotGlobalY << "dir" << robotGlobalDirection << std::endl;
        xDirectionOfPreviousFrame = xAverageDirection;
        yDirectionOfPreviousFrame = yAverageDirection;
        
        imshow("gridLines",warpedImg);

	    waitKey(10);

    
    
    }
    //outfile.close();
    return 0;
}

