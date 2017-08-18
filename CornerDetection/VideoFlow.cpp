
//-----------------------------------【头文件包含部分】---------------------------------------
//		描述：包含程序所依赖的头文件
//---------------------------------------------------------------------------------------------- 
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>  
#include <iostream>
#include "HoughCorner.h"
//-----------------------------------【命名空间声明部分】---------------------------------------
//		描述：包含程序所使用的命名空间
//----------------------------------------------------------------------------------------------- 
using namespace cv;
using namespace std;
//-----------------------------------【main( )函数】--------------------------------------------
//		描述：控制台应用程序的入口函数，我们的程序从这里开始
//-----------------------------------------------------------------------------------------------
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

int main()
{
        //【1】打开摄像头和Mat变量定义
        VideoCapture Cap;					//打开编号为0的摄像头 打开失败则推出程序
        Cap.open(0);
        if (!Cap.isOpened())
	{
            return 1;
	}
	Mat capframe;							//从摄像头读取的一帧图像
	Mat grayframe;							//读取后的图像转化为灰度图
	Mat cannyframe;							//进行边缘检测后的一帧图像
	bool STOP = false;
	//本机摄像头 640*480
        Cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
        Cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	uint start, end;
	
	Cap >> capframe;
	int width, height;
	width = capframe.size().width;
	height = capframe.size().height;
	Point center(width/2,height/2);
	while ( !STOP )
	{

		//【2】读取一帧图像，转化为灰度图
		start = cvGetTickCount();
		Cap >> capframe;
	
		
		
		cvtColor(capframe, grayframe, CV_BGR2GRAY);
	
		//【3】进行canny边缘检测(需要先进行高斯滤波)

		GaussianBlur(grayframe, grayframe, Size(5, 5), 1.5, 1.5);
		Canny(grayframe, cannyframe, 5, 50, 3);
		
		//【3】进行霍夫线变换
		vector<Vec2f> lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
		vector<Point> corners;
		HoughLines(cannyframe, lines, 3, 3.1416 / 180, 150, 0, 0);
		GetLineCorner(lines, corners, 50, width , height );
		//【5】依次在图中绘制出每条线段
		for (size_t i = 0; i < lines.size(); i++)
		{
			float rho = lines[i][0], theta = lines[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000 * (-b));
			pt1.y = cvRound(y0 + 1000 * (a));
			pt2.x = cvRound(x0 - 1000 * (-b));
			pt2.y = cvRound(y0 - 1000 * (a));
			line(capframe, pt1, pt2, Scalar(55, 100, 195), 1, CV_AA);
		}

		for (size_t j = 0; j < corners.size(); j++)
		{
			circle(capframe, corners[j], 3, Scalar(0, 0, 255), 2);
		}
	
		//circle(cannyframe, center, 10, Scalar(0, 0, 255), 5);
		
                imshow("Line", capframe);
		
		if (waitKey(1) >= 0)
			STOP = true;

                //end = cvGetTickCount();
                //cout << (int)((end - start) / cvGetTickFrequency() / 1000) << endl;

	}
    return 0;
}
