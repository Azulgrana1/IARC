
//-----------------------------------��ͷ�ļ��������֡�---------------------------------------
//		����������������������ͷ�ļ�
//---------------------------------------------------------------------------------------------- 
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>  
#include <iostream>
#include "HoughCorner.h"
//-----------------------------------�������ռ��������֡�---------------------------------------
//		����������������ʹ�õ������ռ�
//----------------------------------------------------------------------------------------------- 
using namespace cv;
using namespace std;
//-----------------------------------��main( )������--------------------------------------------
//		����������̨Ӧ�ó������ں��������ǵĳ�������￪ʼ
//-----------------------------------------------------------------------------------------------
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

int main()
{
        //��1��������ͷ��Mat��������
        VideoCapture Cap;					//�򿪱��Ϊ0������ͷ ��ʧ�����Ƴ�����
        Cap.open(0);
        if (!Cap.isOpened())
	{
            return 1;
	}
	Mat capframe;							//������ͷ��ȡ��һ֡ͼ��
	Mat grayframe;							//��ȡ���ͼ��ת��Ϊ�Ҷ�ͼ
	Mat cannyframe;							//���б�Ե�����һ֡ͼ��
	bool STOP = false;
	//��������ͷ 640*480
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

		//��2����ȡһ֡ͼ��ת��Ϊ�Ҷ�ͼ
		start = cvGetTickCount();
		Cap >> capframe;
	
		
		
		cvtColor(capframe, grayframe, CV_BGR2GRAY);
	
		//��3������canny��Ե���(��Ҫ�Ƚ��и�˹�˲�)

		GaussianBlur(grayframe, grayframe, Size(5, 5), 1.5, 1.5);
		Canny(grayframe, cannyframe, 5, 50, 3);
		
		//��3�����л����߱任
		vector<Vec2f> lines;//����һ��ʸ���ṹlines���ڴ�ŵõ����߶�ʸ������
		vector<Point> corners;
		HoughLines(cannyframe, lines, 3, 3.1416 / 180, 150, 0, 0);
		GetLineCorner(lines, corners, 50, width , height );
		//��5��������ͼ�л��Ƴ�ÿ���߶�
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
