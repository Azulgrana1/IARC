
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cmath>
#include <string>
#include <iostream>
#include "uEye.h"

using namespace cv;
using namespace std;

struct PosePara
{
	double tx;
	double ty;
	double theta;
	double height;
};

static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step, Mat& Pose,
	double, const Scalar& color)
{
	std::vector<Point2f> pre, cur;


	for (int y = 0; y < cflowmap.rows; y += step)
	{
		for (int x = 0; x < cflowmap.cols; x += step)
		{

			const Point2f& fxy = flow.at<Point2f>(y, x);
			pre.push_back(Point2f(x, y));
			cur.push_back(Point2f(x + fxy.x, y + fxy.y));

			
			
			line(cflowmap, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
				color);
			circle(cflowmap, Point(x, y), 2, color, -1);
			
		}

	}


	Pose = estimateRigidTransform(pre, cur, false);



}


int calcPose(PosePara &Pose, Mat dPose)
{
	double scale = 1;
	
	scale = dPose.at<double>(0)*dPose.at<double>(4) - dPose.at<double>(1)*dPose.at<double>(3);
	if ( scale > 0 )
	{
		scale = sqrt(scale);
		Pose.height /= scale;
	}
	else
	{
		return 2;	//scale error
	}

	if ( abs( dPose.at<double>(3) ) <= 1 )
	{
		Pose.theta += asin(dPose.at<double>(3) / scale);//;* 180 / 3.1416;
	}
	else
	{
		return 3;	//angle error
	}


	Pose.tx += dPose.at<double>(2) * Pose.height;
	Pose.ty += dPose.at<double>(5) * Pose.height;

	

	return 1;

}



int main(int argc, char** argv)
{


	

	VideoCapture cap(0);
	if (!cap.isOpened())
	{
		cout << "Can't open the camera!" << endl;
		return -1;
	}


	Mat flow, cflow, frame;
	Mat gray, prevgray, uflow;
	PosePara pp;

	namedWindow("flow", 1);

	PosePara Pose;
	Pose.height = 1;
	Pose.theta = 0;
	Pose.tx = 0;
	Pose.ty = 0;

	Mat dPose;


	while (1)
	{

		uint start, end, duration;

		start = cvGetTickCount();

		cap >> frame;

		
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		

		string str_tx;
		char ctx[256];
		sprintf(ctx,"%f",Pose.tx);
		str_tx = "x:" + string(ctx);

		string str_ty;
		char cty[256];
		sprintf(cty, "%f", Pose.ty);
		str_ty = "y:" + string(cty);

		string str_t;
		char ct[256];
		sprintf(ct, "%f", Pose.theta);
		str_t = "theta:" + string(ct);

		if (!prevgray.empty())
		{
			calcOpticalFlowFarneback(prevgray, gray, uflow, 0.5, 3, 15, 3, 5, 1.2, 0);
			cvtColor(prevgray, cflow, COLOR_GRAY2BGR);
			uflow.copyTo(flow);
			drawOptFlowMap(flow, cflow, 20, dPose, 1.5, Scalar(0, 255, 0));

			calcPose(Pose, dPose);

			putText(cflow, str_tx, Point(10, 20), 1, 1, Scalar(0, 255, 0));
			putText(cflow, str_ty, Point(10, 40), 1, 1, Scalar(0, 255, 0));
			putText(cflow, str_t, Point(10, 60), 1, 1, Scalar(0, 255, 0));

			end = cvGetTickCount();
			duration = end - start;

			string fps;
			char fpsc[20];
			double dua;
			dua = cvGetTickFrequency()*1000000/(double)duration;
			sprintf(fpsc,"%f", dua);
			fps = "fps:" + string(fpsc);
			
			putText(cflow, fps, Point(10, 80), 1, 1, Scalar(0, 255, 0));
			cout << Pose.height << endl;

			imshow("flow", cflow);
			//cout << pp.theta << endl;
		}
		

		if (waitKey(1) >= 0)
			break;
		std::swap(prevgray, gray);
	}
	return 0;
}

