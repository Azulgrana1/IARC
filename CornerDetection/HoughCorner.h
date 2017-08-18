#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

/*-----------------------------------【GetLineCorner( )函数】------------------------------------
描述：输入直线簇，得到正交的直线的交点
vector<Vec2f>lines：输入直线簇
vector<Point>&corners：输出的交点集
float threshold：相邻交点的最小距离
int width, int height：图像的宽和高
------------------------------------------------------------------------------------------------*/
int GetLineCorner(vector<Vec2f>lines, vector<Point>&corners, float threshold, int width, int height);
