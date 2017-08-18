#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

/*-----------------------------------��GetLineCorner( )������------------------------------------
����������ֱ�ߴأ��õ�������ֱ�ߵĽ���
vector<Vec2f>lines������ֱ�ߴ�
vector<Point>&corners������Ľ��㼯
float threshold�����ڽ������С����
int width, int height��ͼ��Ŀ�͸�
------------------------------------------------------------------------------------------------*/
int GetLineCorner(vector<Vec2f>lines, vector<Point>&corners, float threshold, int width, int height);
