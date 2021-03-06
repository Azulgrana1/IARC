#include "HoughCorner.h"

int PointDistance(Point x, Point y)
{
	return abs(x.x - y.x) + abs(x.y - y.y);
}

/*-----------------------------------【GetLineCorner( )函数】------------------------------------
描述：输入直线簇，得到正交的直线的交点
vector<Vec2f>lines：输入直线簇
vector<Point>&corners：输出的交点集
float threshold：相邻交点的最小距离
int width, int height：图像的宽和高
------------------------------------------------------------------------------------------------*/
int GetLineCorner(vector<Vec2f>lines, vector<Point>&corners, float threshold, int width, int height)
{
	corners.clear();
	vector<Point> intersections;
	vector<vector<Point> > classfied;
	if (!lines.size())
	{
		return 0;
	}
	double a, b, c, d, e, f;
	Point corner;
	//for (size_t i = 0; i < lines.size(); i++)
	//{
	//	cout << i << ". r = " << lines[i][0] << "; theta = " << lines[i][1] << endl;
	//}
	//【1】对检测到的直线进行分类，这两类应该是正交的
	vector<Vec2f>Lx,Ly;
	Lx.push_back(lines[0]);
	for (size_t i = 1; i < lines.size(); i++)
	{
		if ( ( abs(lines[i][1] - lines[0][1]) > 3 * CV_PI / 8) && (abs(lines[i][1] - lines[0][1]) < 5 * CV_PI / 8) )
		{
			Ly.push_back(lines[i]);
		}
		else
		{
			Lx.push_back(lines[i]);
		}
	}

	//【2】求两类直线的交点；此处尚未考虑theta = pi/2 theta = 0的情况
	for (size_t i = 0; i < Lx.size(); i++)
	{
		a = cos(Lx[i][1]);
		b = sin(Lx[i][1]);
		c = Lx[i][0];
		for (size_t j = 0; j < Ly.size(); j++)
		{
			d = cos(Ly[j][1]);
			e = sin(Ly[j][1]);
			f = Ly[j][0];
			corner.x = -(b*f - e*c) / (a*e - b*d);
			corner.y = -(a*f - d*c) / (b*d - a*e);
			if (corner.x < width && corner.y < height)
			{
				intersections.push_back(corner);
			}
		}
	}
	if (intersections.size()<1)
	{
		return 0;
	}
	//【3】将距离较近的交点合并，当作一个角点
	int count = 0;
	corners.push_back(intersections[0]);
	for (size_t i = 1; i < intersections.size(); i++)
	{
		for (size_t j = 0; j < corners.size(); j++)
		{
			if (PointDistance(corners[j], intersections[i]) < threshold)
			{
				//什么都不做 暂时
				corners[j].x = intersections[i].x*0.1 + corners[j].x*0.9;
				corners[j].y = intersections[i].y*0.1 + corners[j].y*0.9;
				break;
			}
			else
			{	
				count ++;
			}
			if (count == corners.size())
			{
				corners.push_back(intersections[i]);
			}
		}
		count = 0;
	}

	return 1;
}

/*-----------------------------------【GetLineCorner( )函数】------------------------------------
描述：输入前一帧和当前帧的特征点的数列，得到这两帧的相对位移
vector<Point>last：	上一帧的特征点
vector<Point>current：下一帧的特征点
Point &MoveVector：	相对位移
------------------------------------------------------------------------------------------------*/
int GetMove(vector<Point>last, vector<Point>current, Point &MoveVector)
{

}
