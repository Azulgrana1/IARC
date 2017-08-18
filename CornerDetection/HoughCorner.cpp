#include "HoughCorner.h"

int PointDistance(Point x, Point y)
{
	return abs(x.x - y.x) + abs(x.y - y.y);
}

/*-----------------------------------��GetLineCorner( )������------------------------------------
����������ֱ�ߴأ��õ�������ֱ�ߵĽ���
vector<Vec2f>lines������ֱ�ߴ�
vector<Point>&corners������Ľ��㼯
float threshold�����ڽ������С����
int width, int height��ͼ��Ŀ��͸�
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
	//��1���Լ�⵽��ֱ�߽��з��࣬������Ӧ����������
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

	//��2��������ֱ�ߵĽ��㣻�˴���δ����theta = pi/2 theta = 0�����
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
	//��3��������Ͻ��Ľ���ϲ�������һ���ǵ�
	int count = 0;
	corners.push_back(intersections[0]);
	for (size_t i = 1; i < intersections.size(); i++)
	{
		for (size_t j = 0; j < corners.size(); j++)
		{
			if (PointDistance(corners[j], intersections[i]) < threshold)
			{
				//ʲô������ ��ʱ
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

/*-----------------------------------��GetLineCorner( )������------------------------------------
����������ǰһ֡�͵�ǰ֡������������У��õ�����֡�����λ��
vector<Point>last��	��һ֡��������
vector<Point>current����һ֡��������
Point &MoveVector��	���λ��
------------------------------------------------------------------------------------------------*/
int GetMove(vector<Point>last, vector<Point>current, Point &MoveVector)
{

}