#include "Library.h"
Mat  poly2mask(vector<vector<Point>> pt,int M,int N)
{
	Mat mask = Mat::zeros(M,N,CV_8UC1);
	drawContours(mask,pt,0,255,-1);
	return mask;
}

double GetLineProb(Mat pmap,Point pt1,Point pt2)
{
	double tick = 2;
	double ang = GetAngle(double(pt1.x-pt2.x),double(pt1.y-pt2.y));

	double dx = tick * cos(ang/180*PI);
	double dy = tick * sin(ang/180*PI);
	vector<Point> pt;
	vector<vector<Point>> pt_contour;
	pt.push_back(Point(round_double(pt1.y-1-dx),round_double(pt1.x-1+dy)));
	pt.push_back(Point(round_double(pt1.y-1+dx),round_double(pt1.x-1-dy)));
	pt.push_back(Point(round_double(pt2.y-1+dx),round_double(pt2.x-1-dy)));
	pt.push_back(Point(round_double(pt2.y-1-dx),round_double(pt2.x-1+dy)));
	
	pt_contour.push_back(pt);
	Mat BW = poly2mask( pt_contour,pmap.rows,pmap.cols);
	Mat output ;
	bitwise_and(BW,pmap,output);
	Scalar ss = sum(output);
	Scalar ss_b = sum(BW);
	double prop = ss[0]/ss_b[0];

	return prop;

}