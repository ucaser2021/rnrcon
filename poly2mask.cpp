#include "Library.h"

Mat  poly2mask(vector<vector<Point>> pt,int M,int N)
{
	Mat mask = Mat::zeros(M,N,CV_8UC1);
	drawContours(mask,pt,0,255,-1);
	return mask;
}