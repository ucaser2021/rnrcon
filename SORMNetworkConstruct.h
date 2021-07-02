#ifndef _SORMNETWORKCONSTRUCT_H
#define _SORMNETWORKCONSTRUCT_H
#include "Road_Centerline_Extraction.h"
void find_lt_float(Mat image,double num,vector<Point> &points);
void find_lt_double(Mat image,double num,vector<Point> &points);
void FindConnCliques(vector<Line> &linelist,Mat &Adj,vector<vector<int>> &Clqs);
#endif