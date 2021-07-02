#ifndef _LIBRARY_H
#define _LIBRARY_H
#include "Road_Centerline_Extraction.h"
double GetAngle(double x,double y);
int round_double(double number);
double GetLineProb(Mat pmap,Point pt1,Point pt2);
double SegmentPotentialsLL(vector<Line> linelist,vector<uchar> labels,MRF mrf);
double SormCliquePotential(vector<Line> &linelist,vector<uchar> &labels,vector<int> &Clique,MRF mrf);
vector<double> CliquePotentialsLL(vector<Line> &linelist,vector<uchar> &labels,vector<vector<int>> &Cliques,MRF mrf);
double MRFLocalEnergy(int node,vector<Line> &linelist,vector<uchar> &labels,vector<vector<int>> &Cliques,MRF mrf );
Mat mat2gray(Mat &img);
#endif