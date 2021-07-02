#ifndef _SORM_H
#define _SORM_H
#include "Road_Centerline_Extraction.h"
void find(Mat image,int num,Mat &points);
void ClusterCentersInit(int imH,int imW,int d,Mat &CC);
Mat GetMemberInd(Mat A,Mat B);
double GetMidNumNoSort(int *arr,int size);
Mat GetDistMatrix(Mat c);
void find_lt(Mat image,int num,vector<Point> &points);
void find_ge(Mat image,int num,vector<Point> &points);
int round_double(double number);
void KMedians(Mat &m,Mat &sp, Mat &CC,int min_dist,int minSampleCnt,Mat I);
void find_ge_int(Mat image,int num,vector<Point> &points);
void find_int(Mat image,int num,vector<Point> &points);
void find_lt_int(Mat image,int num,vector<Point> &points);
int find_lt_float(Mat image,int num);
void find_ge_float(Mat image,int num,vector<Point> &points);
#endif