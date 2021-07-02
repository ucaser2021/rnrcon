#ifndef _ROAD_CENTERLINE_EXTRACTION_H
#define _ROAD_CENTERLINE_EXTRACTION_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include <stdio.h>      
#include <math.h> 
using namespace cv;
using namespace std;
#define PI   3.141592653589793
#define INF  100000000.0
#define CV_SORT_EVERY_ROW    0
#define CV_SORT_EVERY_COLUMN 1
#define CV_SORT_ASCENDING    0
#define CV_SORT_DESCENDING   16
struct Line{
	Point s;
	Point e;
	double c[2];
	double len;
	double ang;
	Point adj;
	double nclq;
	double prob;
	int nr;
	int ClqS;
	int ClqE;
	int clq[2];
	double SegPot[2];

};
struct MRF
{
	int gnLevel ;
	int step ;
	double c ;
	int Ti ;
	int sorm;
	double Ke;
	double Kl;
	double Kc;
	double Ki;
	double Vo;
	double eratio;
	double CliqPotMin;
	int allowjunc;

};
typedef struct Region
{                            
	int Area;
	Mat PixelList;
	Mat Centroid;
	double MajorAxisLength;
	double MinorAxisLength;
};
vector<Mat>  Multiple_Feature_Extraction(Mat &img1 ,int* R);
Mat Feature_Binarization(vector<Mat> MS_Feat,int dm,int dn);
Mat RoadTemplateMatchingFilter(Mat &I,int N,int angle_step,int* road_range);
Mat skeletonization(Mat inputImage);
vector<Point2i> find_branchpoints(Mat &img);
Mat cc_threshold(Mat bw,int ccmin,int majmin);
vector<Point> sorm(Mat &I,double d,double minDist,int minSampleCnt);
void SORMNetworkConstruct(vector<Point> CC,Mat &pmap,float mxDist,float minProb,vector<Line> &linelist ,vector<vector <int>> &Cliques);
vector<uchar>  MRFRelaxWithICM(vector<Line> &linelist,vector<uchar> &iLabels, MRF mrf,vector<vector<int>> &Cliques);
Region* regionprops(Mat L, int seg_num);
Mat bwlabel(Mat _src, int* seg_num, int max_value);
Mat Area_Selection(Mat f,double min_length,double minarea,double theta);
#endif