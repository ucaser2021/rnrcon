#ifndef _ROADTEMPLATEMATCHINGFILTER_H
#define _ROADTEMPLATEMATCHINGFILTER_H
#include "Road_Centerline_Extraction.h"
vector<Mat> gen_filter_bank_int2(int ws ,int min_rw,int max_rw);
Mat filter_balance(Mat &filt);
#endif