#include "Road_Centerline_Extraction.h"

Mat Feature_Binarization(vector<Mat> MS_Feat,int dm,int dn)
{

	double T_r[3],T_c[3];
	Scalar r_mean,c_mean;
	for(int i =0;i<3;i++)
	{
		r_mean = mean(MS_Feat[i*3]);
		T_r[i] = r_mean[0] * 0.85;
		c_mean = mean(MS_Feat[i*3+1]);
		T_c[i] = c_mean[0] * 1.1;
	}


	Mat output = Mat::zeros(dm,dn,CV_8UC1);
	Mat output1;
	Mat out_curr,theta_curr,out_new0,out_new,out_curro,out_curr1;
	int theta;
	for(int j = 0;j<=17;j++)
	{
		theta = j*10;
		out_curr = Mat::zeros(dm,dn,CV_8UC1);
		for(int k=0;k<3;k++)
		{
			
			if (theta == 0)
			{
				bitwise_or(MS_Feat[k*3+2]>=175,MS_Feat[k*3+2]<=theta+4,theta_curr);
			}
			else
			{
			   bitwise_and(MS_Feat[k*3+2]>=theta-5,MS_Feat[k*3+2]<=theta+4,theta_curr);
			}
			bitwise_and(MS_Feat[k*3+0]<=T_r[k],MS_Feat[k*3+1]<=T_c[k],out_new0);
			bitwise_and(out_new0,theta_curr,out_new);
			out_curro = out_curr.clone();
			bitwise_or(out_new,out_curro,out_curr);
		
		}
		out_curr1 = Area_Selection(out_curr,15,40,theta);
		out_curr = out_curr.clone();
		bitwise_or(output,out_curr,output1);
		output = output1.clone();
	
	
	}

	return output;



}