#include "Road_Centerline_Extraction.h"
Mat dele_en(Mat img,Mat index)
{
	Mat output = img.clone();
	for(int i =0;i<index.rows;i++)
	{
		output.at<uchar>(int(index.at<double>(i,1)),int(index.at<double>(i,0))) = 0;
	
	}
	return output;


}
Mat cc_threshold(Mat bw,int ccmin,int majmin)
{

	Mat CC;
	Mat bw1 = bw.clone();
	int num;
	CC = bwlabel(bw, &num, 255);
	Region* stats = regionprops(CC, num);
	vector<double> ratio;
	if(majmin)
	{
	   for(int i=0;i<num;i++)
	   {
		   ratio.push_back(double(stats[i].MajorAxisLength)/double(stats[i].MinorAxisLength));
	   
	   }
	
	}
	Mat bw_n;
	bool del = false;
	for(int i=0;i<num;i++)
	{   del = false;
		if(stats[i].Area<=ccmin)
		{
		  del = true;
		}
		else
		{
			if(majmin)
			{
				if((stats[i].MinorAxisLength>3)||(ratio[i]<15))
				{
				 del = true;
				}
			
			}
		
		}
		if(del)
		{
		    bw_n =  dele_en(bw1,stats[i].PixelList);
			bw1 = bw_n.clone();
		
		}

	
	}
	return bw1;



}