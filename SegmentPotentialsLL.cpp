#include"Library.h"
double SegmentPotentialsLL(vector<Line> linelist,vector<uchar> labels,MRF mrf)
{
	double V =0;
	for(unsigned int i=0;i<labels.size();i++)
	{
	   if(labels[i]==1)
	   {
		   if(mrf.allowjunc ==0)
		   {
		     V =V;
		   }
		   else
		   {
			   V = V+linelist[i].SegPot[1];
		   
		   }
	   
	   
	   }
	   else
	   {
		   V = V+linelist[i].SegPot[0];
	   
	   
	   }
	
	
	}

	return V;


}


