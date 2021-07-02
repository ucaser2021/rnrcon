#include "Road_Centerline_Extraction.h"
Mat dele_e(Mat img,Mat index)
{
	Mat output = img.clone();
	for(int i =0;i<index.rows;i++)
	{
		output.at<uchar>(int(index.at<double>(i,1)),int(index.at<double>(i,0))) = 0;
	
	}
	return output;


}
Mat Area_Selection(Mat f,double min_length,double minarea,double theta)
{
	Mat f_0,fig;
	if((theta<=45 && theta>=0)||(theta>=135 && theta<=170))
	{
	   Mat structureElement = getStructuringElement(MORPH_RECT, Size(3, 7), Point(-1, -1));
	   
	   morphologyEx(f, f_0, MORPH_CLOSE,structureElement );
	
	}
	else
	{
	 if((theta< 130 && theta>50))
	 {
	   Mat structureElement = getStructuringElement(MORPH_RECT, Size(7, 3), Point(-1, -1));
	   
	   morphologyEx(f, f_0, MORPH_CLOSE,structureElement );
	
	 }
	 else
	 {
	    Mat structureElement = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	   
	    morphologyEx(f, f_0, MORPH_CLOSE,structureElement );
	 
	 }
	
	
	}
	Mat L;
	int num;
	L = bwlabel(f_0, &num, 255);
	Region* stats = regionprops(L, num);
	Mat figo;
	for(int i =0;i<num;i++)
	{
		if(stats[i].MajorAxisLength<min_length || stats[i].Area <minarea )
		{
			figo = dele_e(f_0,stats[i].PixelList);
			f_0 = figo.clone();
		  
		}
	
	}
	return f_0;

}