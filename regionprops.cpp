#include "Road_Centerline_Extraction.h"

Region* regionprops(Mat L, int seg_num)
{
	Region* region = new Region[seg_num];
	Mat loc;
	int value;
	for(int i=0;i<seg_num;i++)
	{
		region[i].Area = 0;
	
	}
	
	// Area and PixelList
	loc.create(1, 2, CV_64F);
	//vector<double> loc;
	for (int row = 0; row < L.rows; row++)
	{
		for (int col = 0; col < L.cols; col++)
		{
			value = L.at<int>(row, col) - 1;

			if (value >= 0)
			{

				region[value].Area++;

				loc.at<double>(0, 0) = col + 0.0;
				loc.at<double>(0, 1) = row + 0.0;
				if(region[value].PixelList.rows == 0 || region[value].PixelList.cols==0)
				{
					region[value].PixelList=loc.clone();
				
				}
				else
				{
				   region[value].PixelList.push_back(loc);
				}

				
			}			
		}
	}

	// Centroid
	for (int i = 0; i < seg_num; i++)
	{
		region[i].Centroid.create(1, 2, CV_64F);
		reduce(region[i].PixelList, region[i].Centroid, 0, CV_REDUCE_AVG);
	}

	// MajorAxisLength, MinorAxisLength
	Mat list;
	Mat x, y;
	Mat temp;

	double xbar, ybar;
	double uxx, uyy, uxy;
	double common;
	double Two_sqrt_2 = 2.0 * sqrt(2.0);

	for (int i = 0; i < seg_num; i++)
	{
		list = region[i].PixelList;
		xbar = region[i].Centroid.at<double>(0, 0);
		ybar = region[i].Centroid.at<double>(0, 1);

		x = list.col(0) - xbar;
		y = list.col(1) - ybar;

		uxx = cv::sum(x.mul(x))[0] / (x.rows) + 1.0 / 12.0;
		uyy = cv::sum(y.mul(y))[0] / (x.rows) + 1.0 / 12.0;
		uxy = cv::sum(x.mul(y))[0] / (x.rows);		

		//cout << region[i].Centroid << "  " << ybar << "  " << uxy << endl;

		common = sqrt(pow((uxx - uyy), 2) + 4.0 * pow(uxy, 2));
		region[i].MajorAxisLength = Two_sqrt_2 * sqrt(uxx + uyy + common);
		region[i].MinorAxisLength = Two_sqrt_2 * sqrt(uxx + uyy - common);
	}

	return region;
}