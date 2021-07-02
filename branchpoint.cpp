#include "Road_Centerline_Extraction.h"

vector<Point2i> find_branchpoints(Mat &img)
{
	vector<Point2i> branch_locations;
	for(int i=1;i<img.cols;i++)
	{
		bool lit = false;
		vector<int> begin_black_regions;
		vector<int> end_black_regions;

		if(255 == img.at<unsigned char>(0,i))
		{
			lit = true;
		
		}
		else
		{
			lit = false;
			begin_black_regions.push_back(0);
		
		}
		for(int j=1;j<img.rows-1;j++)
		{
			if(255 == img.at<unsigned char>(j,i) && lit == false)
			{
				lit = true;
				end_black_regions.push_back(j-1);
			
			}
			else if( 0 == img.at<unsigned char>(j,i) && lit == true)
			{
				lit = false;
				begin_black_regions.push_back(j);
			
			
			}
		
		
		}

		if(0 == img.at<unsigned char>(img.rows -1,i) && lit == false)
		{
			end_black_regions.push_back(img.rows -1);
		
		}
		else if(0 == img.at<unsigned char>(img.rows -1,i) && lit == true)
		{
			begin_black_regions.push_back(img.rows - 1);
			end_black_regions.push_back(img.rows - 1);
		
		}
		else if (255 == img.at<unsigned char>(img.rows -1 ,i) && lit == false)
		{
			end_black_regions.push_back(img.rows - 2);
		
		}
		if(begin_black_regions.size() != end_black_regions.size())
		{
		
			cout << begin_black_regions.size() << " " << end_black_regions.size() << endl;
		}

		for(size_t k =0;k<begin_black_regions.size();k++)
		{
			bool found_branch = true;
			for(int l = begin_black_regions[k]; l<=end_black_regions[k];l++)
			{
				if(0 == img.at<unsigned char>(l,i-1))
				{
					found_branch = false;
					break;
				
				}
			
			}
		
		
		if(found_branch == true)
		{
			Point2i location(i-1,begin_black_regions[k]);
			branch_locations.push_back(location);
		
		}
	
		
	}
	
	
	}

	return branch_locations;




}
