#include "sorm.h"
#include "Library.h"
double dist;
//打印一个Mat矩阵
void PrintMat(Mat A)
{
  for(int i=0;i<A.rows;i++)
  {
    for(int j=0;j<A.cols;j++)
      cout<<A.at<int>(i,j)<<' ';
    cout<<endl;
  }
  cout<<endl;
}
vector<Point> sorm(Mat &I,double d,double minDist,int minSampleCnt)
{
	dist = d;
	int imH = I.rows;
	int imW = I.cols;
	Mat m;
	Mat CCi;
	Mat CC;
	find(I,255,m);
	ClusterCentersInit(imH,imW, d,CCi);
	KMedians(m,CCi, CC,minDist, minSampleCnt, I);


	return CC;

}
Mat GetMemberInd(Mat A,Mat B)
{
	int hA = A.rows;
	int wA = 2;
	int hB = B.rows;
	Mat A_X =repeat(A(Rect(0,0,1,hA)).clone().reshape(0,1),hB,1);
	Mat A_Y =repeat(A(Rect(1,0,1,hA)).clone().reshape(0,1),hB,1);

	Mat B_X = repeat(B(Rect(0,0,1,hB)),1,hA);
	Mat B_Y = repeat(B(Rect(1,0,1,hB)),1,hA);

	Mat A_XF ;
	A_X.convertTo(A_XF,B_X.type(),1.0);
	Mat A_YF ;
	A_Y.convertTo(A_YF,B_Y.type(),1.0);
	Mat X =(A_XF-B_X).mul(A_XF-B_X);
	Mat Y =(A_YF-B_Y).mul(A_YF-B_Y);

	Mat D;
	add(X,Y,D);
	Mat indx;
	sortIdx(D,indx,CV_SORT_EVERY_COLUMN+CV_SORT_ASCENDING);
	Mat one = Mat::ones(1,hA,CV_32SC1);
	Mat ind;
	add(indx(Rect(0,0,hA,1)),one,ind);
	return ind;



}
void KMedians(Mat &m,Mat &sp, Mat &CC,int min_dist,int minSampleCnt,Mat I)
{
	Mat temp = Mat::zeros(1,m.rows,CV_32SC1);
	int k = sp.rows;
	Mat c = sp.clone();

	int iter = 0;
	Mat com1,com2;
	Scalar com1_sum,com2_sum;
	Mat ID;
	while(1)
	{
		
		ID = GetMemberInd(m,c);
		compare(temp, ID, com1, CMP_GT);
		com1_sum = sum(com1);
		compare(temp, ID, com2, CMP_LT);
		com2_sum = sum(com2);
		if (com1_sum[0]+com2_sum[0] == 0)
		{
			int j =0;
			Mat c2 = c.clone();
			Mat ds = Mat::zeros(k,1,CV_32FC1);
			for(int i=1;i<=k;i++)
			{
				vector<Point> f;
				find_int(ID,i,f);
				if (f.size()>=1)
				{
					ds.at<float>(i-1,0) = 1.0;
					j =j+1;
					c2.at<float>(j-1,0) = c.at<float>(i-1,0);
					c2.at<float>(j-1,1) = c.at<float>(i-1,1);
				
				}
				else
				{
					ds.at<float>(i-1,0)=0;
				
				}
			
			}

			if(j!=0)
			{
				Mat c_temp = c2(Rect(0,0,2,j));
				Mat c = c_temp.clone();
			}
			else
			{
				c.release();
			
			}

			ID = GetMemberInd(m,c);

			break;
		}
		else
		{

			temp = ID.clone();
		
		}

		for(int i =1;i<=k;i++)
		{
			vector<Point> f;
			find_int(ID,i,f);
			Mat x_i = Mat::zeros(1,f.size(),CV_32SC1);
			Mat y_i = Mat::zeros(1,f.size(),CV_32SC1);
			Mat x_is = Mat::zeros(1,f.size(),CV_32SC1);
			Mat y_is = Mat::zeros(1,f.size(),CV_32SC1);
			if(f.size()>0)
			{
				for(unsigned int g=0;g<f.size();g++)
				{
					x_i.at<int>(0,g)=m.at<int>(f[g].x-1,0);
					y_i.at<int>(0,g)=m.at<int>(f[g].x-1,1);
				}
				sort(x_i,x_is,CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
				sort(y_i,y_is,CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
				if(f.size()%2==0)
				{
					c.at<float>(i-1,0) = (x_is.at<int>(0,f.size()/2-1) + x_is.at<int>(0,f.size()/2))/2.0;
					c.at<float>(i-1,1) = (y_is.at<int>(0,f.size()/2-1) + y_is.at<int>(0,f.size()/2))/2.0;
				
				}
				else
				{
				  c.at<float>(i-1,0) = x_is.at<int>(0,(f.size()+1)/2-1);
				  c.at<float>(i-1,1) = y_is.at<int>(0,(f.size()+1)/2-1);

				}
				
			}
	
		
		}
		//Node Pruning: Merge the node pairs if the ecludian distance 
        //between them is below the min_dist.
        //
		Mat D = GetDistMatrix(c);
		//Check if there are nodes to be merged.
		int D_p = find_lt_float(D,min_dist);
		if(D_p>=1)
		{
			vector<Point> D_m;
			find_ge_float(D,min_dist,D_m);
			for(unsigned int i=1;i<=D_m.size();i++)
			{
				D.at<float>(D_m[i-1].x-1,D_m[i-1].y-1)=INF;
			
			
			}
			Mat c_pruned = c.clone();
			double minVal=0.e0;
			double maxVal=0.e0;
			Point minLoc;
			Point maxLoc;
			while(1)
			{
				//find the node pair in which the distance between the nodes is minimum.
				minMaxLoc(D,&minVal,&maxVal,&minLoc,&maxLoc);
				if(minVal<INF)
				{
					int jj = minLoc.x;
					//% remove one of the node in the pair by replacing the location of it by 0 
					c_pruned.at<float>(jj,0)=0;
					c_pruned.at<float>(jj,1)=0;

					// update the Distance matrix
                    //The Node jj is removed, so delete the entries related
                    //with node jj.ind2sub

					for(int i =0;i<D.rows;i++)
					{
						D.at<float>(i,jj)=INF;
						D.at<float>(jj,i)=INF;
					}
				}
				else
				{
					Mat c_curr;
					Mat loc ;
					loc.create(1,2,CV_32FC1);
					for(unsigned int i =0;i<c_pruned.rows;i++)
					{
						if(c_pruned.at<float>(i,0)!=0 && c_pruned.at<float>(i,1)!=0)
						{
							loc.at<float>(0,0) = c_pruned.at<float>(i,0);
							loc.at<float>(0,1) = c_pruned.at<float>(i,1);
							if(c_curr.rows==0)
							{
								c_curr = loc.clone();
							}
							else
							{
							  	c_curr.push_back(loc);
							}
						
						
						}

					
					}
					c = c_curr.clone();
					break;		
				
				}		
			}
		
		}
	}
	int cn = c.rows;
	k =0;
	Mat loc1;
	loc1.create(1,2,CV_32SC1);
	for(int i =1;i<=cn;i++)
	{
		vector<Point> m_i;
		find_int(ID,i,m_i);
		if(m_i.size()>=minSampleCnt)
		{
			k=k+1;
			loc1.at<int>(0,0) = round_double(double(c.at<float>(i-1,0)));
			loc1.at<int>(0,1) = round_double(double(c.at<float>(i-1,1)));
			if(CC.rows==0)
			{
				CC = loc1.clone();
			}
			else
			{
			  CC.push_back(loc1);
			}
			
		}		
	}


}
Mat GetDistMatrix(Mat c)
{
	int cn = c.rows;
	Mat c_x = c(Rect(0,0,1,cn));
	Mat c_y = c(Rect(1,0,1,cn));
	Mat X2 = repeat(c_x,1,cn);
	Mat X2_t = X2.t();
	Mat X1 = X2_t.reshape(0,1);
	X2 = X2.reshape(0,1);


	Mat Y2 = repeat(c_y,1,cn);
	Mat Y2_t = Y2.t();
	Mat Y1 = Y2_t.reshape(0,1);
	Y2 = Y2.reshape(0,1);

	Mat X_d = Mat::zeros(cn*cn,1,CV_32FC1);
	subtract(X1,X2,X_d);
	Mat Y_d = Mat::zeros(cn*cn,1,CV_32FC1);;
	subtract(Y1,Y2,Y_d);

	Mat D1,D2,D;

	multiply(X_d,X_d,D1);
	multiply(Y_d,Y_d,D2);

	add(D1,D2,D);
	sqrt(D,D);
	D = D.reshape(0,cn);

	for(int i=0;i<cn;i++)
	{
		for(int j=i;j<cn;j++)
		{
			D.at<float>(i,j) = INF;
		}
	
	}


	return D;



}
void ClusterCentersInit(int imH,int imW,int d,Mat &CC)
{
	int crow_cnt = floor(double((imW-1)/double(d)));
	int ccol_cnt = floor(double((imH-1)/double(d)));
	int k = ccol_cnt * crow_cnt;
	Mat loc;
	loc.create(1,2,CV_32FC1);
	for(int i=1;i<=ccol_cnt;i++)
	{
		loc.at<float>(0,0) = d+d*(i-1);
		loc.at<float>(0,1) = d;
		if(CC.rows==0)
		{
			CC=loc.clone();
		
		}
		else
		{
			CC.push_back(loc);
		}
	
	
	}
	for(int i =2;i<=crow_cnt;i++)
	{
		for(int j =1;j<=ccol_cnt;j++)
		{
			loc.at<float>(0,0) = d+d*(j-1);
		    loc.at<float>(0,1) = d+(i-1)*d;
		    if(CC.rows==0)
		    {
			  CC=loc.clone();
		
		    }
		    else
		   {
			CC.push_back(loc);
		   }
		
		}	
	
	}

}
void find_lt(Mat image,int num,vector<Point> &points)
{
	for(int col =0;col<image.cols;++col)
	{
		for(int row =0;row<image.rows;++row)
		{
			if(image.at<uchar>(row,col)< num)
			{
				points.push_back(Point(row+1,col+1));
			
			}
		}
	
	}

}
void find_lt_int(Mat image,int num,vector<Point> &points)
{
	for(int col =0;col<image.cols;++col)
	{
		for(int row =0;row<image.rows;++row)
		{
			if(image.at<int>(row,col)< num)
			{
				points.push_back(Point(row+1,col+1));
			
			}
		}
	
	}

}
int find_lt_float(Mat image,int num)
{
  Mat NUM_M = Mat(image.rows,image.cols,image.type(),cv::Scalar::all(num));
  Mat com;
  compare(image,NUM_M,com,CMP_LT);
  Scalar ss = sum(com);
  return ss[0];

}

void find_int(Mat image,int num,vector<Point> &points)
{
	for(int col =0;col<image.cols;++col)
	{
		for(int row =0;row<image.rows;++row)
		{
			if(image.at<int>(row,col) == num)
			{
				points.push_back(Point(row+1,col+1));
			
			}	
		}
	
	}

}

void find(Mat image,int num,Mat &points)
{
	Mat loc;
	loc.create(1,2,CV_32SC1);
	for(int col =0;col<image.cols;++col)
	{
		for(int row =0;row<image.rows;++row)
		{
			if(image.at<uchar>(row,col) == num)
			{
				loc.at<int>(0,0) = row +1;
				loc.at<int>(0,1) = col +1;
				if(points.rows==0 || points.cols==0)
				{
					points = loc.clone();
				
				}
				else
				{
					points.push_back(loc);
				}
			
			}	
		}
	
	}

}

void find_ge_float(Mat image,int num,vector<Point> &points)
{
	for(int col =0;col<image.cols;++col)
	{
		for(int row =0;row<image.rows;++row)
		{
			if(image.at<float>(row,col)>= num)
			{
				points.push_back(Point(row+1,col+1));
			
			}	
		}
	}

}

void find_ge(Mat image,int num,vector<Point> &points)
{
	for(int col =0;col<image.cols;++col)
	{
		for(int row =0;row<image.rows;++row)
		{
			if(image.at<uchar>(row,col)>= num)
			{
				points.push_back(Point(row+1,col+1));
			
			}	
		}
	}

}
void find_ge_int(Mat image,int num,vector<Point> &points)
{
	for(int col =0;col<image.cols;++col)
	{
		for(int row =0;row<image.rows;++row)
		{
			if(image.at<int>(row,col)>= num)
			{
				points.push_back(Point(row+1,col+1));
			
			}	
		}
	}

}
int PartSort(int *arr,int start,int end)
{
	int left = start;
	int right = end;
	int key = arr[end];
	while(left<right)
	{
		while(left<right && arr[left]<=key)
		{
			++left;
		}
		while(left<right && arr[right]>=key)
		{
			--right;
		}
		if(left<right)
		{
			swap(arr[left],arr[right]);
		
		}

	}
	swap(arr[right],arr[end]);
	return left;

}

double GetMidNumNoSort(int *arr,int N)
{
	int i,j,min;
	int t;
	int mid;
	if(N%2==0)
	{
		mid = N/2-1;
	
	}
	else
	{
		mid = (N+1)/2;
	}
	for(i=0;i<=N-1;i++)
	{
		min = i;
		for(j=i+1;j<N;j++)
		{
			if(arr[j]<arr[min])
			{
				min=j;
				t=arr[i];
			    arr[i] = arr[min];
			    arr[min] = t;
			}
		
		}
	
	}

	double mdia ;
	if(N%2==0)
	{
		mdia = (arr[mid]+arr[mid+1])/2.0;
	
	}
	else
	{
		mdia = arr[mid];
	
	}

	return mdia;

}