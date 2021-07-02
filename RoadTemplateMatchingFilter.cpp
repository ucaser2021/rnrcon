#include "RoadTemplateMatchingFilter.h"
#include "Library.h"
Mat angelRotate(Mat &img, int degree)
{
	Mat img_rotate;

	Point2f center = Point2f(img.cols / 2, img.rows / 2);
	Mat M = getRotationMatrix2D(center,double(degree), 1.e0);
	// CV_INTER_NN,CV_INTER_LINEAR ,CV_INTER_CUBIC
	warpAffine(img, img_rotate, M, img.size(), INTER_NEAREST | CV_WARP_FILL_OUTLIERS, BORDER_CONSTANT, Scalar());
	return img_rotate;
}
Mat mat2gray(Mat &img)
{
  double minv;
  double maxv;
  minMaxLoc(img,&minv,&maxv,NULL,NULL);

  Mat img_g = Mat::zeros(img.rows,img.cols,img.type());
  for(int i=0;i<img.rows;i++)
  {
	  for(int j=0;j<img.cols;j++)
	  {
		  img_g.at<float>(i,j) = (img.at<float>(i,j) - minv)/(maxv-minv);
	  
	  }
  
  
  }
  return img_g;
	



}


Mat RoadTemplateMatchingFilter(Mat &I,int N,int angle_step,int* road_range)
{
	int ws = 3 * road_range[1];
	vector<Mat> fb = gen_filter_bank_int2(ws, road_range[0], road_range[1]);
	int rot_step_cnt = 180 /angle_step + 1;

	int wsn = (ws+1)/2;

	vector<vector<Mat>> kernel;
	vector<vector<Mat>> kernel_left;
	vector<vector<Mat>> kernel_right;
	Mat in_result;



	for(unsigned int m =0;m<fb.size();m++)
	{
		vector<Mat> kernel_v;
		vector<Mat> kernel_left_v;
		vector<Mat> kernel_right_v;
		for(int i =0;i<rot_step_cnt;i++)
		{
			Mat kernel_m;
			Mat kernel_left_m = Mat::zeros(ws,ws,CV_32FC1);
			Mat kernel_right_m = Mat::zeros(ws,ws,CV_32FC1);

			kernel_m = angelRotate(fb[m], i*angle_step);
			kernel_v.push_back(kernel_m);

		    for(int r=0;r<ws;r++)
			{
				for(int c=0;c<wsn;c++)
				{
					kernel_left_m.at<float>(r,c)= fb[m].at<float>(r,c);
				
				}
				
			
			}
			Mat kernel_left_imed = angelRotate(kernel_left_m, i*angle_step);
			Mat kernel_leftb = filter_balance(kernel_left_imed);
			kernel_left_v.push_back(kernel_leftb);
			

			for(int r=0;r<ws;r++)
			{
				for(int c=wsn-1;c<ws;c++)
				{
					kernel_right_m.at<float>(r,c)=fb[m].at<float>(r,c);
				
				}
			
			}
			
			Mat kernel_right_imed =angelRotate(kernel_right_m, i*angle_step);
			Mat kernel_rightb = filter_balance(kernel_right_imed);
			kernel_right_v.push_back(kernel_rightb);

		
		}

		kernel_left.push_back(kernel_left_v);
		kernel.push_back(kernel_v);
		kernel_right.push_back(kernel_right_v);
	
	}	

	Mat result;
	I.convertTo(result,CV_32FC1,1.0/255.0,0);

	for(int k =1;k<=N;k++)
	{
		printf("iter:%d\n",k);
		Mat imfilt1(result.rows,result.cols,CV_32FC1,Scalar(-INF));
		Mat imfilt2(result.rows,result.cols,CV_32FC1,Scalar(-INF));
		int kk;
		if(4-k>=1)
		{
			kk=4-k;
		
		
		}
		else
		{
		   kk=1;
		
		}
		for(unsigned int m=1;m<=fb.size()+kk-3;m++)
		{

			for(int i=1;i<=rot_step_cnt;i++)
			{
				Point anchor_center(-1, -1);
				Mat res1 = Mat::zeros(result.rows,result.cols,result.type());
				Mat res2 = Mat::zeros(result.rows,result.cols,result.type());
				Mat res = Mat::zeros(result.rows,result.cols,result.type());
				filter2D(result,res1, -1,kernel_left[m-1][i-1], anchor_center, 0, BORDER_CONSTANT);
				filter2D(result, res2, -1, kernel_right[m-1][i-1],anchor_center, 0, BORDER_CONSTANT);
				filter2D(result, res, -1, kernel[m-1][i-1],anchor_center, 0, BORDER_CONSTANT);

				Mat com1,com2;
				Mat com1d,com2d;
				compare(res1,res2,com1,CMP_LE);
				com1.convertTo(com1d,CV_32FC1,1.0/255.0);
				compare(res2,res1,com2,CMP_LT);
				com2.convertTo(com2d,CV_32FC1,1.0/255.0);

				Mat res12;
				Mat res11,res22;
				multiply(res1,com1d,res11);
				multiply(res2,com2d,res22);

				Mat res12min;

				add(res11,res22,res12min);

				compare(res12min,res,com1,CMP_GE);
				com1.convertTo(com1d,CV_32FC1,1.0/255.0);
				compare(res,res12min,com2,CMP_GT);
				com2.convertTo(com2d,CV_32FC1,1.0/255.0);
				
				multiply(res12min,com1d,res11);
				multiply(res,com2d,res22);

				add(res11,res22,imfilt1);

				compare(imfilt1,imfilt2,com1,CMP_GE);
				com1.convertTo(com1d,CV_32FC1,1.0/255.0);
				compare(imfilt2,imfilt1,com2,CMP_GT);
				com2.convertTo(com2d,CV_32FC1,1.0/255.0);
				
				multiply(imfilt1,com1d,res11);
				multiply(imfilt2,com2d,res22);

				
				add(res11,res22,imfilt2);
				in_result = imfilt2.clone();
				imfilt2 = in_result.clone();


				
			}

	
		}
		result = in_result.clone();
		
	
	}

	Mat result_g = mat2gray(result);
	
	return result_g;

}

Mat filter_balance(Mat &filt)
{
	Mat zeros_mat = Mat::zeros(filt.rows,filt.cols,filt.type());
	Mat filt_g = Mat::zeros(filt.rows,filt.cols,filt.type());
	Mat filt_gg = Mat::zeros(filt.rows,filt.cols,filt.type());;
	Mat filt_l = Mat::zeros(filt.rows,filt.cols,filt.type());
	Mat filt_ll = Mat::zeros(filt.rows,filt.cols,filt.type());;
	Mat com_g = Mat::zeros(filt.rows,filt.cols,filt.type());
	Mat com_dg = Mat::zeros(filt.rows,filt.cols,filt.type());
	Mat com_l = Mat::zeros(filt.rows,filt.cols,filt.type());
	Mat com_dl = Mat::zeros(filt.rows,filt.cols,filt.type());

	compare(filt,zeros_mat,com_g,CMP_GT);
	com_g.convertTo(com_dg,filt.type(),1.0/255.0,0);
	compare(filt,zeros_mat,com_l,CMP_LT);
	com_l.convertTo(com_dl,filt.type(),1.0/255.0,0);


	multiply(com_dg,filt,filt_g);
	multiply(com_dl,filt,filt_l);
	Mat filt_output = Mat::zeros(filt.rows,filt.cols,filt.type());
	Scalar g_s = sum(filt_g);
	Scalar l_s = sum(filt_l);

	//multiply(filt,filt_g,filt_gg);
	//multiply(filt,filt_l,filt_ll);

	add(1/g_s[0]*filt_g , abs(1/l_s[0])*filt_l,filt_output);

	return filt_output;


}

Mat guss(int size,float sigma)
{
	float w = 2.0*sigma*sigma;
	Mat K = cvCreateMat(size,size,CV_32FC1);
	int m = size / 2;
	int n = size / 2;
	for(int i=(-m);i<=m;i++)
	{
		int row = i + m;
		for(int j=(-n);j<=n;j++)
		{
			int col = j + n;
			float v = exp(-(i*i+j*j)/w);
			K.at<float>(row,col) = v;

		}
	
	}
	CvScalar p;
	p = sum(K);
	Mat K_n;
	K.convertTo(K_n,-1,1/p.val[0],0);
	return K_n;

}
vector<Mat> gen_filter_bank_int2(int ws ,int min_rw,int max_rw)
{
	vector<Mat> fbs;
	Mat gf = guss(ws,float(ws)/3);
	double x_n[5] = {0.e0,1.e0,2.e0,3.e0,4.e0};
	double y_n[5] = {-1*sin(PI*x_n[0]/4),-1*sin(PI*x_n[1]/4),-1*sin(PI*x_n[2]/4),-1*sin(PI*x_n[3]/4),-1*sin(PI*x_n[4]/4)};
	double y_nsum = abs(y_n[0])+abs(y_n[1])+abs(y_n[2])+abs(y_n[3])+abs(y_n[4]);
	for(int i =0;i<=4;i++)
	{
		y_n[i] = y_n[i] /(2.0*y_nsum);
	}

	for(int i=min_rw;i<=max_rw;i=i+2)
	{
		Mat x = Mat::zeros(1,ws,CV_32FC1);
		Mat y = Mat::zeros(1,ws,CV_32FC1);
		int ws2 = (ws + 1) / 2;
		int k =0;
		for(int j =(ws2-(i-1)/2)-1;j<=ws2+(i-1)/2-1;j++)
		{
			x.at<float>(0,j) = i-1 + k;
			k = k+1;
		
		}
		double y_sum=0.e0;
		for(int j=0;j<ws;j++)
		{
			y.at<float>(0,j) = (-1*sin(PI*x.at<float>(0,j)/(i-1)));
			if (y.at<float>(0,j)>0)
			{
				y_sum = y_sum + y.at<float>(0,j);
			
			}
		
		}
		for(int j=0;j<ws;j++)
		{
			y.at<float>(0,j) = y.at<float>(0,j)/y_sum;

		
		}
		k = 0;
		for(int j=(ws2-(i+7)/2)-1;j<=(ws2-(i-1)/2)-1;j++)
		{
			y.at<float>(0,j)=y_n[k];
			k=k+1;
		
		}
		k = 0;
		for(int j=(ws2+(i-1)/2)-1;j<=(ws2+(i+7)/2)-1;j++)
		{
			y.at<float>(0,j)=y_n[k];
			k=k+1;
		
		}
		Mat fb;
		Mat yr = repeat(y,ws,1);
		Mat yr_n;
		yr.convertTo(yr_n,gf.type(),1.0,0);
		multiply(yr_n,gf,fb);
		fbs.push_back(fb);
	}
	
	return fbs;








}
