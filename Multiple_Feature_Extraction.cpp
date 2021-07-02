#include "Road_Centerline_Extraction.h"
double round(double r)
{
	return(r>0.0)?floor(r+0.5):ceil(r-0.5);
}
Mat  rotateImage1(Mat img, double degree)
{
	double angle = degree  * CV_PI / 180.; // 弧度  
	double a = sin(angle), b = cos(angle);
	int width = img.cols;
	int height = img.rows;
	int width_rotate = ceil(height * fabs(a) + width * fabs(b));
	int height_rotate = ceil(width * fabs(a) + height * fabs(b));

	// 旋转中心;
	Point center = Point(width/ 2, height/2);
	Mat map_matrix = getRotationMatrix2D(center, double(degree), 1.e0);

	map_matrix.at<double>(0, 2) += (width_rotate - width) / 2;
	map_matrix.at<double>(1, 2) += (height_rotate - height) / 2;

	Mat img_rotate = Mat::zeros(height_rotate,width_rotate,img.type());
	// INTER_NEARST,INTER_LINEAR ,CV_INTER_CUBIC
	warpAffine(img, img_rotate, map_matrix, img_rotate.size() , INTER_NEAREST | CV_WARP_FILL_OUTLIERS, BORDER_CONSTANT, Scalar());
	return img_rotate;
}
Mat angleRotate(Mat& img,int angle)
{
	float rads = float(angle * CV_PI / 180.0);
	int rowsi = img.rows;
	int colsi = img.cols;

	int rowsf = ceil(rowsi * abs(cos(rads)) + colsi * abs(sin(rads)));
	int colsf = ceil(rowsi * abs(sin(rads)) + colsi * abs(cos(rads)));

	Mat C = Mat(rowsf,colsf,img.type(),cv::Scalar::all(0.0));
	int xo = ceil(rowsi / 2.0);
	int yo = ceil(colsi / 2.0);

	int midx = ceil(rowsf / 2.0);
	int midy = ceil(colsf / 2.0);

	int x,y;
	int i,j;
	for(i=1;i<=rowsf;i++)
	{
		for(j=1;j<=colsf;j++)
		{
			x = (i-midx) * cos(rads) + (j-midy) * sin(rads);
			y = -(i-midx) * sin(rads) + (j-midy) * cos(rads);
			x = round(x) + xo;
			y = round(y) + yo;
			if(x >=1 && y>=1 && x<=rowsi && y<=colsi)
			{
				C.at<float>(i-1,j-1) = img.at<float>(x-1,y-1);
			}

		
		}
	
	}
	return C;

}
vector<Mat>  Multiple_Feature_Extraction(Mat &img1 ,int* R)
{
	int dm =img1.rows;
	int dn = img1.cols;
	Mat img;
	img1.convertTo(img,CV_32FC1,1.0);
	vector<Mat> channels;

	Mat r_curr;
	Mat theta_curr;
	Mat c_curr;

	Mat com1,com2;
	int k,n;
	double theta = 0.e0;
	int s;
	Scalar ss;
	Mat h_rotate,r_new,theta_new;
	Point anchor_center(-1,-1);
	Mat h,r_sum;
	for(k=0;k<3;k++)
	{
		r_sum = Mat::zeros(dm,dn,img.type());
		s = R[k];
		h = Mat::ones(1,s,CV_32FC1);
		for(n=0;n<=17;n++)
		{
			theta = n*10.0;			
			h_rotate = rotateImage1(h,theta);
			ss = sum(h_rotate);
			h_rotate = h_rotate / ss[0];
			filter2D(img,r_new,img.depth(),h_rotate,anchor_center,0,BORDER_REPLICATE);
			r_sum += r_new;
			theta_new = Mat(dm,dn,img.type(),cv::Scalar::all(theta));
			if(n == 0)
			{
				r_curr = r_new.clone();
				theta_curr = theta_new.clone();
			
			}
			else
			{
				compare(r_new,r_curr,com1,CMP_LT);
				compare(r_new,r_curr,com2,CMP_GE);

				com1.convertTo(com1,CV_32FC1,1.0/255.0);
				com2.convertTo(com2,CV_32FC1,1.0/255.0);
				r_curr = r_curr.mul(com2);
				r_curr += r_new.mul(com1);
				theta_curr = theta_curr.mul(com2);
				theta_curr += theta_new.mul(com1);

			
			}

		
		}
		channels.push_back(r_curr);
		divide(r_curr,r_sum/18.0,c_curr);
		channels.push_back(c_curr-1);
		channels.push_back(theta_curr);
		h.release();
	
	
	}
	return channels;

}