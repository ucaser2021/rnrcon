#include "Road_Centerline_Extraction.h"

int main()
{
	
	//Mat I = imread("D:\\road_extraction1\\sar.jpg",0);

	////Multi-scale feature extarction and classification
	//int R[3] = {9,21,35};
	//vector<Mat> MS_Feat = Multiple_Feature_Extraction(I ,R);
	//Mat output =  Feature_Binarization(MS_Feat,I.rows,I.cols);
	//Mat output1 = Area_Selection(output,15,40,180);
 //   Mat structureElement = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	//Mat output2;
	//morphologyEx(output1,output2, MORPH_CLOSE,structureElement );
	//output = output2.clone();
	//imwrite("E:\\output.tif",output);
	////// Refinement
	//int road_range[2]={3,13};
	//Mat clres_filt =  RoadTemplateMatchingFilter(output,3,10, road_range);
	//double filt_th = 0.05;
	//Mat clres_filt_th = Mat::zeros(clres_filt.rows,clres_filt.cols,CV_8UC1);
	//for(int i=0;i<clres_filt.rows;i++)
	//{
	//	for(int j=0;j<clres_filt.cols;j++)
	//	{
	//		//printf("%f\n",clres_filt.at<float>(i,j));
	//		if(clres_filt.at<float>(i,j)>filt_th)
	//		{
	//			clres_filt_th.at<uchar>(i,j) = 255;
	//		
	//		}
	//		//("%f\t %d\n",clres_filt.at<float>(i,j),clres_filt_th.at<uchar>(i,j));
	//	
	//	}
	//
	//}

	//imwrite("E:\\clres_filt_th.tif",clres_filt_th);

	////skeleton extraction
	//Mat img_skel = skeletonization(clres_filt_th);

	//imwrite("E:\\img_skel.tif",img_skel);
	////prune branches with lenght less than or equal to 15
	//vector<Point2i> bp =  find_branchpoints(img_skel);
	//Mat img_bp = Mat::zeros(img_skel.rows,img_skel.cols,img_skel.type());
	//for(unsigned int i=0;i<bp.size();i++)
	//{
	//	img_bp.at<uchar>(bp[i].y,bp[i].x) = 255;
	//
	//}
	//imwrite("E:\\img_bp.tif",img_bp);
	//structureElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1));
	//Mat img_bpd;
	//dilate(img_bp,img_bpd,structureElement);
	//for(int i=0;i<img_bpd.rows;i++)
	//{
	//	for(int j=0;j<img_bpd.cols;j++)
	//	{
	//		if(img_bpd.at<uchar>(i,j)==255)
	//		{
	//		  img_skel.at<uchar>(i,j)=0;
	//		}
	//	
	//	}
	//
	//
	//}
	//imwrite("E:\\img_skel1.tif",img_skel);


 //   Mat img_skel_n = cc_threshold(img_skel,15,0);
	//structureElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1));
	//Mat clres_filt_th_ref;
	//dilate(img_skel_n,clres_filt_th_ref,structureElement);
	//imwrite("E:\\clres_filt_th_ref.tif",clres_filt_th_ref);
	Mat clres_filt_th_ref = imread("E:\\clres_filt_th_ref.tif",0);
	Mat output = imread("E:\\output.tif",0);
	vector<Point> CC = sorm(clres_filt_th_ref,25,12,10);
	vector<Line> linelist;
	vector<vector<int>> Cliques;
	SORMNetworkConstruct(CC,output,70.0,0.3,linelist,Cliques);

	vector<uchar> iLabels ;
	for(unsigned int i =0;i<linelist.size();i++)
	{
		if(linelist[i].prob>0.4)
		{
			iLabels.push_back(1);
		
		
		}
		else
		{
			iLabels.push_back(0);
		
		}
	
	
	}
	MRF mrf;
	mrf.gnLevel =7;
	mrf.step = 50;
	mrf.c=0.5;
	mrf.Ti=2;
	mrf.sorm=1;
	mrf.Ke=0.3;
	mrf.Kl=0.01;
	mrf.Kc=0.3;
	mrf.Ki=0.4;
	mrf.Vo=0.7;
	mrf.eratio = 1.0/25.0;
	mrf.allowjunc = 1;

	vector<uchar> opt_labels = MRFRelaxWithICM(linelist,iLabels, mrf,Cliques);


}