#include "SORMNetworkConstruct.h"
#include "Library.h"
void SORMNetworkConstruct(vector<Point> CC,Mat &pmap,float mxDist,float minProb,vector<Line> &linelist,vector<vector <int>> &Cliques)
{
	int ang_th = 16;
	Mat Adj = Mat::zeros(CC.size(),CC.size(),CV_32SC1);
	Mat D = Mat::zeros(1,CC.size(),CV_64FC1);
	int k;
	for(unsigned int i=0;i<CC.size();i++)
	{
		for(unsigned int j=0;j<CC.size();j++)
		{
			if(i!=j)
			{
				D.at<double>(0,j)=sqrt(double((CC[j].x-CC[i].x)*(CC[j].x-CC[i].x)+(CC[j].y-CC[i].y)*(CC[j].y-CC[i].y)));

			}
			else
			{
				D.at<double>(0,j) = INF;
			
			}


		}
		vector<Point> clsPts;
		find_lt_double(D,mxDist,clsPts);
        Mat prob  = Mat::zeros(1,clsPts.size(),CV_64FC1);
		vector<double>Ang ; 
		vector<Point> clsPts_post;
		for(unsigned int j=1;j<=clsPts.size();j++)
		{
			
			prob.at<double>(0,j-1)=GetLineProb(pmap,CC[i],CC[clsPts[j-1].y-1]);
			if(prob.at<double>(0,j-1)>minProb)
			{
			   Ang.push_back(GetAngle(double(CC[i].x-CC[clsPts[j-1].y-1].x),double(CC[i].y-CC[clsPts[j-1].y-1].y)));
			   clsPts_post.push_back(clsPts[j-1]);

			}
		
		}
		if (clsPts_post.size()>0)
		{
		int row_n = clsPts_post.size();
		Mat D_INDEX = Mat::zeros(1,row_n,CV_64FC1);
		for(int index=0;index<row_n;index++ )
		{
			D_INDEX.at<double>(0,index)=D.at<double>(0,clsPts_post[index].y-1);
		
		}
		Mat D_INDEXS;
		sortIdx(D_INDEX,D_INDEXS,CV_SORT_EVERY_ROW  +  CV_SORT_ASCENDING);
		Mat matrix = Mat::zeros(row_n,3,CV_64FC1);
		int in ;
		for(int index = 0;index<row_n;index++)
		{
			in = D_INDEXS.at<int>(0,index);
			matrix.at<double>(index,0)=clsPts_post[in].y;
			matrix.at<double>(index,1)=D_INDEX.at<double>(0,in);
			matrix.at<double>(index,2)=Ang[in];
		
		
		}
		k=1;
		vector<vector<double>> AllocReg;
		vector<double> AllocReg1;
		AllocReg1.push_back(matrix.at<double>(0,2)-ang_th/2.0);
		AllocReg1.push_back(matrix.at<double>(0,2)+ang_th/2.0);
		AllocReg.push_back(AllocReg1);

		vector<int> clsPtsTmp;
		clsPtsTmp.push_back(int(matrix.at<double>(0,0)));

		for(unsigned int m =2;m<=clsPts_post.size();m++)
		{
			double notA =0.e0;

			for(int n =1;n<=k;n++)
			{
				if(matrix.at<double>(m-1,2)>AllocReg[n-1][0] && matrix.at<double>(m-1,2)<AllocReg[n-1][1])
				{
					notA = 1.e0;
				
				}
			
			}
			if(notA == 0.e0)
			{
				k = k+1;
				vector<double> AllocReg_new ;
				AllocReg_new.push_back(matrix.at<double>(m-1,2)-ang_th/2.0);
				AllocReg_new.push_back(matrix.at<double>(m-1,2)+ang_th/2.0);
				AllocReg.push_back(AllocReg_new);
				clsPtsTmp.push_back(int(matrix.at<double>(m-1,0)));
			
			
			}	
		
		}
		
		
		for(unsigned int j =0;j<clsPtsTmp.size();j++)
		{
			Adj.at<int>(i,clsPtsTmp[j]-1) = 1;
			Adj.at<int>(clsPtsTmp[j]-1,i)=1;

		}
	
	
	}

}
  vector<int> X,Y;
  for(int j =1;j<=Adj.cols;j++)
  {
	  for (int i =1;i<=j;i++)
	  {
	  
		  if(Adj.at<int>(i-1,j-1)==1)
		  {
			  X.push_back(i);
			  Y.push_back(j);
		  
		  
		  }
	  }
  
  
  }
  double maxlen = 0.e0;
  for(unsigned int i =1;i<=X.size();i++)
  {
	  Line line_c;
	  line_c.s= CC[X[i-1]-1];
	  line_c.e = CC[Y[i-1]-1];
	  line_c.c[0] = (line_c.s.x+line_c.e.x)/2.0;
	  line_c.c[1] = (line_c.s.y+line_c.e.y)/2.0;
	  line_c.len = sqrt(double((line_c.s.x-line_c.e.x)*(line_c.s.x-line_c.e.x)+(line_c.s.y-line_c.e.y)*(line_c.s.y-line_c.e.y)));
	  line_c.ang = GetAngle(double(line_c.e.x-line_c.s.x),double(line_c.e.y-line_c.s.y));
	  line_c.adj.x = X[i-1];
	  line_c.adj.y = Y[i-1];
	  line_c.prob =  GetLineProb(pmap,line_c.s,line_c.e);

	  line_c.nr = i;
	  linelist.push_back(line_c);
	  if(line_c.len>maxlen)
	  {
		  maxlen = line_c.len;
	  
	  }
  
  
  }
  for(unsigned int i =1;i<=X.size();i++)
  {
	  linelist[i-1].len = linelist[i-1].len/maxlen;
  }
  FindConnCliques(linelist,Adj,Cliques);



}
void FindConnCliques(vector<Line> &linelist,Mat &Adj,vector<vector<int>> &Clqs)
{
	for(int i=0;i<Adj.rows;i++)
	{
		for(int j =0;j<=i;j++)
		{
			Adj.at<int>(i,j)=0;
		
		
		
		}

	}
	vector<int> tmp;
	vector<int> AdjV1,AdjV2;
	int k = 0;

	for(unsigned int i =0;i<linelist.size();i++)
	{
		linelist[i].ClqS=0;
		linelist[i].ClqE=0;	
		tmp.push_back(linelist[i].adj.x);
		tmp.push_back(linelist[i].adj.y);
		AdjV1.push_back(linelist[i].adj.x);
		AdjV2.push_back(linelist[i].adj.y);

	}

	for(unsigned int i=0;i<linelist.size();i++)
	{
		vector<vector<int>> connS,connE;
		vector<int> connS1,connS2,connE1,connE2;
		for(unsigned int j =0;j<AdjV1.size();j++)
		{
			if(AdjV1[j] == AdjV1[i] && j!=i)
			{
				connS1.push_back(j+1);

			}
			if(AdjV2[j] == AdjV2[i] && j!=i)
			{
				connS2.push_back(j+1);
			
			
			}
			if(AdjV1[j] == AdjV2[i] && j!=i)
			{
				connE1.push_back(j+1);
			
			}
			if(AdjV2[j] == AdjV1[i] && j!=i)
			{
				connE2.push_back(j+1);
			}
		}
		connS.push_back(connS1);
		connS.push_back(connS2);
		connE.push_back(connE1);
		connE.push_back(connE2);

		if(linelist[i].ClqS == 0)
		{
			k = k+1;

			vector<int>  ClqS_curr ;
			ClqS_curr.push_back(i+1);
			
			if(connS[0].size()>=1 || connS[1].size()>=1)
			{
				for(unsigned int j =0;j<connS[0].size();j++)
				{
					if(linelist[i].s.x == linelist[connS[0][j]-1].s.x  && linelist[i].s.y == linelist[connS[0][j]-1].s.y )
					{
						if(linelist[connS[0][j]-1].ClqS == 0)
						{
							linelist[connS[0][j]-1].ClqS =1;
							linelist[connS[0][j]-1].clq[0]=k;
							ClqS_curr.push_back(connS[0][j]);
						
						}
					
					
					}
				
				
				}
				for(unsigned int j =0;j<connS[1].size();j++)
				{
					if(linelist[i].s.x == linelist[connS[1][j]-1].e.x  && linelist[i].s.y == linelist[connS[1][j]-1].e.y )
					{
						if(linelist[connS[1][j]-1].ClqE == 0)
						{
							linelist[connS[1][j]-1].ClqE =1;
							linelist[connS[1][j]-1].clq[1]=k;
							ClqS_curr.push_back(connS[1][j]);

						
						}
					
					
					}
				
				
				}

			}
			linelist[i].ClqS =1;
			linelist[i].clq[0] = k;
			Clqs.push_back(ClqS_curr);
		}
	
		if(linelist[i].ClqE == 0)
		{
			 k=k+1;
			 vector<int> Clqs_curr;
			 Clqs_curr.push_back(i+1);
			 if(connE[0].size()>=1 || connE[1].size()>=1)
			 {
				 for(unsigned int j =0;j<connE[0].size();j++)
				 {
					if(linelist[i].e.x == linelist[connE[0][j]-1].s.x && linelist[i].e.y == linelist[connE[0][j]-1].s.y) 	
					{
						if(linelist[connE[0][j]-1].ClqS==0)
						{
						  linelist[connE[0][j]-1].ClqS = 1;
						  linelist[connE[0][j]-1].clq[0] =k;
						  Clqs_curr.push_back(connE[0][j]);
						}
						
					}
				 
				 
				 }
				 for(unsigned int j =0;j<connE[1].size();j++)
				 {
					 if(linelist[i].e.x==linelist[connE[1][j]-1].e.x && linelist[i].e.y==linelist[connE[1][j]-1].e.y)
					 {
						 if(linelist[connE[1][j]-1].ClqE==0)
						 {
							 linelist[connE[1][j]-1].ClqE =1;
							 linelist[connE[1][j]-1].clq[1] = k;
							 Clqs_curr.push_back(connE[1][j]);
						 
						 }
					 
					 }
				 
				 }
			 
			 }
			 linelist[i].ClqE = 1;
			 linelist[i].clq[1]=k;
			 Clqs.push_back(Clqs_curr);
		
		}
	
	}


}

void find_lt_double(Mat image,double num,vector<Point> &points)
{
	for(int col =0;col<image.cols;++col)
	{
		for(int row =0;row<image.rows;++row)
		{
			if(image.at<double>(row,col)< num)
			{
				points.push_back(Point(row+1,col+1));
			
			}
		}
	
	}

}