#include "MRFRelaxWithICM.h"
vector<uchar>  MRFRelaxWithICM(vector<Line> &linelist,vector<uchar> &iLabels, MRF mrf,vector<vector<int>> &Cliques)
{
	mrf.CliqPotMin = -1 *(-mrf.Kl*2 + mrf.Kc*0);

	for(unsigned int i=0;i<linelist.size();i++)
	{
		linelist[i].SegPot[0] =(-0.4587+linelist[i].prob);

		if(mrf.allowjunc==0)
		{
			linelist[i].SegPot[1]=0;
		
		}
		else
		{
			linelist[i].SegPot[1] = (0.5413 - linelist[i].prob);
		
		}
	
	
	}
	vector<uchar> labels  = iLabels;
	int ngrp = 4;

	vector<vector<int>>  G;
	G =  CreateGroupMatrix( linelist, 4);
	vector<vector<uchar>> labels_allp;
	vector<uchar> label_allp1,label_allp2,label_allp3,label_allp4;
	for(int m=1;m<=16;m++)
	{
		vector<uchar> allp_curr;
		allp_curr = de2bi(m,ngrp);
		label_allp1.push_back(allp_curr[0]);
		label_allp2.push_back(allp_curr[1]);
		label_allp3.push_back(allp_curr[2]);
		label_allp4.push_back(allp_curr[3]);
	}
	labels_allp.push_back(label_allp1);
	labels_allp.push_back(label_allp2);
	labels_allp.push_back(label_allp3);
	labels_allp.push_back(label_allp4);

	double E = CalculateEnergyLL( linelist,labels,Cliques, mrf);

	printf("Initial Energy = %f\n",E);
	int num = 0;
	double E_old = 0.e0;
	vector<vector<uchar>> labels_l;
	for(unsigned int i=0;i<labels.size();i++)
	{
		vector<uchar> labels_lc;
		for(int j=0;j<16;j++)
		{
			labels_lc.push_back(labels[i]);
		
		}
		labels_l.push_back(labels_lc);
	
	}
	for(int k=1;k<=mrf.step;k++)
	{
		for(unsigned int i=1;i<=G.size();i++)
		{
			vector<int> G_c = G[i-1];

			for(unsigned int j =0;j<G_c.size();j++)
			{
				labels_l[G_c[j]-1] = labels_allp[j];
			}
			double E_loc[16];
			for(int m =1;m<=16;m++)
			{
				vector<uchar> label_lm;
				for(unsigned int j=0;j<labels.size();j++)
				{
					label_lm.push_back(labels_l[j][m-1]);
				
				}
				E_loc[m-1] = 0.e0;
				for(int n=0;n<4;n++)
				{
					E_loc[m-1] = E_loc[m-1] + MRFLocalEnergy(G_c[n],linelist,label_lm,Cliques,mrf);
				
				}
			}
			int ind = 0;
			for(int m=1;m<16;m++)
			{
				if(E_loc[m]<E_loc[m-1])
				{
					ind = m;
				
				}
			}

			for(unsigned int j=0;j<labels.size();j++)
			{
				labels[j] = labels_l[j][ind];
			
			}
		}

		E = CalculateEnergyLL(linelist,labels,Cliques,mrf);
		if(abs(E-E_old)<0.1)
		{
			num = num+1;
		
		}
		if(num >=10)
		{
		
			break;
		}
		E_old = E;
		printf("_%d,Energy = %f\n",k,E);
	}
	return labels;
}
vector<uchar> de2bi(int m,int ngrp)
{
	vector<uchar> b_l;
	int j = m;
	for(int i=0;i<ngrp;i++)
	{
		b_l.push_back(j%2);
		j = j/2;
	
	}

	return b_l;

}
vector<vector<int>> CreateGroupMatrix(vector<Line> linelist, int nset)
{
	int nconn = linelist.size();

	vector<double> Cent1,Cent2;
	for(unsigned int i =0;i<linelist.size();i++)
	{
		Cent1.push_back(linelist[i].c[0]);
	    Cent2.push_back(linelist[i].c[1]);
	
	}

	vector<vector<int>>  Grp;
	Mat D_index = Mat::zeros(1,nconn,CV_64FC1);
	Mat sortIdxArr;
	int idx;
	for(int i=0;i<nconn;i++)
	{
		vector<int> nodes;
		for(int j=0;j<nconn;j++)
		{
		   D_index.at<double>(0,j)=sqrt(double((Cent1[j]-linelist[i].c[0])*(Cent1[j]-linelist[i].c[0])+(Cent2[j]-linelist[i].c[1])*(Cent2[j]-linelist[i].c[1])));
		}
		
	    sortIdx(D_index, sortIdxArr, CV_SORT_EVERY_ROW  +  CV_SORT_ASCENDING);
		vector<double> D_curr;
		for(int j=0;j<nconn;j++)
		{
			idx = sortIdxArr.at<int>(0,j);
			nodes.push_back(idx+1);
		}
		vector<int> Grp_curr;
		for(int j=0;j<nset;j++)
		{
			Grp_curr.push_back(nodes[j]);
		}
		Grp.push_back(Grp_curr);
	
	}
	return Grp;

}