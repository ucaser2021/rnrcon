#include"Library.h"

double MRFLocalEnergy(int node,vector<Line> &linelist,vector<uchar> &labels,vector<vector<int>> &Cliques,MRF mrf )
{
	double V;
	if(labels[node-1]==0)
	{
		V = linelist[node-1].SegPot[0];
	}
	else
	{
		V = linelist[node-1].SegPot[1];
	}

	double Vc;
	Vc = 0.5*(SormCliquePotential(linelist,labels,Cliques[linelist[node-1].clq[0]-1],mrf)+
		SormCliquePotential(linelist,labels,Cliques[linelist[node-1].clq[1]-1],mrf));

	double E = Vc + mrf.eratio * V;
	return E;



}