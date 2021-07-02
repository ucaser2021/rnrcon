#include "Library.h"

double CalculateEnergyLL( vector<Line> &linelist,vector<uchar> &labels,vector<vector<int>> &clique, MRF mrf)
{
	double Vtot = mrf.eratio * SegmentPotentialsLL(linelist,labels, mrf);
	vector<double> Vc = CliquePotentialsLL(linelist,labels,clique,mrf);
	double Vc_sum = 0.e0;
	for(unsigned int i=0;i<Vc.size();i++)
	{
	   Vc_sum = Vc_sum + Vc[i];
	}

	double E = Vtot + Vc_sum;
	return E;


}