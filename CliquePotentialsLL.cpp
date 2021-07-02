#include"Library.h"

vector<double> CliquePotentialsLL(vector<Line> &linelist,vector<uchar> &labels,vector<vector<int>> &Cliques,MRF mrf)
{
	vector<double> V;
	double v_curr;
	for(unsigned int i=0;i<Cliques.size();i++)
	{

		vector<int> Clique = Cliques[i];
		v_curr = SormCliquePotential(linelist,labels,Clique, mrf);
		V.push_back(v_curr);
	
	}
	return V;


}