#include "Library.h"
double GetDeflAngle(Line line1,Line line2)
{
	double defl = 0.e0;
	if((line1.s.x==line2.s.x && line1.s.y ==line2.s.y)||(line1.e.x ==line2.e.x && line1.e.y==line2.e.y))
	{
		defl = abs(line1.ang - line2.ang);
	
	}
	else
	{
		if((line1.s.x==line2.e.x && line1.s.y==line2.e.y)||(line1.e.x==line2.s.x && line1.e.y==line2.s.y))
		{
			defl = abs(180-(line1.ang-line2.ang));
		}
	
	}
	if(defl >360)
	{
		defl = defl - 360;
	
	}
	if(defl>180)
	{
	   defl = 360 - defl;
	}

	return defl;
}
double SormCliquePotential(vector<Line> &linelist,vector<uchar> &labels,vector<int> &Clique,MRF mrf)
{
	vector<int> plist;
	for(unsigned int i=0;i<Clique.size();i++)
	{
		if(labels[Clique[i]-1]==1)
		{
			plist.push_back(Clique[i]);
		
		}
	}
	double V = 0.e0;
	if(plist.size()==0)
	{
		V = mrf.Vo;
	}
	if(plist.size()==1)
	{
		V = mrf.Ke - mrf.Kl * linelist[plist[0]-1].len;
	
	}
	double defl;
	if(plist.size()==2)
	{
		  defl = GetDeflAngle(linelist[plist[0]-1],linelist[plist[1]-1]);
		  if(defl>90)
		  {
			  V= -mrf.Kl*(linelist[plist[0]-1].len + linelist[plist[1]-1].len)+mrf.Kc*sin(defl/180.0*PI);
		  }
		  else
		  {
			  V = mrf.Ki *2;
		  }
	
	}
	double defl1,defl2,defl3;
	//junctions
	if(plist.size()==3)
	{
		V = mrf.Ki*3;
		if(mrf.allowjunc == 1)
		{
			defl1 = GetDeflAngle(linelist[plist[0]-1],linelist[plist[1]-1]);
			defl2 = GetDeflAngle(linelist[plist[0]-1],linelist[plist[2]-1]);
			defl3 = GetDeflAngle(linelist[plist[1]-1],linelist[plist[2]-1]);
			Line line_b1,line_b2;
			if(defl3<=defl1 && defl3<=defl2)
			{
				line_b1 = linelist[plist[1]-1];
				line_b2 = linelist[plist[2]-1];
			}
			if(defl2<=defl1 && defl2<defl3)
			{
				line_b1 = linelist[plist[0]-1];
				line_b2 = linelist[plist[2]-1];
			
			}
			if(defl1<defl2 && defl1<defl3)
			{

			    line_b1 = linelist[plist[0]-1];
				line_b2 = linelist[plist[1]-1];
			
			}
			int Clq;
			if((line_b1.clq[0]==line_b2.clq[0])||(line_b1.clq[0]==line_b2.clq[1]))
			{
				Clq = line_b1.clq[0];
			
			}
			else
			{
				if((line_b1.clq[1]==line_b2.clq[0])||(line_b1.clq[1]==line_b2.clq[1]))
				{
				
				   Clq = line_b1.clq[1];
				}
			
			}

			vector<int> Clq_b1,Clq_b2;
			for(int i=0;i<2;i++)
			{
				if(line_b1.clq[i] !=Clq)
				{
					Clq_b1.push_back(line_b1.clq[i]);
				
				}
				if(line_b2.clq[i]!=Clq)
				{
					Clq_b2.push_back(line_b2.clq[i]);
				
				}
			
			}
			vector<int> conns_b1,conns_b2;
			for(unsigned int i=0;i<linelist.size();i++)
			{
				if(linelist[i].clq[0]==Clq_b1[0] && linelist[i].clq[0]!=line_b1.nr)
				{
					conns_b1.push_back(int(ceil(linelist[i].clq[0]/2.0)));

				}
				if(linelist[i].clq[1]==Clq_b1[0] && linelist[i].clq[1]!=line_b1.nr)
				{
					conns_b1.push_back(int(ceil(linelist[i].clq[1]/2.0)));
				
				}
				if(linelist[i].clq[0]==Clq_b2[0] && linelist[i].clq[0]!=line_b2.nr)
				{
					conns_b2.push_back(int(ceil(linelist[i].clq[0]/2.0)));
				}
				if(linelist[i].clq[1]==Clq_b2[0] && linelist[i].clq[0]!=line_b2.nr)
				{
					conns_b2.push_back(int(ceil(linelist[i].clq[1]/2.0)));
				}	
			
			}
			vector<int> ind_b1,ind_b2;
			for(unsigned int i=0;i<conns_b1.size();i++)
			{
				if(labels[conns_b1[i]-1]==1)
				{
					ind_b1.push_back(i+1);
				
				}
			
			}
			for(unsigned int i=0;i<conns_b2.size();i++)
			{
				if(labels[conns_b2[i]-1]==1)
				{
					ind_b2.push_back(i+1);
				
				}
			
			}
			double D;
			if(ind_b1.size()==1 && ind_b2.size()==1)
			{
				if((GetDeflAngle(line_b1,linelist[conns_b1[ind_b1[0]-1]-1])>=120)&&(GetDeflAngle(line_b2,linelist[conns_b2[ind_b2[0]-1]-1])>=120))
				{
					D=0.0;
				
				}
				else
				{
					D = INF;
				}

				double defln[3],sindfl[3];
				if(defl1<=defl2 && defl2<=defl3)
				{
					defln[0] = defl1;
					if(defl2<=defl3)
					{
						defln[1]=defl2;
						defln[2]=defl3;
					
					}
					else
					{
						defln[1]=defl3;
						defln[2]=defl2;
					
					}
				
				}
				if(defl2<defl1 && defl2 <=defl3)
				{
					defln[0] = defl2;
					if(defl1<=defl3)
					{
						defln[1]=defl1;
						defln[2]=defl3;
					}
					else
					{
						defln[1] = defl3;
						defln[2] = defl1;
					}
				}
				if(defl3<defl1 && defl3<defl2)
				{
					defln[0] = defl3;
					if(defl1<=defl2)
					{
						defln[1]=defl1;
						defln[2]=defl2;
					}
					else
					{
						defln[1]=defl2;
						defln[2]=defl1;
					
					}
				
				}
				for(int i=0;i<3;i++)
				{
					sindfl[i] = sin(defln[i]/180.0*PI);
				
				}
				double len_sum = 0.e0;
				for(unsigned int i =0;i<plist.size();i++)
				{
					len_sum = len_sum + linelist[plist[i]-1].len;
				
				}
				if(defln[1]>90 && D <4)
				{

					V = -mrf.Kl * len_sum + mrf.Kc*sindfl[0]+(1-(sindfl[1]+sindfl[2])/2.0);
				
				}
			
			}

		}
		else
		{
			V = mrf.Ki*plist.size();

         }
	
	
	
	}

	V = V+mrf.CliqPotMin;

	return V;


}