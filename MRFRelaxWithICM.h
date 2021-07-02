#ifndef _MRFRELAXWITHICM_H
#define _MRFRELAXWITHICM_H
#include "Road_Centerline_Extraction.h"
vector<vector<int>> CreateGroupMatrix(vector<Line> linelist, int nset);
vector<uchar> de2bi(int m,int ngrp);
double CalculateEnergyLL( vector<Line> &linelist,vector<uchar> &labels,vector<vector<int>> &clique, MRF mrf);
double MRFLocalEnergy(int node,vector<Line> &linelist,vector<uchar> &labels,vector<vector<int>> &Cliques,MRF mrf );
#endif