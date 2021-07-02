#include "Library.h"

double GetAngle(double x,double y)
{
	double ang = 0.e0;
   if(x<0 && y<0)
   {
	   ang = 180 + atan(abs(y)/abs(x))/PI*180;
   
   }
   if(x<0 && y>=0)
   {
	   ang = 180 - atan(y/abs(x))/PI*180;
   
   }
   if(x>=0 && y<=0)
   {
   
	   ang = 360 -atan(abs(y)/x)/PI*180;

   }
   if(x>=0 && y>=0)
   {
	   ang = atan(y/x)/PI*180; 
   
   }
   return ang;

}