#include "Library.h"
int round_double(double number)
{
	int num_int;
	if(number>0.0)
	{
		num_int = floor(num_int+0.5);
	   
	}
	else
	{
	    num_int = ceil(num_int-0.5);
	}
	return num_int;
}