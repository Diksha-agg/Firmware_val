#include "rt_nonfinite.h"
#include <math.h>
//#include "interpl.h"


double interp1(double x[2], double y[2], double xp)
{
 return y[0] + ((y[1]-y[0])/(x[1]-x[0])) * (xp - x[0]);

}

