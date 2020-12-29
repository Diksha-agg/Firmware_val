/*
 * File: time_trajj.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

/* Include Files */
#include "rt_nonfinite.h"
#include <math.h>
/*#include "controller.h"
#include "time_trajj.h"*/

/* Function Definitions */

/*
 * Arguments    : double t
 *                double posd[3]
 *                double veld[3]
 *                double rot_des[3]
 *                double omegad[3]
 *                double controld[2]
 * Return Type  : void
 */
void traj_gen(double f[120], double traj[31], double traj_dot[31])

//global flag;
int n=4;
int N=31;
double Tf = 3.1;
double M[4][4]
double t0;
double tf;
double b[4];
/*
if (flag==4 || flag == 5)
    Tf=3.58;
else if (flag ==2 || flag == 3)
        Tf = 3.1;
    end
end */
//f=zeros(30*n,1);
for (i1=0; i1<N-1; i1++)
{t0 = (i1)*Tf/(N-1);
 tf = (i1+1)*Tf/(N-1);
 for( int i=0;i<4;++i)
 {
	 for(int j=0;j<4;++j)
	 {	if(i==0)
			{M[i][j] = pow(t0,j);}
		else if(i==1)
			{M[i][j] = pow(tf,j);}
		else if (i==2)
			{M[i][j] = j*pow(t0,j-1);}
		else if (i==3)
			{M[i][j] = j*pow(tf,j-1);}
	 }
 }
 b[0]=traj[i];
 b[1]=traj[i+1];
 b[2]=traj_dot[i];
 b[3]=traj_dot[i+1];

}
for i=1:N-1
    t0 = (i-1)*Tf/(N-1);
    tf = i*Tf/(N-1);
    M  = ...
       [1    t0  t0^2    t0^3;
        1    tf  tf^2    tf^3;
        0    1   2*t0    3*t0^2;
        0    1   2*tf    3*tf^2];

    b = [traj(i);traj(i+1);traj_dot(i);traj_dot(i+1)];
    A = M\b;
    f((1:n) + n*(i-1))=A;
end
end
