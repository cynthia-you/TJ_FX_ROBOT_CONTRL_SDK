#include "O3Polynorm.h"
#include "math.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CO3Polynorm::CO3Polynorm()
{

}

CO3Polynorm::~CO3Polynorm()
{

}

double CO3Polynorm::CalPnY(double PnPara[4],double x)
{
	return ( (PnPara[0] * x + PnPara[1] ) * x + PnPara[2] ) * x + PnPara[3];
}

double CO3Polynorm::CalPnFD(double PnPara[4],double x)
{
	return ( 3.0 * PnPara[0] * x + 2.0 * PnPara[1] ) * x + PnPara[2] ;
}

double CO3Polynorm::CalPnSD(double PnPara[4],double x)
{
	return  6.0 * PnPara[0] * x +  2.0 * PnPara[1] ;
}

bool CO3Polynorm::CalXPara(double x[4],double ret_XPara[10])
{
	long i;
	double A[3][3];
	for(i = 0; i < 3; i ++)
	{
		double dx =  x[i+1]-x[0];
		A[i][2] = dx;
		A[i][1] = dx * dx;
		A[i][0] = dx * A[i][1];
	}
	double d =  A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
				A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
				A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]) ;
	if (fabs(d) < 0.000000000001)
	{
		return false;
	}

	double k = 1.0 / d;
	ret_XPara[0] = k * (A[1][1]*A[2][2] - A[2][1] * A[1][2]);
	ret_XPara[1] = k * (A[2][1]*A[0][2] - A[0][1] * A[2][2]);
	ret_XPara[2] = k * (A[0][1]*A[1][2] - A[1][1] * A[0][2]);
	ret_XPara[3] = k * (A[2][0]*A[1][2] - A[1][0] * A[2][2]);
	ret_XPara[4] = k * (A[0][0]*A[2][2] - A[2][0] * A[0][2]);
	ret_XPara[5] = k * (A[1][0]*A[0][2] - A[0][0] * A[1][2]);
	ret_XPara[6] = k * (A[1][0]*A[2][1] - A[2][0] * A[1][1]);
	ret_XPara[7] = k * (A[0][1]*A[2][0] - A[0][0] * A[2][1]);
	ret_XPara[8] = k * (A[0][0]*A[1][1] - A[0][1] * A[1][0]);
	ret_XPara[9] = x[0];
	return true;
}

void CO3Polynorm::CalPnPara(double XPara[10],double y[4],double ret_PnPara[4])
{
	double d1 = y[1]-y[0];
	double d2 = y[2]-y[0];
	double d3 = y[3]-y[0];
	double ta = XPara[0] * d1 + XPara[1] * d2 + XPara[2] * d3;
	double tb = XPara[3] * d1 + XPara[4] * d2 + XPara[5] * d3;
	double tc = XPara[6] * d1 + XPara[7] * d2 + XPara[8] * d3;
	double ts = XPara[9];
	ret_PnPara[0] = ta;
	ret_PnPara[1] = tb - 3 * ta * ts;
	ret_PnPara[2] = tc - 2 * tb * ts + 3 * ta * ts * ts;
	ret_PnPara[3] =    -     tc * ts + tb * ts * ts  - ta * ts * ts * ts + y[0]; 	
}

bool CO3Polynorm::CalPnParaSoC(double p0_XYO1O2[4],double p1_XY[2],double ret_PnPara[4])
{
	double X0_1 = p0_XYO1O2[0];
	double X0_2 = X0_1 * X0_1;
	double X0_3 = X0_2 * X0_1;

	double X1_1 = p1_XY[0];
	double X1_2 = X1_1 * p1_XY[0];
	double X1_3 = X1_2 * p1_XY[0];

	double L = p0_XYO1O2[1];
	double M = p0_XYO1O2[2];
	double N = p0_XYO1O2[3];

	double I = X1_3 - X0_3;
	double J = X1_2 - X0_2;
	double K = X1_1 - X0_1;

	double R = p1_XY[1];
	double F = R - L;

	double dv = (I - 3 * X0_1 * J + 3 * X0_2 * K);

	double d =  0.5 * J * N + M * K - N * K * X0_1;
	ret_PnPara[0] = (F - d) / dv;
	ret_PnPara[1] = 0.5 * N - 3 * ret_PnPara[0] * X0_1;
	ret_PnPara[2] = M + 3 * ret_PnPara[0] * X0_2 - N * X0_1;
	ret_PnPara[3] = L - ((ret_PnPara[0] * X0_1  + ret_PnPara[1]) * X0_1 + ret_PnPara[2]) * X0_1;
	return true;
}
