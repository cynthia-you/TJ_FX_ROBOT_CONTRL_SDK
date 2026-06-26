
#ifndef _O3POLYNORM_H_ 
#define _O3POLYNORM_H_
/*/
	3-Order Polynome	:	Y = Pn(X) = A * X^3 + B * X^2 + + C * X + D
	Parameters of Pn(x) :	double PnPara[4] = {A,B,C,D}
	FD					:	First derivative
	SD					:	Second Derivative
	SOC					:	Second-order Continuity
	XPara				:	temporary data
	x[4] y[4]			:	y[i] = Pn(x[i])

	p0_XYO1O2[4]		:	{x[0],Pn(x[0]),FD(x[0]),SD(x[0])}
	p1_XY[2]			:	{x[1],Pn(x[1])}

/*/

class CO3Polynorm  
{
public:
	CO3Polynorm();
	virtual ~CO3Polynorm();
	static double CalPnY(double PnPara[4],double x);
	static double CalPnFD(double PnPara[4],double x);
	static double CalPnSD(double PnPara[4],double x);

	/*/ Cal Parameters of Pn(x) By four Points : x[4],y[4]/*/
	static bool   CalXPara(double x[4],double ret_XPara[10]);
	static void   CalPnPara(double XPara[10],double y[4],double ret_PnPara[4]);

	/*/ Cal Parameters of Pn(x) By two Points   :p0{x[0],Pn(x[0]),FD(x[0]),SD(x[0])} p1{x[1],Pn(x[1])}/*/
	static bool   CalPnParaSoC(double p0_XYO1O2[4],double p1_XY[2],double ret_PnPara[4]);

};

#endif 
