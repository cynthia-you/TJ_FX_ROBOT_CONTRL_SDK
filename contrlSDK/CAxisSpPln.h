#ifndef _FX_AXIS_SP_PLN_H_
#define _FX_AXIS_SP_PLN_H_

class CAxisSpPln
{
public:
	CAxisSpPln();
	virtual ~CAxisSpPln();
	bool OnSetLmt(long dof,double PosNeg[8], double PosPos[8], double VelLmt[8], double AccLmt[8]);
	long OnPln(double startp[8],double stopp[8],double vel_ratio,double acc_ratio);
	bool OnCut(double retp[8]);
protected:
	long m_dof;
	double m_PosNeg[8];
	double m_PosPos[8];
	double m_VelLmt[8];
	double m_AccLmt[8];


	double m_start[8];
	double m_stop[8];
	long   m_Pln_Type[8];
	double m_Pln_Len[8];
	double m_Pln_TRatio[8];
	double   m_Pln_T[8];
	double   m_Pln_P1[8][6]; // start_pos vel acc len t r
	double   m_Pln_P2[8][6];
	double   m_Pln_P3[8][6];

	double m_totl_t;
	double m_cur_t;


	double m_value[8][10];
	long   m_wpos;
	bool   m_FristTag;

	double m_ts;
	
};



#endif

