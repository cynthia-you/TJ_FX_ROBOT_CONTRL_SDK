#include "CAxisSpPln.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

CAxisSpPln::CAxisSpPln()
{
	m_dof = 0;
	m_ts = 0.02;
}
CAxisSpPln::~CAxisSpPln()
{

}
bool CAxisSpPln::OnSetLmt(long dof, double PosNeg[8], double PosPos[8], double VelLmt[8], double AccLmt[8])
{
	if (dof <= 0 || m_dof > 8)
	{
		return false;
	}
	m_dof = dof;
	for (long i = 0; i < dof; i++)
	{
		m_PosNeg[i] = PosNeg[i];
		m_PosPos[i] = PosPos[i];
		m_VelLmt[i] = VelLmt[i];
		m_AccLmt[i] = AccLmt[i];
		printf("NPVA %d	%lf %lf	%lf	%lf	\n",i,m_PosNeg[i],m_PosPos[i],m_VelLmt[i],m_AccLmt[i]);
	}
	m_ts = 0.02;

	return true;
}
long CAxisSpPln::OnPln(double startp[8], double stopp[8], double vel_ratio, double acc_ratio)
{
	if (m_dof <= 0)
	{
		return -1;
	}
	{
		long i;
		for ( i = 0; i < m_dof; i++)
		{
			if (startp[i] < m_PosNeg[i] || startp[i] > m_PosPos[i])
			{
				return -1;
			}
			if (stopp[i] < m_PosNeg[i] || stopp[i] > m_PosPos[i])
			{
				return -1;
			}
		}
	}
	long i;

	double vr = vel_ratio;
	if (vr < 0.01)
	{
		vr = 0.01;
	}
	if (vr > 1)
	{
		vr = 1;
	}


	double ar = vel_ratio;
	if (acc_ratio < 0.01)
	{
		acc_ratio = 0.01;
	}
	if (acc_ratio > 1)
	{
		acc_ratio = 1;
	}

	double t_max = 0;
	long   t_max_axis_num = -1;
	for ( i = 0; i < m_dof; i++)
	{
		m_start[i] = startp[i];
		m_stop[i] = stopp[i];
		double dsp = startp[i] - stopp[i];

		if (dsp < 0)
		{
			dsp = -dsp;
		}
		m_Pln_Len[i] = dsp;
		if (dsp < 0.1)
		{
			m_Pln_Type[i] = 0;
		}
		else
		{
			double v = m_VelLmt[i] * vr;
			double a = m_AccLmt[i] * ar;
			double sa = 0.5 * v * v / a;

			if (sa * 2.0 >= dsp)
			{
				m_Pln_Type[i] = 1;
				double t1 = sqrt( dsp / a);
				double t = 2 * t1;

				m_Pln_T[i] = t;
				double v = t1 * a;
				m_Pln_P1[i][0] = 0;
				m_Pln_P1[i][1] = 0;
				m_Pln_P1[i][2] = a;
				m_Pln_P1[i][3] = t1;

				m_Pln_P3[i][0] = dsp*0.5;
				m_Pln_P3[i][1] = v;
				m_Pln_P3[i][2] = -a;
				m_Pln_P3[i][3] =  t;

				if (t >= t_max)
				{
					t_max = t;
					t_max_axis_num = i;
				}
			}
			else
			{
				m_Pln_Type[i] = 2;
				double t1 = v / a;
				double t2 = (dsp - 2 * sa)/v;
				double t = 2 * t1 + t2;

				m_Pln_T[i] = t;
				m_Pln_P1[i][0] = 0;
				m_Pln_P1[i][1] = 0;
				m_Pln_P1[i][2] = a;
				m_Pln_P1[i][3] = t1;

				m_Pln_P2[i][0] = sa ;
				m_Pln_P2[i][1] = v;
				m_Pln_P2[i][2] = 0;
				m_Pln_P2[i][3] = t1 + t2;

				m_Pln_P3[i][0] = dsp -  sa;
				m_Pln_P3[i][1] = v;
				m_Pln_P3[i][2] = -a;
				m_Pln_P3[i][3] =  t;

				if (t >= t_max)
				{
					t_max = t;
					t_max_axis_num = i;
				}

			}
		}
	}

	if (t_max_axis_num == -1)
	{
		return 0;
	}

	for ( i = 0; i < m_dof; i++)
	{
		if (i == t_max_axis_num)
		{
			m_Pln_TRatio[i] = 1;
		}
		else
		{
			if (m_Pln_Type[i] != 0)
			{
				m_Pln_TRatio[i] = m_Pln_T[i] / t_max;
			}
		}
		
	}
	m_totl_t = t_max;
	m_cur_t = 0;

	m_FristTag = true;
	return t_max / m_ts + 6;
}


bool CAxisSpPln::OnCut(double retp[8])
{
	long i;
	long j;
	for ( i = 0; i < m_dof; i++)
	{
		double cut_t = m_cur_t;
		if (m_Pln_Type[i] == 0)
		{
			double r = cut_t / m_totl_t;
			retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
		}
		if (m_Pln_Type[i] == 1)
		{
			cut_t *= m_Pln_TRatio[i];

			if (cut_t < m_Pln_P1[i][3])
			{
				double len = 0.5 * m_Pln_P1[i][2] * cut_t * cut_t;
				double r = len / m_Pln_Len[i];
				retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
			}
			else
			{
				cut_t -= m_Pln_P1[i][3];
				double len = m_Pln_P3[i][0] + m_Pln_P3[i][1]* cut_t +  0.5 * m_Pln_P3[i][2] * cut_t * cut_t;

				double r = len / m_Pln_Len[i];
				retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
			}
		}
		if (m_Pln_Type[i] == 2)
		{
			cut_t *= m_Pln_TRatio[i];

			if (cut_t < m_Pln_P1[i][3])
			{
				double len = 0.5 * m_Pln_P1[i][2] * cut_t * cut_t;
				double r = len / m_Pln_Len[i];
				retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
				printf("r1 %lf ",r);
			}
			else
			{
				if (cut_t < m_Pln_P2[i][3])
				{
					cut_t -= m_Pln_P1[i][3];
					double len = m_Pln_P2[i][0] + m_Pln_P2[i][1] * cut_t;
					double r = len / m_Pln_Len[i];
					retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i]; 
					printf("r2 %lf ", r);
				}
				else
				{
					cut_t -= m_Pln_P2[i][3];
					double len = m_Pln_P3[i][0] + m_Pln_P3[i][1] * cut_t + 0.5 * m_Pln_P3[i][2] * cut_t * cut_t;
					double r = len / m_Pln_Len[i];
					retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
					printf("r3 %lf ", r);
				}
				
			}
		}
	}

	m_cur_t += m_ts;
	if (m_cur_t >= m_totl_t)
	{
		m_cur_t = m_totl_t;
	}


	if (m_FristTag == true)
	{
		m_FristTag = false;
		m_wpos = 0;
		for ( i = 0; i < m_dof; i++)
		{
			for ( j = 0; j < 5; j++)
			{
				m_value[i][j] = retp[i];
			}
		}
		return true;
	}
	else
	{
		for (i = 0; i < m_dof; i++)
		{
		  m_value[i][m_wpos] = retp[i];	
		  retp[i] = 0;
		  for ( j = 0; j < 5; j++)
		  {
			  retp[i] += m_value[i][j];
		  }
		  retp[i] *= 0.2;
		}
		m_wpos++;
		if (m_wpos >= 5)
		{
			m_wpos = 0;
		}

	}

	return true;
}
