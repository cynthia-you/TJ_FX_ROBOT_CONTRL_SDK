//#include "pch.h"
#include "AxisPln.h"
#include "math.h"
#include "O3Polynorm.h"
#include "FxRobot.h"
#include "FXMatrix.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CAxisPln::CAxisPln()
{

}

CAxisPln::~CAxisPln()
{

}

bool CAxisPln::OnPln(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet* ret)
{
	ret->OnInit(PotT_2d);
	ret->OnEmpty();
	double s = fabs(start_pos - end_pos);

	if (s < 0.001)
	{
		double iv[2];
		iv[0] = start_pos;
		iv[1] = 0;
		ret->OnSetPoint(iv);
		iv[0] = end_pos;
		iv[1] = 0;
		ret->OnSetPoint(iv);

		return true;
	}
	if (InitPln(s, fabs(vel), fabs(acc), fabs(jerk)) == false)
	{
		return false;
	}

	double fln = fabs(acc / jerk) * 500.0;
	if (fln < 10)
	{
		fln = 10;
	}
	if (fln > 399)
	{
		fln = 399;
	}

	m_filt_cnt = fln;
	fln = m_filt_cnt;
	m_filt_pos = 0;
	long i, j;
	for (j = 0; j < m_filt_cnt; j++)
	{
		m_filt_value[j] = 0;
	}
	long num = OnGetPlnNum();
	double rp = 0.0;
	double rv; double temp;
	for (i = 0; i < num; i++)
	{
		rp = OnGetPln(&rv);
		m_filt_value[m_filt_pos] = rp;
		m_filt_pos++;
		if (m_filt_pos >= m_filt_cnt)
		{
			m_filt_pos = 0;

		}
		double vv = 0;
		for (j = 0; j < m_filt_cnt; j++)
		{
			vv += m_filt_value[j];
		}
		double iv[2];
		iv[0] = vv / fln; 
		iv[1] = 0;
		ret->OnSetPoint(iv);
	}
	
	//�����ļ���λ�ý���ƽ��
	for (i = 0; i < m_filt_cnt - 1; i++)
	{
		m_filt_value[m_filt_pos] = rp;
		m_filt_pos++;
		if (m_filt_pos >= m_filt_cnt)
		{
			m_filt_pos = 0;
		}
		double vv = 0;
		for (j = 0; j < m_filt_cnt; j++)
		{
			vv += m_filt_value[j];
		}
		double iv[2];
		iv[0] = vv / fln;
		iv[1] = 0;
		ret->OnSetPoint(iv);
	}
	
	num = ret->OnGetPointNum();//ƽ����һ����·����

	double sig = 1.0;
	if (end_pos < start_pos)
	{
		sig = -1.0;
	}

	for (i = 0; i < num; i++)
	{

		double* cur = ret->OnGetPoint(i);
		double t = cur[0];
		cur[0] = start_pos + sig * t;

	}


	for (i = 1; i < num - 1; i++)
	{
		double* pre = ret->OnGetPoint(i - 1);
		double* cur = ret->OnGetPoint(i);
		double* nex = ret->OnGetPoint(i + 1);
		cur[1] = (nex[0] - pre[0]) * 250.0; 
	}
	
	return true;


}


bool CAxisPln::OnPlnAcc(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet* ret)
{
	ret->OnInit(PotT_2d);
	ret->OnEmpty();
	double s = fabs(start_pos - end_pos);

	if (s < 0.001)
	{
		double iv[2];
		iv[0] = start_pos;
		iv[1] = 0;
		ret->OnSetPoint(iv);
		iv[0] = end_pos;
		iv[1] = 0;
		ret->OnSetPoint(iv);

		return true;
	}
	if (InitPln(s, fabs(vel), fabs(acc), fabs(jerk)) == false)
	{
		return false;
	}

	double fln = fabs(acc / jerk) * 500.0;
	if (fln < 10)
	{
		fln = 10;
	}
	if (fln > 399)
	{
		fln = 399;
	}

	m_filt_cnt = fln;
	fln = m_filt_cnt;
	m_filt_pos = 0;
	long i, j;
	for (j = 0; j < m_filt_cnt; j++)
	{
		m_filt_value[j] = 0;
	}
	long num = OnGetPlnNum();
	double rp;
	double rv; double temp;
	for (i = 0; i < num; i++)
	{
		rp = OnGetPln(&rv);
		m_filt_value[m_filt_pos] = rp;
		m_filt_pos++;
		if (m_filt_pos >= m_filt_cnt)
		{
			m_filt_pos = 0;

		}
		double vv = 0;
		for (j = 0; j < m_filt_cnt; j++)
		{
			vv += m_filt_value[j];
		}
		double iv[2];
		iv[0] = vv / fln;
		iv[1] = 0;
		ret->OnSetPoint(iv);
	}

	//�����ļ���λ�ý���ƽ��
	//for (i = 0; i < m_filt_cnt - 1; i++)
	//{
	//	m_filt_value[m_filt_pos] = rp;
	//	m_filt_pos++;
	//	if (m_filt_pos >= m_filt_cnt)
	//	{
	//		m_filt_pos = 0;
	//	}
	//	double vv = 0;
	//	for (j = 0; j < m_filt_cnt; j++)
	//	{
	//		vv += m_filt_value[j];
	//	}
	//	double iv[2];
	//	iv[0] = vv / fln;
	//	iv[1] = 0;
	//	ret->OnSetPoint(iv);
	//}

	num = ret->OnGetPointNum();//ƽ����һ����·����

	double sig = 1.0;
	if (end_pos < start_pos)
	{
		sig = -1.0;
	}

	for (i = 0; i < num; i++)
	{

		double* cur = ret->OnGetPoint(i);
		double t = cur[0];
		cur[0] = start_pos + sig * t;

	}


	for (i = 1; i < num - 1; i++)
	{
		double* pre = ret->OnGetPoint(i - 1);
		double* cur = ret->OnGetPoint(i);
		double* nex = ret->OnGetPoint(i + 1);
		cur[1] = (nex[0] - pre[0]) * 250.0;
	}


	//--�޸�1
	//if (num < 2) {
	//	return false;
	//}
	//double* pathLast = ret->OnGetPoint(num - 2);
	//double* pathEnd = ret->OnGetPoint(num - 1);
	//int last = fabs((pathLast[1] - 0) / (acc * 0.002)) + 1;
	//double pathCon[2] = { 0 };
	//pathCon[0] = pathLast[0];
	//pathCon[1] = pathLast[1];
	//for (int i = 1; i < last; i++) {
	//	double t = i * 0.002;
	//	pathCon[1] = pathLast[1] + fabs(acc) * (-sig) * t;
	//	pathCon[0] = pathLast[0] + pathLast[1] * t + 0.5 * fabs(acc) * (-sig) * t * t;
	//	if (i == 1) {
	//		pathEnd[0] = pathCon[0];
	//		pathEnd[1] = pathCon[1];
	//		continue;
	//	}
	//	ret->OnSetPoint(pathCon);
	//}
	//printf("pathCon[1]=%f--pathCon[0]-%f----\n", pathCon[1], pathCon[0]);
	////--�����һ�ι켣
	//double t_dacc = m_time_dacc / 2;

	//pathCon[0] = end_pos;
	//pathCon[1] = 0;
	//ret->OnSetPoint(pathCon);
	//--�޸�1
	
	//--�޸�2
	if (num < 2) {
		return false;
	}
	//printf("%f------\n", m_time_dacc /0.002);
	//int trajPos = num - m_time_dacc / 0.004; printf("%d------\n", trajPos);

	double* pathLast = ret->OnGetPoint(num - 2); //printf("%f------\n", pathLast[0]);
	double* pathEnd = ret->OnGetPoint(num - 1);
	int last = (m_time_dacc / 1.5) / 0.002 + 1;
	double pathCon[2] = { 0 };
	pathCon[0] = pathLast[0]; 
	pathCon[1] = pathLast[1]; 
	for (int i = 1; i < last; i++) {
		double t = i * 0.002;
		pathCon[1] = pathLast[1] + fabs(acc) * (-sig) * t; 
		pathCon[0] = pathLast[0] + pathLast[1] * t + 0.5 * fabs(acc) * (-sig) * t * t;
		if (i == 1) {
			pathEnd[0] = pathCon[0];
			pathEnd[1] = pathCon[1];
			continue;
		}
		ret->OnSetPoint(pathCon);
	}
	
	/*pathLast[0] = pathCon[0]; printf("pathCon[0]=%f-----------\n", pathCon[0]);
	pathLast[1] = pathCon[1]; printf("pathCon[1]=%f-----------\n", pathCon[1]);*/
	double trajCon[2] = { 0 };
	for (int i = 1; i < last; i++) {
		double t = i * 0.002;
		trajCon[1] = pathCon[1] + fabs(acc) * (sig) * t; //printf("pathCon[1]=%f-----------\n", pathCon[1]);
		trajCon[0] = pathCon[0] + pathCon[1] * t + 0.5 * fabs(acc) * (sig) * t * t;
		if (trajCon[1] > 0) {
			break;
		}
		ret->OnSetPoint(trajCon);
	}

	/*pathCon[0] = end_pos;
	pathCon[1] = 0;
	ret->OnSetPoint(pathCon);*/
	//--�޸�2

	return true;


}


bool CAxisPln::OnPlnAccNew(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet* ret, CPointSet* ps)
{
	//��������-����-����-����-����
	ret->OnInit(PotT_2d);
	ret->OnEmpty();
	double s = fabs(start_pos - end_pos);

	if (s < 0.001)
	{
		double iv[2];
		iv[0] = start_pos;
		iv[1] = 0;
		ret->OnSetPoint(iv);
		iv[0] = end_pos;
		iv[1] = 0;
		ret->OnSetPoint(iv);

		return true;
	}
	if (InitPln(s, fabs(vel), fabs(acc), fabs(jerk)) == false)
	{
		return false;
	}

	double fln = fabs(acc / jerk) * 500.0;
	if (fln < 10)
	{
		fln = 10;
	}
	if (fln > 399)
	{
		fln = 399;
	}

	m_filt_cnt = fln;
	fln = m_filt_cnt;
	m_filt_pos = 0;
	long i, j;
	for (j = 0; j < m_filt_cnt; j++)
	{
		m_filt_value[j] = 0;
	}
	long num = OnGetPlnNum();
	double rp;
	double rv; double temp;
	for (i = 0; i < num; i++)
	{
		rp = OnGetPln(&rv);
		m_filt_value[m_filt_pos] = rp;
		m_filt_pos++;
		if (m_filt_pos >= m_filt_cnt)
		{
			m_filt_pos = 0;

		}
		double vv = 0;
		for (j = 0; j < m_filt_cnt; j++)
		{
			vv += m_filt_value[j];
		}
		double iv[2];
		iv[0] = vv / fln;
		iv[1] = 0;
		ret->OnSetPoint(iv);
	}

	num = ret->OnGetPointNum();//ƽ����һ����·����

	double sig = 1.0;
	if (end_pos < start_pos)
	{
		sig = -1.0;
	}

	for (i = 0; i < num; i++)
	{

		double* cur = ret->OnGetPoint(i);
		double t = cur[0];
		cur[0] = start_pos + sig * t;

	}


	for (i = 1; i < num - 1; i++)
	{
		double* pre = ret->OnGetPoint(i - 1);
		double* cur = ret->OnGetPoint(i);
		double* nex = ret->OnGetPoint(i + 1);
		cur[1] = (nex[0] - pre[0]) * 250.0;
	}

	//--�޸�2
	if (num < 2) {
		return false;
	}
	int trajPos = num - m_time_dacc / 0.004; 
	for (int i = 0; i <= trajPos; i++) {
		ps->OnSetPoint(ret->OnGetPoint(i));
	}

	double* pathLast = ret->OnGetPoint(trajPos); 

	int last = num - trajPos;
	double pathCon[2] = { 0 };
	pathCon[0] = pathLast[0];
	pathCon[1] = pathLast[1];
	for (int i = 1; i < last*5; i++) {
		double t = i * 0.002;
		pathCon[1] = pathLast[1] + fabs(acc) * (-sig) * t;
		pathCon[0] = pathLast[0] + pathLast[1] * t + 0.5 * fabs(acc) * (-sig) * t * t;

		ps->OnSetPoint(pathCon);
	}

	double trajCon[2] = { 0 };
	for (int i = 1; ; i++) {
		double t = i * 0.002;
		trajCon[1] = pathCon[1] + fabs(acc) * (sig)*t; //printf("pathCon[1]=%f-----------\n", pathCon[1]);
		trajCon[0] = pathCon[0] + pathCon[1] * t + 0.5 * fabs(acc) * (sig)*t * t;
		if (trajCon[1] > 0) {
			break;
		}
		ps->OnSetPoint(trajCon);
	}

	//--�޸�2

	return true;


}

bool CAxisPln::OnPlnAccSimple(double start_pos, double vel, double accmax, CPointSet* axis)
{
	axis->OnInit(PotT_2d);
	axis->OnEmpty();
	double x0 = start_pos;
	double acc = accmax;
	double vmax = vel;
	double v0 = 0;
	double t = 0.002;
	double t2 = t * t;
	double pv[2];
	pv[0] = x0;
	pv[1] = v0;
	axis->OnSetPoint(pv);
	while (v0 < vmax)
	{
		double s = v0 * t + acc * 0.5 * t2;
		v0 += t * acc;
		x0 += s;

		pv[0] = x0;
		pv[1] = v0;
		axis->OnSetPoint(pv);
	}

	while (v0 > -vmax)
	{
		double s = v0 * t - acc * 0.5 * t2;
		v0 -= t * acc;
		x0 += s;

		pv[0] = x0;
		pv[1] = v0;
		axis->OnSetPoint(pv);
	}



	while (v0 < 0)
	{
		double s = v0 * t + acc * 0.5 * t2;
		v0 += t * acc;
		x0 += s;

		pv[0] = x0;
		pv[1] = v0;
		axis->OnSetPoint(pv);
	}

	pv[0] = x0;
	pv[1] = v0;
	axis->OnSetPoint(pv);
	return true;


	/*CPointSet allAxis;
	allAxis.OnInit(PotT_9d);
	allAxis.OnEmpty();
	double ax[9] = { 0,0,-90,0,-90,0,0,0,10000 };

	long i;
	long num = axis.OnGetPointNum();

	double* p = axis.OnGetPoint(0);

	ax[5] = p[0];


	for (i = 0; i < 10; i++)
	{
		allAxis.OnSetPoint(ax);
	}


	for (i = 0; i < num; i++)
	{
		p = axis.OnGetPoint(i);
		ax[5] = p[0];
		ax[8] = 0;

		if (p[1] > -20 && p[1] < 20 && p[2] < -199)
		{
			ax[8] = 33;
		}

		allAxis.OnSetPoint(ax);
	}

	p = axis.OnGetPoint(num - 1);

	ax[5] = p[0];

	ax[8] = 10000;
	for (i = 0; i < 10; i++)
	{
		allAxis.OnSetPoint(ax);
	}

	allAxis.OnSave("..\\AllAxis_6_izz");
	allAxis.OnSaveCSV("..\\AllAxis_6_izz.CSV");*/
}

bool CAxisPln::InitPln(double s, double v, double a, double j)
{
	m_s = s;
	m_v = v;
	m_a = a;
	double acc_t = v / a;
	double acc_s = 0.5 * v * acc_t;
	if (acc_s < 0.5 * m_s)
	{
		m_time_acc = acc_t; //printf("a_max-----%f\n", m_v / m_time_acc);
		m_time_dacc = acc_t;
		m_time_vel = (m_s - 2 * acc_s) / m_v;
	}
	else
	{
		m_time_acc = sqrt(m_s / m_a);
		m_time_dacc = m_time_acc; 
		m_time_vel = 0;
		m_v = m_time_acc * m_a; //printf("m_v-----%f\n", m_v);
	}
	m_cur_time = 0.0;
	return true;
}

long CAxisPln::OnGetPlnNum()
{
	double t = m_time_acc + m_time_dacc + m_time_vel;
	double t_num = t / 0.002;
	long ret = t_num + 2;
	return ret;
}

double CAxisPln::OnGetPln(double* ret_v)
{
	//�����з��ص�ǰ�ٶȣ��������ص�ǰλ��
	if (m_cur_time <= m_time_acc)
	{
		double s = 0.5 * m_a * m_cur_time * m_cur_time;
		*ret_v = m_cur_time * m_a; 
		m_cur_time += 0.002;
		return s;
	}
	if (m_cur_time <= (m_time_acc + m_time_vel))
	{
		double s1 = 0.5 * m_a * m_time_acc * m_time_acc;
		double s2 = m_v * (m_cur_time - m_time_acc);
		double s = s1 + s2;
		*ret_v = m_v;
		m_cur_time += 0.002;
		return s;
	}

	if (m_cur_time <= (m_time_acc + m_time_vel + m_time_acc))
	{
		double s1 = 0.5 * m_a * m_time_acc * m_time_acc;
		double s2 = m_v * (m_time_vel);
		double d_t = m_cur_time - m_time_acc - m_time_vel;
		double v_t = m_v - d_t * m_a;
		double s3 = 0.5 * (v_t + m_v) * d_t;
		double s = s1 + s2 + s3;

		*ret_v = v_t;

		m_cur_time += 0.002;
		return s;
	}

	*ret_v = 0;
	return m_s;

}

bool CAxisPln::OnMovL(long RobotSetial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, char* path)
{
	///////determine same points
	long i = 0;
	long j = 0;
	long same_tag[6] = { 0 };
	for (i = 0; i < 6; i++)
	{
		if (fabs(end_pos[i] - start_pos[i]) < 0.01)
		{
			same_tag[i] = 1;
		}
	}
	///////Check Max Axis
	CPointSet ret[6];
	long num[3] = { 0 };//ret[0].OnGetPointNum();
	long max_num = 0;
	long max_num_axis = 0;

	for (i = 0; i < 3; i++)
	{
		OnPln(start_pos[i], end_pos[i], vel, acc, jerk, &ret[i]);
		num[i] = ret[i].OnGetPointNum();
		if (num[i] > max_num)
		{
			max_num = num[i];
			max_num_axis = i;
		}
	}

	//Cuter Euler-Angle
	double Q_start[4] = { 0 };
	double Q_end[4] = { 0 };
	FX_ABC2Quaternions(start_pos, Q_start);
	FX_ABC2Quaternions(end_pos, Q_end);

	//Calculate Quaternions Angle
	double cosangle = Q_start[0] * Q_end[0] + Q_start[1] * Q_end[1] +
					  Q_start[2] * Q_end[2] + Q_start[3] * Q_end[3];
	if (cosangle < 0.0)
	{
		cosangle = -cosangle;
		Q_end[0] = -Q_end[0];
		Q_end[1] = -Q_end[1];
		Q_end[2] = -Q_end[2];
		Q_end[3] = -Q_end[3];
	}
	double qangle = FX_ACOS(cosangle) * 2 * FXARM_R2D;

	//Cut Quaterniongs PLN
	OnPln(0, qangle, vel, acc, jerk, &ret[3]);
	double qnum = ret[3].OnGetPointNum();

	if (qnum > max_num)
	{
		max_num = qnum;
		max_num_axis = 3;
	}


	CPointSet out;
	out.OnInit(PotT_9d);
	double tmp[9] = { 0 };

	for (i = 0; i < max_num; i++)
	{
		double* p = ret[max_num_axis].OnGetPoint(i);
		tmp[0] = end_pos[0];
		tmp[1] = end_pos[1];
		tmp[2] = end_pos[2];
		tmp[max_num_axis] = p[0];

		if ((same_tag[3] + same_tag[4] + same_tag[5]) < 3)
		{
			double ratio;
			if (max_num_axis == 3)
			{
				ratio = p[0]/ qangle;
				FX_QuaternionSlerp(Q_start, Q_end, ratio, &tmp[3]);
			}
			else
			{
				ratio = i / (double)(max_num - 1);
				FX_QuaternionSlerp(Q_start, Q_end, ratio, &tmp[3]);
			}
			
		}
		else
		{
			tmp[3] = Q_start[0];
			tmp[4] = Q_start[1];
			tmp[5] = Q_start[2];
			tmp[6] = Q_start[3];
		}

		out.OnSetPoint(tmp);
	}

	//set 4 same point
	for (i = 0; i < 4; i++)
	{
		out.OnSetPoint(tmp);
	}

	long dof = 0;
	bool end_tag=false;
	for(dof=0;dof<3;dof++)
	{
		if (dof != max_num_axis )
		{
			if (same_tag[dof] == 0)
			{
				double step = (double)(num[dof] - 1) / (max_num + 1);
				long   serial = 0;
				double tmpy = 0;
				for (i = 0; i < num[dof] - 3; i += 2)
				{
					double* p1 = ret[dof].OnGetPoint(i);
					double* p2 = ret[dof].OnGetPoint(i + 1);
					double* p3 = ret[dof].OnGetPoint(i + 2);
					double* p4 = ret[dof].OnGetPoint(i + 3);

					double x[4];
					double y[4];
					double xpara[10];
					double retpara[4];

					x[0] = i;
					x[1] = i + 1;
					x[2] = i + 2;
					x[3] = i + 3;

					y[0] = p1[0];
					y[1] = p2[0];
					y[2] = p3[0];
					y[3] = p4[0];

					CO3Polynorm::CalXPara(x, xpara);
					CO3Polynorm::CalPnPara(xpara, y, retpara);

					if (i == 0)
					{
						//for (j = 0; j < 3; j++)
						for (; tmpy < x[3]; tmpy = serial * step)
						{
							double sloy = CO3Polynorm::CalPnY(retpara, tmpy);
							double* p = out.OnGetPoint(serial);

							serial++;

							if (p != NULL)
							{
								p[dof] = sloy;
							}
						}
					}
					else
					{
						long k = 0;
						while (tmpy > x[0])
						{
							k++;
							tmpy -= step;
						}
						k--;
						tmpy += step;

						while (tmpy < x[1])
						{
							double sloy = CO3Polynorm::CalPnY(retpara, tmpy);
							double* p = out.OnGetPoint(serial - k);
							if (p != NULL)
							{
								double r1 = j;
								double r2;
								r1 /= step;
								r2 = 1 - r1;
								sloy = sloy * r1 + p[dof] * r2;
								p[dof] = sloy;
							}

							tmpy += step;
							k--;
						}

						while (tmpy < x[3])
						{
							double sloy = CO3Polynorm::CalPnY(retpara, tmpy);
							double* p = out.OnGetPoint(serial);

							serial++;
							tmpy += step;
							if(sloy<x[3]&&tmpy>x[3])
							{
							    end_tag=true;
							}

							if (p != NULL)
							{
								p[dof] = sloy;
							}
						}

						if(end_tag==true)
						{
						    double* p = out.OnGetPoint(serial);
						    if(p!=NULL)
						    {
						        p[dof]=end_pos[dof];
						    }
						}
					}
				}
			}
			else
			{
				for (i = 0; i < max_num; i++)
				{
					double* p = out.OnGetPoint(i);
					if (p != NULL)
					{
						p[dof] = start_pos[dof];
					}
				}
			}
		}
	}

	long final_num = out.OnGetPointNum();
	////////////////////InvKine//////////////
	FX_InvKineSolvePara sp;

	for (i = 0; i < 7; i++)
	{
		sp.m_Input_IK_RefJoint[i] = ref_joints[i];
	}

	CPointSet final_points;
	final_points.OnInit(PotT_9d);
	double tmppoints[7] = { 0 };
	double TCP[4][4];
	double ret_joints[9] = { 0 };
	//initial 
	for (i = 0; i < 4; i++)
	{
		for (j = 0;j < 4; j++)
		{
			TCP[i][j] = 0;
		}
	}

	for (i = 0; i < final_num; i++)
	{
		double* pp = out.OnGetPoint(i);
		tmppoints[0] = pp[0];
		tmppoints[1] = pp[1];
		tmppoints[2] = pp[2];
		tmppoints[3] = pp[3];
		tmppoints[4] = pp[4];
		tmppoints[5] = pp[5];
		tmppoints[6] = pp[6];

		FX_Quaternions2ABCMatrix(&tmppoints[3], &tmppoints[0], TCP);
		for (dof = 0; dof < 4; dof++)
		{
			for (j = 0; j < 4; j++)
			{
				sp.m_Input_IK_TargetTCP[dof][j] = TCP[dof][j];
			}
		}

		if(FX_Robot_Kine_IK(RobotSetial, &sp)==false)
		{
			return FX_FALSE;
		}

		// Error feedback
		for (long kk = 0; kk < 7; kk++)
		{
			if (sp.m_Output_JntExdTags[kk] == FX_TRUE)
			{
				//printf("Joint %d exceed limit \n", kk);
				return FX_FALSE;
			}
		}

		if (sp.m_Output_IsOutRange == FX_TRUE)
		{
			//printf("Input Position over reachable space\n");
			return FX_FALSE;
		}
		

		ret_joints[0] = sp.m_Output_RetJoint[0];
		ret_joints[1] = sp.m_Output_RetJoint[1];
		ret_joints[2] = sp.m_Output_RetJoint[2];
		ret_joints[3] = sp.m_Output_RetJoint[3];
		ret_joints[4] = sp.m_Output_RetJoint[4];
		ret_joints[5] = sp.m_Output_RetJoint[5];
		ret_joints[6] = sp.m_Output_RetJoint[6];

		final_points.OnSetPoint(ret_joints);
	}

	//char apth[] = "D:\\cccc\\SPMOVL\\OutPVT.txt";
	char* pp = path;
	final_points.OnSave(path);

	return true;
}

bool CAxisPln::OnMovL_KeepJ(long RobotSerial, double startjoints[7], double stopjoints[7], double vel, char* path)
{
	CPointSet retJoints;

	FX_INT32L i, j;
	retJoints.OnInit(PotT_9d);
	retJoints.OnEmpty();
	Matrix4 pg_start;
	Matrix4 pg_stop;
	Matrix3 nspg_start;
	Matrix3 nspg_stop;
	FX_Robot_Kine_FK_NSP(RobotSerial, startjoints, pg_start, nspg_start);
	FX_Robot_Kine_FK_NSP(RobotSerial, stopjoints, pg_stop, nspg_stop);

	CPointSet pset;
	pset.OnInit(PotT_40d);

	Quaternion q_start;
	Quaternion q_stop;

	Quaternion q_nsp_start;
	Quaternion q_nsp_stop;

	FX_Matrix2Quaternion4(pg_start, q_start);
	FX_Matrix2Quaternion4(pg_stop, q_stop);

	FX_Matrix2Quaternion3(nspg_start, q_nsp_start);
	FX_Matrix2Quaternion3(nspg_stop, q_nsp_stop);

	double dx = pg_start[0][3] - pg_stop[0][3];
	double dy = pg_start[1][3] - pg_stop[1][3];
	double dz = pg_start[2][3] - pg_stop[2][3];
	double length_pos = sqrt(dx * dx + dy * dy + dz * dz);
	if (vel < 0.1)
	{
		vel = 0.1;
	}
	FX_DOUBLE cut_step = vel * 0.002;

	long tnum = length_pos / cut_step + 2;
	double dnum = tnum;
	double input[40];
	for (i = 0; i < 40; i++)
	{
		input[i] = 0;
	}
	for (i = 0; i <= tnum; i++)
	{
		double r = i;
		r /= dnum;
		input[0] = pg_start[0][3] * (1.0 - r) + r * pg_stop[0][3];
		input[1] = pg_start[1][3] * (1.0 - r) + r * pg_stop[1][3];
		input[2] = pg_start[2][3] * (1.0 - r) + r * pg_stop[2][3];
		FX_QuaternionSlerp(q_start, q_stop, r, &input[3]);
		Quaternion nspq;
		FX_QuaternionSlerp(q_nsp_start, q_nsp_stop, r, nspq);
		Matrix3 tmpm;
		FX_Quaternions2Matrix3(nspq, tmpm);
		input[7] = tmpm[0][0];
		input[8] = tmpm[1][0];
		input[9] = tmpm[2][0];

		pset.OnSetPoint(input);
	}

	FX_INT32L num = 0;
	num = pset.OnGetPointNum();
	FX_InvKineSolvePara sp;
	sp.m_DGR1 = 10;
	sp.m_DGR2 = 10;
	sp.m_DGR3 = 10;

	double last_joint[7];
	for (i = 0; i < 7; i++)
	{
		//printf("%.2lf ", startjoints[i]);
		last_joint[i] = startjoints[i];
		sp.m_Input_IK_RefJoint[i] = startjoints[i];
		sp.m_Output_RetJoint[i] = startjoints[i];
	}

	//printf("\n");

	bool _jext = false;

	for (i = 0; i < num; i++)
	{
		double* p = pset.OnGetPoint(i);
		FX_Quaternions2ABCMatrix(&p[3], &p[0], sp.m_Input_IK_TargetTCP);
		sp.m_Input_IK_ZSPPara[0] = p[7];
		sp.m_Input_IK_ZSPPara[1] = p[8];
		sp.m_Input_IK_ZSPPara[2] = p[9];
		sp.m_Input_IK_ZSPType = 1;

		for (j = 0; j < 7; j++)
		{
			sp.m_Input_IK_RefJoint[j] = last_joint[j];
		}

		if (FX_Robot_Kine_IK(RobotSerial, &sp) == FX_FALSE)
		{
			return false;
		}

		for (j = 0; j < 7; j++)
		{
			p[j + 10] = sp.m_Output_RetJoint[j];
			last_joint[j] = sp.m_Output_RetJoint[j];
		}


		if (sp.m_Output_IsJntExd == FX_TRUE)
		{
			_jext = true;
			double cur_ext = sp.m_Output_JntExdABS;

			double old_ext = cur_ext;
			long dir = 1;
			sp.m_Input_ZSP_Angle = 0.01;
			FX_Robot_Kine_IK_NSP(RobotSerial, &sp);
			double t_ext1 = sp.m_Output_JntExdABS;
			sp.m_Input_ZSP_Angle = -0.01;
			FX_Robot_Kine_IK_NSP(RobotSerial, &sp);
			double t_ext2 = sp.m_Output_JntExdABS;

			if (t_ext2 < t_ext1)
			{
				if (cur_ext < t_ext2)
				{
					//printf("A\n");
					return false;
				}
				dir = -1;
				old_ext = cur_ext;
			}
			else
			{

				if (cur_ext < t_ext1)
				{
					//printf("B\n");
					return false;
				}
			}

			sp.m_Input_ZSP_Angle = dir;
			while (cur_ext > 0.00001)
			{
				FX_Robot_Kine_IK_NSP(RobotSerial, &sp);
				cur_ext = sp.m_Output_JntExdABS;
				//printf("<%lf/%lf> \n ", cur_ext,sp.m_Input_ZSP_Angle);
				if (FX_Fabs(sp.m_Input_ZSP_Angle) > 360)
				{
					printf("\n\n no result\n");
					return false;
				}

				//printf("%.2lf  -->\n ", cur_ext);

				sp.m_Input_ZSP_Angle += dir;



			}

			//printf("---------------------\n");

			sp.m_Input_ZSP_Angle -= dir;
			p[17] = sp.m_Input_ZSP_Angle;
			//printf("[%d]<%lf>   ----- ", i, p[17]);


		}
		else
		{
			//printf("[%d] ----- ", i);
		}


		for (j = 0; j < 7; j++)
		{
			p[j + 19] = sp.m_Output_RetJoint[j];
		}

		retJoints.OnSetPoint(&p[19]);
	}

	long final_num = retJoints.OnGetPointNum();
	char* pp = path;
	if (retJoints.OnSave(path) == false)
	{
		printf("num= %d false\n",final_num);
	}

	return true;
}

bool CAxisPln::OnMovL_KeepJ_Cut(long RobotSerial, double startjoints[7], double stopjoints[7], double vel, double acc, char* path)
{
	//Final result in retjoints
	CPointSet retJoints;
	FX_INT32L i = 0;
	FX_INT32L j = 0;
	retJoints.OnInit(PotT_9d);
	retJoints.OnEmpty();

	//Pset for XYZ Q NSP 
	CPointSet pset;
	pset.OnInit(PotT_40d);

	//XYZ Q NSP
	Matrix4 pg_start;
	Matrix4 pg_stop;
	Matrix3 nspg_start;
	Matrix3 nspg_stop;
	FX_Robot_Kine_FK_NSP(RobotSerial, startjoints, pg_start, nspg_start);
	FX_Robot_Kine_FK_NSP(RobotSerial, stopjoints, pg_stop, nspg_stop);

	Quaternion q_start;
	Quaternion q_stop;

	Quaternion q_nsp_start;
	Quaternion q_nsp_stop;

	FX_Matrix2Quaternion4(pg_start, q_start);
	FX_Matrix2Quaternion4(pg_stop, q_stop);

	FX_Matrix2Quaternion3(nspg_start, q_nsp_start);
	FX_Matrix2Quaternion3(nspg_stop, q_nsp_stop);
	/////////////
	///////determine same points:For XYZ
	long same_tag[3] = { 0 };
	for (i = 0; i < 3; i++)
	{
		if (fabs(pg_start[i][3] - pg_stop[i][3]) < 0.01)
		{
			same_tag[i] = 1;
		}
	}
	///////Check Max Axis
	CPointSet ret[3];
	long num_axis[3] = { 0 };//ret[0].OnGetPointNum();
	long max_num = 0;
	long max_num_axis = 0;

	//double acc = vel * 10;
	double jerk = acc;

	for (i = 0; i < 3; i++)
	{
		OnPln(pg_start[i][3], pg_stop[i][3], vel, acc, jerk, &ret[i]);
		num_axis[i] = ret[i].OnGetPointNum();
		if (num_axis[i] > max_num)
		{
			max_num = num_axis[i];
			max_num_axis = i;
		}
	}

	double input[40];
	for (i = 0; i < 40; i++)
	{
		input[i] = 0;
	}

	for (i = 0; i < max_num; i++)
	{
		double* p = ret[max_num_axis].OnGetPoint(i);
		input[0] = pg_stop[0][3];
		input[1] = pg_stop[1][3];
		input[2] = pg_stop[2][3];
		input[max_num_axis] = p[0];

		double ratio = i / (double)(max_num - 1);
		FX_QuaternionSlerp(q_start, q_stop, ratio, &input[3]);
		Quaternion nspq;
		FX_QuaternionSlerp(q_nsp_start, q_nsp_stop, ratio, nspq);
		Matrix3 tmpm;
		FX_Quaternions2Matrix3(nspq, tmpm);
		input[7] = tmpm[0][0];
		input[8] = tmpm[1][0];
		input[9] = tmpm[2][0];

		pset.OnSetPoint(input);
	}

	//set 4 same point
	for (i = 0; i < 4; i++)
	{
		pset.OnSetPoint(input);
	}

	long dof = 0;
	bool end_tag = false;
	for (dof = 0; dof < 3; dof++)
	{
		if (dof != max_num_axis)
		{
			if (same_tag[dof] == 0)
			{
				double step = (double)(num_axis[dof] - 1) / (max_num + 1);
				long   serial = 0;
				double tmpy = 0;
				for (i = 0; i < num_axis[dof] - 3; i += 2)
				{
					double* p1 = ret[dof].OnGetPoint(i);
					double* p2 = ret[dof].OnGetPoint(i + 1);
					double* p3 = ret[dof].OnGetPoint(i + 2);
					double* p4 = ret[dof].OnGetPoint(i + 3);

					double x[4];
					double y[4];
					double xpara[10];
					double retpara[4];

					x[0] = i;
					x[1] = i + 1;
					x[2] = i + 2;
					x[3] = i + 3;

					y[0] = p1[0];
					y[1] = p2[0];
					y[2] = p3[0];
					y[3] = p4[0];

					CO3Polynorm::CalXPara(x, xpara);
					CO3Polynorm::CalPnPara(xpara, y, retpara);

					if (i == 0)
					{
						//for (j = 0; j < 3; j++)
						for (; tmpy < x[3]; tmpy = serial * step)
						{
							double sloy = CO3Polynorm::CalPnY(retpara, tmpy);
							double* p = pset.OnGetPoint(serial);

							serial++;

							if (p != NULL)
							{
								p[dof] = sloy;
							}
						}
					}
					else
					{
						long k = 0;
						while (tmpy > x[0])
						{
							k++;
							tmpy -= step;
						}
						k--;
						tmpy += step;

						while (tmpy < x[1])
						{
							double sloy = CO3Polynorm::CalPnY(retpara, tmpy);
							double* p = pset.OnGetPoint(serial - k);
							if (p != NULL)
							{
								double r1 = j;
								double r2;
								r1 /= step;
								r2 = 1 - r1;
								sloy = sloy * r1 + p[dof] * r2;
								p[dof] = sloy;
							}

							tmpy += step;
							k--;
						}

						while (tmpy < x[3])
						{
							double sloy = CO3Polynorm::CalPnY(retpara, tmpy);
							double* p = pset.OnGetPoint(serial);

							serial++;
							tmpy += step;
							if (sloy<x[3] && tmpy>x[3])
							{
								//add last point
								end_tag = true;
							}

							if (p != NULL)
							{
								p[dof] = sloy;
							}
						}

						if (end_tag == true)
						{
							double* p = pset.OnGetPoint(serial);
							if (p != NULL)
							{
								p[dof] = pg_stop[dof][3];
							}
						}

					}
				}
			}
			else
			{
				for (i = 0; i < max_num; i++)
				{
					double* p = pset.OnGetPoint(i);
					if (p != NULL)
					{
						p[dof] = pg_start[dof][3];
					}
				}
			}
		}
	}
	
	char apthhh[] = "D:\\cccc\\SPMOVL\\MOVL_cut_xyz.txt";
	char* pppp = apthhh;
	pset.OnSave(pppp);

	FX_INT32L num = 0;
	num = pset.OnGetPointNum();
	FX_InvKineSolvePara sp;
	sp.m_DGR1 = 10;
	sp.m_DGR2 = 10;
	sp.m_DGR3 = 10;

	double last_joint[7];
	for (i = 0; i < 7; i++)
	{
		last_joint[i] = startjoints[i];
		sp.m_Input_IK_RefJoint[i] = startjoints[i];
		sp.m_Output_RetJoint[i] = startjoints[i];
	}

	bool _jext = false;

	for (i = 0; i < num; i++)
	{
		double* p = pset.OnGetPoint(i);
		FX_Quaternions2ABCMatrix(&p[3], &p[0], sp.m_Input_IK_TargetTCP);
		sp.m_Input_IK_ZSPPara[0] = p[7];
		sp.m_Input_IK_ZSPPara[1] = p[8];
		sp.m_Input_IK_ZSPPara[2] = p[9];
		sp.m_Input_IK_ZSPType = 1;

		for (j = 0; j < 7; j++)
		{
			sp.m_Input_IK_RefJoint[j] = last_joint[j];
		}

		if (FX_Robot_Kine_IK(RobotSerial, &sp) == FX_FALSE)
		{
			return false;
		}

		for (j = 0; j < 7; j++)
		{
			p[j + 10] = sp.m_Output_RetJoint[j];
			last_joint[j] = sp.m_Output_RetJoint[j];
		}


		if (sp.m_Output_IsJntExd == FX_TRUE)
		{
			_jext = true;
			double cur_ext = sp.m_Output_JntExdABS;

			double old_ext = cur_ext;
			long dir = 1;
			sp.m_Input_ZSP_Angle = 0.01;
			FX_Robot_Kine_IK_NSP(RobotSerial, &sp);
			double t_ext1 = sp.m_Output_JntExdABS;
			sp.m_Input_ZSP_Angle = -0.01;
			FX_Robot_Kine_IK_NSP(RobotSerial, &sp);
			double t_ext2 = sp.m_Output_JntExdABS;

			if (t_ext2 < t_ext1)
			{
				if (cur_ext < t_ext2)
				{
					//printf("A\n");
					return false;
				}
				dir = -1;
				old_ext = cur_ext;
			}
			else
			{

				if (cur_ext < t_ext1)
				{
					//printf("B\n");
					return false;
				}
			}

			sp.m_Input_ZSP_Angle = dir;
			while (cur_ext > 0.00001)
			{
				FX_Robot_Kine_IK_NSP(RobotSerial, &sp);
				cur_ext = sp.m_Output_JntExdABS;
				//printf("<%lf/%lf> \n ", cur_ext,sp.m_Input_ZSP_Angle);
				if (FX_Fabs(sp.m_Input_ZSP_Angle) > 360)
				{
					//printf("\n\n no result\n");
					return false;
				}

				//printf("%.2lf  -->\n ", cur_ext);

				sp.m_Input_ZSP_Angle += dir;



			}

			//printf("---------------------\n");

			sp.m_Input_ZSP_Angle -= dir;
			p[17] = sp.m_Input_ZSP_Angle;
			//printf("[%d]<%lf>   ----- ", i, p[17]);


		}

		for (j = 0; j < 7; j++)
		{
			p[j + 19] = sp.m_Output_RetJoint[j];
		}

		retJoints.OnSetPoint(&p[19]);
	}

	long final_num = retJoints.OnGetPointNum();
	char* pp = path;
	retJoints.OnSave(path);

	return true;
}


bool CAxisPln::OnMovJ(long RobotSetial, double start_joint[7], double end_joint[7], double vel, double acc, double jerk, char* path)
{
	///////determine same joints
	long i = 0;
	long j = 0;
	long same_tag[7] = { 0 };

	for (i = 0; i < 7; i++)
	{
		if (fabs(end_joint[i] - start_joint[i]) < 0.01)
		{
			same_tag[i] = 1;
		}
	}

	CPointSet ret[7];
	long num[7] = { 0 };
	long max_num = 0;
	long max_axis = 0;
	for (i = 0; i < 7; i++)
	{
		if (!same_tag[i])
		{
			OnPln(start_joint[i], end_joint[i], vel, acc, jerk, &ret[i]);
			num[i] = ret[i].OnGetPointNum();
			if (num[i] > max_num)
			{
				max_num = num[i];
				max_axis = i;
			}
		}
	}

	CPointSet final_ret;
	final_ret.OnInit(PotT_9d);
	double out_joints[9] = { 0 };
	for (i = 0; i < max_num; i++)
	{
		for (j = 0; j < 7; j++)
		{
			if (i < num[j])
			{
				double* p = ret[j].OnGetPoint(i);
				out_joints[j] = p[0];
			}
			else
			{
				out_joints[j] = end_joint[j];
			}
		}
		final_ret.OnSetPoint(out_joints);
	}

	long final_num = final_ret.OnGetPointNum();
	char* pp = path;
	if (final_ret.OnSave(path) == false)
	{
		printf("num= %d false\n",final_num);
	}

	return true;
}
