//#include "pch.h"
#include "AxisPln.h"
#include "FXMath.h"
#include "math.h"
#include "O3Polynorm.h"
#include "FxRobot.h"

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

bool CAxisPln::QuaternionNorm(double Q[4])
{
	double qq = FX_Sqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
	if (qq <= FXARM_EPS)
	{
		return FX_FALSE;
	}
	Q[0] /= qq;
	Q[1] /= qq;
	Q[2] /= qq;
	Q[3] /= qq;

	return true;
}

void CAxisPln::ABC2Quaternions(double XYZABC[6], double Q[4])
{
	Matrix3 Trm;
	double sa;
	double sb;
	double sr;
	double ca;
	double cb;
	double cr;

	FX_SIN_COS_DEG(XYZABC[5], &sa, &ca);
	FX_SIN_COS_DEG(XYZABC[4], &sb, &cb);
	FX_SIN_COS_DEG(XYZABC[3], &sr, &cr);

	Trm[0][0] = ca * cb;
	Trm[0][1] = ca * sb * sr - sa * cr;
	Trm[0][2] = ca * sb * cr + sa * sr;

	Trm[1][0] = sa * cb;
	Trm[1][1] = sa * sb * sr + ca * cr;
	Trm[1][2] = sa * sb * cr - ca * sr;

	Trm[2][0] = -sb;
	Trm[2][1] = cb * sr;
	Trm[2][2] = cb * cr;


	FX_DOUBLE tr = Trm[0][0] + Trm[1][1] + Trm[2][2];
	FX_DOUBLE q[4];

	if (tr > 0) {
		FX_DOUBLE S = FX_Sqrt(tr + 1.0) * 2; //
		q[3] = 0.25 * S;
		q[0] = (Trm[2][1] - Trm[1][2]) / S;
		q[1] = (Trm[0][2] - Trm[2][0]) / S;
		q[2] = (Trm[1][0] - Trm[0][1]) / S;
	}
	else if ((Trm[0][0] > Trm[1][1]) && (Trm[0][0] > Trm[2][2])) {
		FX_DOUBLE S = FX_Sqrt(1.0 + Trm[0][0] - Trm[1][1] - Trm[2][2]) * 2;
		q[3] = (Trm[2][1] - Trm[1][2]) / S;
		q[0] = 0.25 * S;
		q[1] = (Trm[0][1] + Trm[1][0]) / S;
		q[2] = (Trm[0][2] + Trm[2][0]) / S;
	}
	else if (Trm[1][1] > Trm[2][2]) {
		FX_DOUBLE S = FX_Sqrt(1.0 + Trm[1][1] - Trm[0][0] - Trm[2][2]) * 2;
		q[3] = (Trm[0][2] - Trm[2][0]) / S;
		q[0] = (Trm[0][1] + Trm[1][0]) / S;
		q[1] = 0.25 * S;
		q[2] = (Trm[1][2] + Trm[2][1]) / S;
	}
	else {
		FX_DOUBLE S = FX_Sqrt(1.0 + Trm[2][2] - Trm[0][0] - Trm[1][1]) * 2;
		q[3] = (Trm[1][0] - Trm[0][1]) / S;
		q[0] = (Trm[0][2] + Trm[2][0]) / S;
		q[1] = (Trm[1][2] + Trm[2][1]) / S;
		q[2] = 0.25 * S;
	}
	QuaternionNorm(q);
	Q[0] = q[0];
	Q[1] = q[1];
	Q[2] = q[2];
	Q[3] = q[3];
}

void CAxisPln::QuaternionSlerp(double Q_from[4], double Q_to[4], double ratio, double Q_ret[4])
{
	double omega, cosom, sinom, scale0, scale1;
	cosom = Q_from[0] * Q_to[0] + Q_from[1] * Q_to[1] + Q_from[2] * Q_to[2] + Q_from[3] * Q_to[3];

	if (cosom < 0.0)
	{
		cosom = -cosom;
		Q_to[0] = -Q_to[0];
		Q_to[1] = -Q_to[1];
		Q_to[2] = -Q_to[2];
		Q_to[3] = -Q_to[3];
	}

	if ((1.0 + cosom) > 0.001)
	{
		omega = acos(cosom);
		sinom = sin(omega);
		scale0 = sin((1.0 - ratio) * omega) / sinom;
		scale1 = sin(ratio * omega) / sinom;
	}
	else
	{
		scale0 = 1.0 - ratio;
		scale1 = ratio;
	}
	Q_ret[0] = scale0 * Q_from[0] + scale1 * Q_to[0];
	Q_ret[1] = scale0 * Q_from[1] + scale1 * Q_to[1];
	Q_ret[2] = scale0 * Q_from[2] + scale1 * Q_to[2];
	Q_ret[3] = scale0 * Q_from[3] + scale1 * Q_to[3];
}

void CAxisPln::Quaternions2ABCMatrix(double q[4], double xyz[3], double m[4][4])
{
	double d11, d12, d13, d14, d22, d23, d24, d33, d34;
	d11 = q[0] * q[0];
	d12 = q[0] * q[1];
	d13 = q[0] * q[2];
	d14 = q[0] * q[3];
	d22 = q[1] * q[1];
	d23 = q[1] * q[2];
	d24 = q[1] * q[3];
	d33 = q[2] * q[2];
	d34 = q[2] * q[3];

	m[0][0] = 1 - 2 * d22 - 2 * d33;
	m[0][1] = 2 * (d12 - d34);
	m[0][2] = 2 * (d13 + d24);
	m[0][3] = xyz[0];

	m[1][0] = 2 * (d12 + d34);
	m[1][1] = 1 - 2 * d11 - 2 * d33;
	m[1][2] = 2 * (d23 - d14);
	m[1][3] = xyz[1];

	m[2][0] = 2 * (d13 - d24);
	m[2][1] = 2 * (d23 + d14);
	m[2][2] = 1 - 2 * d11 - 2 * d22;
	m[2][3] = xyz[2];

	m[3][0] = 0;
	m[3][1] = 0;
	m[3][2] = 0;
	m[3][3] = 1;
}

bool CAxisPln::OnMovL(double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, char* path)
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
	ABC2Quaternions(start_pos, Q_start);
	ABC2Quaternions(end_pos, Q_end);
	
	CPointSet out;
	out.OnInit(PotT_9d);
	double tmp[9] = { 0 };
	double ttmp[2] = { 0 };
	for (i = 0; i < max_num; i++)
	{
		double* p = ret[max_num_axis].OnGetPoint(i);
		tmp[max_num_axis] = p[0];

		if ((same_tag[3] + same_tag[4] + same_tag[5]) < 3)
		{
			double ratio = i / (double)(max_num - 1);
			QuaternionSlerp(Q_start, Q_end, ratio, &tmp[3]);
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
						//for (j = step; j < 3; j++)
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
//	char apth[] = "D:\\cccc\\SPMOVL\\OutPVT.txt";
//	char* ppp = apth;
//	out.OnSave(ppp);
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
	
	for (i = 0; i < max_num; i++)
	{
		double* pp = out.OnGetPoint(i);
		tmppoints[0] = pp[0];
		tmppoints[1] = pp[1];
		tmppoints[2] = pp[2];
		tmppoints[3] = pp[3];
		tmppoints[4] = pp[4];
		tmppoints[5] = pp[5];
		tmppoints[6] = pp[6];

		Quaternions2ABCMatrix(&tmppoints[3], &tmppoints[0], TCP);
		for (dof = 0; dof < 4; dof++)
		{
			for (j = 0; j < 4; j++)
			{
				sp.m_Input_IK_TargetTCP[dof][j] = TCP[dof][j];
			}
		}

		FX_Robot_Kine_IK(0, &sp);
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