#include "AxisPln.h"

double _ratio301[301] = {
	0.000000000,
	0.000000200,
	0.000001600,
	0.000005400,
	0.000012800,
	0.000025000,
	0.000043200,
	0.000068600,
	0.000102400,
	0.000145800,
	0.000200000,
	0.000266200,
	0.000345600,
	0.000439400,
	0.000548800,
	0.000675000,
	0.000819200,
	0.000982600,
	0.001166400,
	0.001371800,
	0.001600000,
	0.001852200,
	0.002129600,
	0.002433400,
	0.002764800,
	0.003125000,
	0.003515200,
	0.003936600,
	0.004390400,
	0.004877800,
	0.005400000,
	0.005958200,
	0.006553600,
	0.007187400,
	0.007860800,
	0.008575000,
	0.009331200,
	0.010130600,
	0.010974400,
	0.011863800,
	0.012800000,
	0.013784200,
	0.014817600,
	0.015901400,
	0.017036800,
	0.018225000,
	0.019467200,
	0.020764600,
	0.022118400,
	0.023529800,
	0.025000000,
	0.026530200,
	0.028121600,
	0.029775400,
	0.031492800,
	0.033275000,
	0.035123200,
	0.037038600,
	0.039022400,
	0.041075800,
	0.043200000,
	0.045396200,
	0.047665600,
	0.050009400,
	0.052428800,
	0.054925000,
	0.057499200,
	0.060152593,
	0.062885926,
	0.065699259,
	0.068592593,
	0.071565926,
	0.074619259,
	0.077752593,
	0.080965926,
	0.084259259,
	0.087632593,
	0.091085926,
	0.094619259,
	0.098232593,
	0.101925926,
	0.105699259,
	0.109552593,
	0.113485926,
	0.117499200,
	0.121591667,
	0.125762133,
	0.130009400,
	0.134332267,
	0.138729533,
	0.143200000,
	0.147742467,
	0.152355733,
	0.157038600,
	0.161789867,
	0.166608333,
	0.171492800,
	0.176442067,
	0.181454933,
	0.186530200,
	0.191666667,
	0.196863133,
	0.202118400,
	0.207431267,
	0.212800533,
	0.218225000,
	0.223703467,
	0.229234733,
	0.234817600,
	0.240450867,
	0.246133333,
	0.251863800,
	0.257641067,
	0.263463933,
	0.269331200,
	0.275241667,
	0.281194133,
	0.287187400,
	0.293220267,
	0.299291533,
	0.305400000,
	0.311544467,
	0.317723733,
	0.323936600,
	0.330181867,
	0.336458333,
	0.342764800,
	0.349100067,
	0.355462933,
	0.361852200,
	0.368266667,
	0.374705133,
	0.381166400,
	0.387649267,
	0.394152533,
	0.400675000,
	0.407215467,
	0.413772733,
	0.420345600,
	0.426932867,
	0.433533333,
	0.440145800,
	0.446769067,
	0.453401933,
	0.460043200,
	0.466691667,
	0.473346133,
	0.480005400,
	0.486668267,
	0.493333533,
	0.500000000,
	0.506666467,
	0.513331733,
	0.519994600,
	0.526653867,
	0.533308333,
	0.539956800,
	0.546598067,
	0.553230933,
	0.559854200,
	0.566466667,
	0.573067133,
	0.579654400,
	0.586227267,
	0.592784533,
	0.599325000,
	0.605847467,
	0.612350733,
	0.618833600,
	0.625294867,
	0.631733333,
	0.638147800,
	0.644537067,
	0.650899933,
	0.657235200,
	0.663541667,
	0.669818133,
	0.676063400,
	0.682276267,
	0.688455533,
	0.694600000,
	0.700708467,
	0.706779733,
	0.712812600,
	0.718805867,
	0.724758333,
	0.730668800,
	0.736536067,
	0.742358933,
	0.748136200,
	0.753866667,
	0.759549133,
	0.765182400,
	0.770765267,
	0.776296533,
	0.781775000,
	0.787199467,
	0.792568733,
	0.797881600,
	0.803136867,
	0.808333333,
	0.813469800,
	0.818545067,
	0.823557933,
	0.828507200,
	0.833391667,
	0.838210133,
	0.842961400,
	0.847644267,
	0.852257533,
	0.856800000,
	0.861270467,
	0.865667733,
	0.869990600,
	0.874237867,
	0.878408333,
	0.882500800,
	0.886514074,
	0.890447407,
	0.894300741,
	0.898074074,
	0.901767407,
	0.905380741,
	0.908914074,
	0.912367407,
	0.915740741,
	0.919034074,
	0.922247407,
	0.925380741,
	0.928434074,
	0.931407407,
	0.934300741,
	0.937114074,
	0.939847407,
	0.942500800,
	0.945075000,
	0.947571200,
	0.949990600,
	0.952334400,
	0.954603800,
	0.956800000,
	0.958924200,
	0.960977600,
	0.962961400,
	0.964876800,
	0.966725000,
	0.968507200,
	0.970224600,
	0.971878400,
	0.973469800,
	0.975000000,
	0.976470200,
	0.977881600,
	0.979235400,
	0.980532800,
	0.981775000,
	0.982963200,
	0.984098600,
	0.985182400,
	0.986215800,
	0.987200000,
	0.988136200,
	0.989025600,
	0.989869400,
	0.990668800,
	0.991425000,
	0.992139200,
	0.992812600,
	0.993446400,
	0.994041800,
	0.994600000,
	0.995122200,
	0.995609600,
	0.996063400,
	0.996484800,
	0.996875000,
	0.997235200,
	0.997566600,
	0.997870400,
	0.998147800,
	0.998400000,
	0.998628200,
	0.998833600,
	0.999017400,
	0.999180800,
	0.999325000,
	0.999451200,
	0.999560600,
	0.999654400,
	0.999733800,
	0.999800000,
	0.999854200,
	0.999897600,
	0.999931400,
	0.999956800,
	0.999975000,
	0.999987200,
	0.999994600,
	0.999998400,
	0.999999800,
	1.000000000};

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CAxisPln::CAxisPln()
{
	m_Set_Freq = FX_FALSE;
}

CAxisPln::~CAxisPln()
{
}

void CAxisPln::OnSetFreq(long freq)
{
	const double base_freq = 1000.0; // 1ms base frequency
	double ratio = base_freq / (double)freq;
	double rounded = round(ratio);

	// An improperly set frequency will lead to uneven performance in each control cycle.
	if (fabs(ratio - rounded) < 1e-3)
	{
		m_freq = (double)freq;
		m_cycle = 1 / m_freq;
		m_filt_cnt = (long)(0.1 * m_freq);
	}
	else
	{
		// default 500Hz
		m_cycle = 0.002;
		m_freq = 500.0;
		m_filt_cnt = 50;
	}

	m_Set_Freq = FX_TRUE;
}

void CAxisPln::OnSetPNVA(double joint_pos[7], double joint_neg[7], double vel_lmt[7], double acc_lmt[7])
{
	for (int i = 0; i < 7; i++)
	{
		m_Joint_Pos_Pos[i] = joint_pos[i];
		m_Joint_Pos_Neg[i] = joint_neg[i];
		m_Joint_Vel_Lmt[i] = vel_lmt[i];
		m_Joint_Acc_Lmt[i] = acc_lmt[i];
	}
}

bool CAxisPln::OnPln(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet *ret)
{
	ret->OnInit(PotT_2d);
	ret->OnEmpty();
	double s = fabs(start_pos - end_pos);

	if (s < 0.001)
	{
		double iv[2] = {0};
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

	if (!m_Set_Freq)
	{
		OnSetFreq(500); // default 500Hz
	}

	double fln = m_filt_cnt;
	m_filt_pos = 0;
	long i = 0;
	long j = 0;
	for (j = 0; j < m_filt_cnt; j++)
	{
		m_filt_value[j] = 0;
	}
	long num = OnGetPlnNum();
	double rp = 0.0;
	double rv = 0.0;

	for (i = 0; i < num; i++)
	{
		rp = OnGetPln(&rv);
		m_filt_value[m_filt_pos] = rp;
		m_filt_pos++;
		if (m_filt_pos >= m_filt_cnt)
		{
			m_filt_pos = 0;
		}
		double vv = 0.0;
		for (j = 0; j < m_filt_cnt; j++)
		{
			vv += m_filt_value[j];
		}
		double iv[2] = {0};
		iv[0] = vv / fln;
		iv[1] = 0;
		ret->OnSetPoint(iv);
	}

	// �����ļ���λ�ý���ƽ��
	for (i = 0; i < m_filt_cnt - 1; i++)
	{
		m_filt_value[m_filt_pos] = rp;
		m_filt_pos++;
		if (m_filt_pos >= m_filt_cnt)
		{
			m_filt_pos = 0;
		}
		double vv = 0.0;
		for (j = 0; j < m_filt_cnt; j++)
		{
			vv += m_filt_value[j];
		}
		double iv[2] = {0};
		iv[0] = vv / fln;
		iv[1] = 0;
		ret->OnSetPoint(iv);
	}

	num = ret->OnGetPointNum(); // ƽ����һ����·����

	double sig = 1.0;
	if (end_pos < start_pos)
	{
		sig = -1.0;
	}

	for (i = 0; i < num; i++)
	{
		double *cur = ret->OnGetPoint(i);
		double t = cur[0];
		cur[0] = start_pos + sig * t;
	}

	for (i = 1; i < num - 1; i++)
	{
		double *pre = ret->OnGetPoint(i - 1);
		double *cur = ret->OnGetPoint(i);
		double *nex = ret->OnGetPoint(i + 1);
		cur[1] = (nex[0] - pre[0]) * m_freq * 0.5;
		// cur[1] = (nex[0] - pre[0]) * 250.0;
	}

	return true;
}

bool CAxisPln::OnPlnAcc(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet *ret)
{
	ret->OnInit(PotT_2d);
	ret->OnEmpty();
	double s = fabs(start_pos - end_pos);

	if (s < 0.001)
	{
		double iv[2] = {0};
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
	long i = 0;
	long j = 0;
	for (j = 0; j < m_filt_cnt; j++)
	{
		m_filt_value[j] = 0;
	}
	long num = OnGetPlnNum();
	double rp = 0.0;
	double rv = 0.0;

	for (i = 0; i < num; i++)
	{
		rp = OnGetPln(&rv);
		m_filt_value[m_filt_pos] = rp;
		m_filt_pos++;
		if (m_filt_pos >= m_filt_cnt)
		{
			m_filt_pos = 0;
		}
		double vv = 0.0;
		for (j = 0; j < m_filt_cnt; j++)
		{
			vv += m_filt_value[j];
		}
		double iv[2] = {0};
		iv[0] = vv / fln;
		iv[1] = 0;
		ret->OnSetPoint(iv);
	}
	num = ret->OnGetPointNum(); // ƽ����һ����·����

	double sig = 1.0;
	if (end_pos < start_pos)
	{
		sig = -1.0;
	}

	for (i = 0; i < num; i++)
	{
		double *cur = ret->OnGetPoint(i);
		double t = cur[0];
		cur[0] = start_pos + sig * t;
	}

	for (i = 1; i < num - 1; i++)
	{
		double *pre = ret->OnGetPoint(i - 1);
		double *cur = ret->OnGetPoint(i);
		double *nex = ret->OnGetPoint(i + 1);
		cur[1] = (nex[0] - pre[0]) * 250.0;
	}

	if (num < 2)
	{
		return false;
	}

	double *pathLast = ret->OnGetPoint(num - 2);
	double *pathEnd = ret->OnGetPoint(num - 1);
	int last = (m_time_dacc / 1.5) / 0.002 + 1;
	double pathCon[2] = {0};
	pathCon[0] = pathLast[0];
	pathCon[1] = pathLast[1];
	for (i = 1; i < last; i++)
	{
		double t = i * 0.002;
		pathCon[1] = pathLast[1] + fabs(acc) * (-sig) * t;
		pathCon[0] = pathLast[0] + pathLast[1] * t + 0.5 * fabs(acc) * (-sig) * t * t;
		if (i == 1)
		{
			pathEnd[0] = pathCon[0];
			pathEnd[1] = pathCon[1];
			continue;
		}
		ret->OnSetPoint(pathCon);
	}

	double trajCon[2] = {0};
	for (i = 1; i < last; i++)
	{
		double t = i * 0.002;
		trajCon[1] = pathCon[1] + fabs(acc) * (sig)*t;
		trajCon[0] = pathCon[0] + pathCon[1] * t + 0.5 * fabs(acc) * (sig)*t * t;
		if (trajCon[1] > 0)
		{
			break;
		}
		ret->OnSetPoint(trajCon);
	}

	return true;
}

bool CAxisPln::OnPlnAccNew(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet *ret, CPointSet *ps)
{
	// ��������-����-����-����-����
	ret->OnInit(PotT_2d);
	ret->OnEmpty();
	double s = fabs(start_pos - end_pos);

	if (s < 0.001)
	{
		double iv[2] = {0};
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
	long i = 0;
	long j = 0;
	for (j = 0; j < m_filt_cnt; j++)
	{
		m_filt_value[j] = 0;
	}
	long num = OnGetPlnNum();
	double rp = 0.0;
	double rv = 0.0;

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
		double iv[2] = {0};
		iv[0] = vv / fln;
		iv[1] = 0;
		ret->OnSetPoint(iv);
	}

	num = ret->OnGetPointNum(); // ƽ����һ����·����

	double sig = 1.0;
	if (end_pos < start_pos)
	{
		sig = -1.0;
	}

	for (i = 0; i < num; i++)
	{
		double *cur = ret->OnGetPoint(i);
		double t = cur[0];
		cur[0] = start_pos + sig * t;
	}

	for (i = 1; i < num - 1; i++)
	{
		double *pre = ret->OnGetPoint(i - 1);
		double *cur = ret->OnGetPoint(i);
		double *nex = ret->OnGetPoint(i + 1);
		cur[1] = (nex[0] - pre[0]) * 250.0;
	}

	//--�޸�2
	if (num < 2)
	{
		return false;
	}
	int trajPos = num - m_time_dacc / 0.004;
	for (i = 0; i <= trajPos; i++)
	{
		ps->OnSetPoint(ret->OnGetPoint(i));
	}

	double *pathLast = ret->OnGetPoint(trajPos);

	int last = num - trajPos;
	double pathCon[2] = {0};
	pathCon[0] = pathLast[0];
	pathCon[1] = pathLast[1];
	for (i = 1; i < last * 5; i++)
	{
		double t = i * 0.004;
		pathCon[1] = pathLast[1] + fabs(acc) * (-sig) * t;
		pathCon[0] = pathLast[0] + pathLast[1] * t + 0.5 * fabs(acc) * (-sig) * t * t;

		ps->OnSetPoint(pathCon);
	}

	double trajCon[2] = {0};
	for (i = 1;; i++)
	{
		double t = i * 0.004;
		trajCon[1] = pathCon[1] + fabs(acc) * (sig)*t; // printf("pathCon[1]=%f-----------\n", pathCon[1]);
		trajCon[0] = pathCon[0] + pathCon[1] * t + 0.5 * fabs(acc) * (sig)*t * t;
		if (trajCon[1] > 0)
		{
			break;
		}
		ps->OnSetPoint(trajCon);
	}

	return true;
}

bool CAxisPln::OnPlnAccSimple(double start_pos, double vel, double accmax, CPointSet *axis)
{
	axis->OnInit(PotT_2d);
	axis->OnEmpty();
	double x0 = start_pos;
	double acc = accmax;
	double vmax = vel;
	double v0 = 0;
	double t = 0.002;
	double t2 = t * t;
	double pv[2] = {0};
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
		m_time_acc = acc_t; // printf("a_max-----%f\n", m_v / m_time_acc);
		m_time_dacc = acc_t;
		m_time_vel = (m_s - 2 * acc_s) / m_v;
	}
	else
	{
		m_time_acc = sqrt(m_s / m_a);
		m_time_dacc = m_time_acc;
		m_time_vel = 0;
		m_v = m_time_acc * m_a; // printf("m_v-----%f\n", m_v);
	}
	m_cur_time = 0.0;
	return true;
}

long CAxisPln::OnGetPlnNum()
{
	double t = m_time_acc + m_time_dacc + m_time_vel;
	double t_num = t / m_cycle;
	long ret = t_num + 2;
	return ret;
}

double CAxisPln::OnGetPln(double *ret_v)
{
	// �����з��ص�ǰ�ٶȣ��������ص�ǰλ��
	if (m_cur_time <= m_time_acc)
	{
		double s = 0.5 * m_a * m_cur_time * m_cur_time;
		*ret_v = m_cur_time * m_a;
		m_cur_time += m_cycle;
		return s;
	}
	if (m_cur_time <= (m_time_acc + m_time_vel))
	{
		double s1 = 0.5 * m_a * m_time_acc * m_time_acc;
		double s2 = m_v * (m_cur_time - m_time_acc);
		double s = s1 + s2;
		*ret_v = m_v;
		m_cur_time += m_cycle;
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

		m_cur_time += m_cycle;
		return s;
	}

	*ret_v = 0;
	return m_s;
}

static bool OnMovCheckVA(CPointSet *pset, double cycle, double joint_acc_lmt[7], double joint_vel_lmt[7])
{
	if (pset == NULL || joint_vel_lmt == NULL || joint_acc_lmt == NULL)
	{
		printf("invalid VA limit\n");
		return false;
	}

	long point_num = pset->OnGetPointNum();
	if (point_num < 2 || cycle <= 0.000001)
	{
		printf("invalid path points num\n");
		return false;
	}

	double inv_cycle = 1.0 / cycle;
	double inv_cycle2 = inv_cycle * inv_cycle;
	double prev_vel[7] = {0};

	for (long i = 1; i < point_num; i++)
	{
		double *prev_point = pset->OnGetPoint(i - 1);
		double *cur_point = pset->OnGetPoint(i);
		if (prev_point == NULL || cur_point == NULL)
		{
			printf("path point is NULL\n");
			return false;
		}

		for (long j = 0; j < 7; j++)
		{
			double delta = cur_point[j] - prev_point[j];
			double cur_vel = FX_Fabs(delta) * inv_cycle;

			if (joint_vel_lmt[j] > 0.000001 && cur_vel > joint_vel_lmt[j] + 0.000001)
			{
				printf("[Warning]Velocity exceed limit at point %ld joint %ld: cur_vel= %lf, limit= %lf\n", i+1, j+1, cur_vel, joint_vel_lmt[j]);
				return false;
			}

			if (i >= 2 && joint_acc_lmt[j] > 0.000001)
			{
				double cur_acc = FX_Fabs(delta * inv_cycle2 - prev_vel[j] * inv_cycle);
				if (cur_acc > joint_acc_lmt[j] + 0.000001)
				{
					printf("[Warning]Acceleration exceed limit at point %ld joint %ld: cur_acc= %lf, limit= %lf\n", i+1, j+1, cur_acc, joint_acc_lmt[j]);
					return false;
				}
			}
			prev_vel[j] = delta * inv_cycle;
		}
	}

	return true;
}

bool CAxisPln::OnMovL(long RobotSerial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, char *path)
{
	///////determine same points
	long i = 0;
	long j = 0;
	long same_tag[6] = {0};
	for (i = 0; i < 6; i++)
	{
		if (fabs(end_pos[i] - start_pos[i]) < 0.01)
		{
			same_tag[i] = 1;
		}
	}
	///////Check Max Axis
	CPointSet ret[6];
	long num[3] = {0}; // ret[0].OnGetPointNum();
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

	// Cuter Euler-Angle based on Base_Coordinate
	double Q_start[4] = {0};
	double Q_end[4] = {0};

	FX_ABC2Quaternions(start_pos, Q_start);
	FX_ABC2Quaternions(end_pos, Q_end);

	// Calculate Quaternions Angle
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

	if ((same_tag[3] + same_tag[4] + same_tag[5]) < 3)
	{
		// Cut Quaterniongs PLN
		OnPln(0, qangle, vel, acc, jerk, &ret[3]);
		double qnum = ret[3].OnGetPointNum();

		if (qnum > max_num)
		{
			max_num = qnum;
			max_num_axis = 3;
		}
	}

	CPointSet out;
	out.OnInit(PotT_9d);
	double tmp[9] = {0};

	for (i = 0; i < max_num; i++)
	{
		double *p = ret[max_num_axis].OnGetPoint(i);
		tmp[0] = end_pos[0];
		tmp[1] = end_pos[1];
		tmp[2] = end_pos[2];
		tmp[max_num_axis] = p[0];

		if ((same_tag[3] + same_tag[4] + same_tag[5]) < 3)
		{
			double ratio = 0.0;
			if (max_num_axis == 3)
			{
				ratio = p[0] / qangle;
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

	// set 4 same point
	for (i = 0; i < 4; i++)
	{
		out.OnSetPoint(tmp);
	}

	long dof = 0;
	bool end_tag = false;
	for (dof = 0; dof < 3; dof++)
	{
		if (dof != max_num_axis)
		{
			if (same_tag[dof] == 0)
			{
				double step = (double)(num[dof] - 1) / (max_num + 1);
				long serial = 0;
				double tmpy = 0;
				for (i = 0; i < num[dof] - 3; i += 2)
				{
					double *p1 = ret[dof].OnGetPoint(i);
					double *p2 = ret[dof].OnGetPoint(i + 1);
					double *p3 = ret[dof].OnGetPoint(i + 2);
					double *p4 = ret[dof].OnGetPoint(i + 3);

					double x[4] = {0};
					double y[4] = {0};
					double xpara[10] = {0};
					double retpara[4] = {0};

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
						// for (j = 0; j < 3; j++)
						for (; tmpy < x[3]; tmpy = serial * step)
						{
							double sloy = CO3Polynorm::CalPnY(retpara, tmpy);
							double *p = out.OnGetPoint(serial);

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
							double *p = out.OnGetPoint(serial - k);
							if (p != NULL)
							{
								double r1 = j;
								double r2 = 0.0;
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
							double *p = out.OnGetPoint(serial);

							serial++;
							tmpy += step;
							if (sloy < x[3] && tmpy > x[3])
							{
								end_tag = true;
							}

							if (p != NULL)
							{
								p[dof] = sloy;
							}
						}

						if (end_tag == true)
						{
							double *p = out.OnGetPoint(serial);
							if (p != NULL)
							{
								p[dof] = end_pos[dof];
							}
						}
					}
				}
			}
			else
			{
				for (i = 0; i < max_num; i++)
				{
					double *p = out.OnGetPoint(i);
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

	sp.m_Input_IK_ZSPType = 0;
	for (i = 0; i < 6; i++)
	{
		sp.m_Input_IK_ZSPPara[i] = 0;
	}
	for (i = 0; i < 7; i++)
	{
		sp.m_Input_IK_RefJoint[i] = ref_joints[i];
	}
	sp.m_Input_ZSP_Angle = 0.0;

	////////////////////////////////////////
	CPointSet final_points;
	final_points.OnInit(PotT_9d);
	double tmppoints[7] = {0};
	double TCP[4][4] = {{0}};
	double ret_joints[9] = {0};

	// initial
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			TCP[i][j] = 0;
		}
	}

	for (i = 0; i < final_num; i++)
	{
		double *pp = out.OnGetPoint(i);
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

		if (i == 0)
		{
			for (j = 0; j < 7; j++)
			{
				sp.m_Input_IK_RefJoint[j] = ref_joints[j];
			}
		}
		else
		{
			for (j = 0; j < 7; j++)
			{
				sp.m_Input_IK_RefJoint[j] = ret_joints[j];
			}
		}

		if (FX_Robot_Kine_IK(RobotSerial, &sp) == false)
		{
			return FX_FALSE;
		}

		// Error feedback
		for (long kk = 0; kk < 7; kk++)
		{
			if (sp.m_Output_JntExdTags[kk] == FX_TRUE)
			{
				// printf("Joint %d exceed limit \n", kk);
				return FX_FALSE;
			}
		}

		if (sp.m_Output_IsOutRange == FX_TRUE)
		{
			// printf("Input Position over reachable space\n");
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

	CPointSet output;
	output.OnInit(PotT_7d);
	CMovingAverageFilter filter;
	if (!filter.FilterPointSet(&final_points, &output))
	{
		printf("failed\n");
	}
	output.OnSave(path);

	return true;
}

bool CAxisPln::OnMovL(long RobotSerial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, CPointSet *ret_pset)
{
	///////determine same points
	long i = 0;
	long j = 0;
	long dof = 0;

	///////Check Max Axis
	CPointSet ret[4];
	long num[4] = {0}; // ret[0].OnGetPointNum();
	long max_num = 0;
	long max_num_axis = 0;
	double axis_len[4] = {0};

	for (i = 0; i < 3; i++)
	{
		axis_len[i] = FX_Sqrt((end_pos[i] - start_pos[i]) * (end_pos[i] - start_pos[i]));
		OnPln(0, axis_len[i], vel, acc, jerk, &ret[i]);
		num[i] = ret[i].OnGetPointNum();
		if (num[i] > max_num)
		{
			max_num = num[i];
			max_num_axis = i;
		}
	}

	// Cuter Euler-Angle based on Base_Coordinate
	double Q_start[4] = {0};
	double Q_end[4] = {0};

	FX_ABC2Quaternions(start_pos, Q_start);
	FX_ABC2Quaternions(end_pos, Q_end);

	// Calculate Quaternions Angle
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
	if(FX_Fabs(cosangle) > 1.000)
	{
		cosangle = FX_Fabs(cosangle)/cosangle;
	}

	double qangle = acos(cosangle) * 2 * FXARM_R2D;
	OnPln(0, qangle, vel*0.8, acc*0.8, jerk*0.8, &ret[3]);
	num[3] = ret[3].OnGetPointNum();

	if (num[3] > max_num)
	{
		max_num = num[3];
		max_num_axis = 3;
		axis_len[3] = qangle;
	}

	CPointSet out;
	out.OnInit(PotT_9d);
	double tmp[9] = {0};

	for (i = 0; i < max_num; i++)
	{
		double *p = ret[max_num_axis].OnGetPoint(i);
		double ratio = p[0]/axis_len[max_num_axis];

		for(j = 0;j < 3; j++)
		{
			tmp[j] = start_pos[j] * (1-ratio) + end_pos[j] * ratio;
		}
		FX_QuaternionSlerp(Q_start,Q_end,ratio,&tmp[3]);

		out.OnSetPoint(tmp);
	}

	// set 3 same point
	for (i = 0; i < 3; i++)
	{
		out.OnSetPoint(tmp);
	}
	long final_num = out.OnGetPointNum();

	if(0)
	{
		char* path1 = (char*)"generate_xyzq.txt";
		out.OnSave(path1);
	}

	////////////////////InvKine//////////////
	FX_InvKineSolvePara sp;

	sp.m_Input_IK_ZSPType = 0;
	for (i = 0; i < 6; i++)
	{
		sp.m_Input_IK_ZSPPara[i] = 0;
	}

	for (i = 0; i < 7; i++)
	{
		sp.m_Input_IK_RefJoint[i] = ref_joints[i];
	}
	sp.m_Input_ZSP_Angle = 0.0;

	////////////////////////////////////////
	ret_pset->OnInit(PotT_7d);
	ret_pset->OnEmpty();

	CPointSet output;
	output.OnInit(PotT_7d);

	double tmppoints[7] = {0};
	double TCP[4][4] = {{0}};
	double ret_joints[9] = {0};
	// initial
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			TCP[i][j] = 0;
		}
	}

	for (i = 0; i < final_num; i++)
	{
		double *pp = out.OnGetPoint(i);
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

		if (i == 0)
		{
			for (j = 0; j < 7; j++)
			{
				sp.m_Input_IK_RefJoint[j] = ref_joints[j];
			}
		}
		else
		{
			for (j = 0; j < 7; j++)
			{
				sp.m_Input_IK_RefJoint[j] = ret_joints[j];
			}
		}

		if (FX_Robot_Kine_IK(RobotSerial, &sp) == false)
		{
			return FX_FALSE;
		}

		// Error feedback
		for (int kk = 0; kk < 7; kk++)
		{
			if (sp.m_Output_JntExdTags[kk] == FX_TRUE)
			{
				printf("Joint %d exceed limit \n", kk);
				return FX_FALSE;
			}
		}

		if (sp.m_Output_IsOutRange == FX_TRUE)
		{
			printf("Input Position over reachable space\n");
			return FX_FALSE;
		}

		ret_joints[0] = sp.m_Output_RetJoint[0];
		ret_joints[1] = sp.m_Output_RetJoint[1];
		ret_joints[2] = sp.m_Output_RetJoint[2];
		ret_joints[3] = sp.m_Output_RetJoint[3];
		ret_joints[4] = sp.m_Output_RetJoint[4];
		ret_joints[5] = sp.m_Output_RetJoint[5];
		ret_joints[6] = sp.m_Output_RetJoint[6];

		output.OnSetPoint(ret_joints);
		CMovingAverageFilter filter;
		if (!filter.FilterPointSet(&output, ret_pset))
		{
			printf("failed\n");
		}
		// ret_pset->OnSetPoint(ret_joints);
	}

	if(0)
	{
		char* path2 = (char*)"output_joints.txt";
		ret_pset->OnSave(path2);
	}

	// VA check
	OnMovCheckVA(ret_pset, m_cycle, m_Joint_Acc_Lmt, m_Joint_Vel_Lmt);
	return true;
}

bool CAxisPln::OnMovL_KeepJ_Cut(long RobotSerial, double startjoints[7], double stopjoints[7], double vel, double acc, char *path)
{
	// Final result in retjoints
	CPointSet retJoints;
	FX_INT32L i = 0;
	FX_INT32L j = 0;
	retJoints.OnInit(PotT_9d);
	retJoints.OnEmpty();

	// Pset for XYZ Q NSP
	CPointSet pset;
	pset.OnInit(PotT_40d);

	// XYZ Q NSP
	Matrix4 pg_start = {{0}};
	Matrix4 pg_stop = {{0}};
	Matrix3 nspg_start = {{0}};
	Matrix3 nspg_stop = {{0}};
	FX_Robot_Kine_FK_NSP(RobotSerial, startjoints, pg_start, nspg_start);
	FX_Robot_Kine_FK_NSP(RobotSerial, stopjoints, pg_stop, nspg_stop);

	Quaternion q_start = {0};
	Quaternion q_stop = {0};

	Quaternion q_nsp_start = {0};
	Quaternion q_nsp_stop = {0};

	FX_Matrix2Quaternion4(pg_start, q_start);
	FX_Matrix2Quaternion4(pg_stop, q_stop);

	FX_Matrix2Quaternion3(nspg_start, q_nsp_start);
	FX_Matrix2Quaternion3(nspg_stop, q_nsp_stop);
	/////////////
	///////determine same points:For XYZ
	long same_tag[3] = {0};
	for (i = 0; i < 3; i++)
	{
		if (fabs(pg_start[i][3] - pg_stop[i][3]) < 0.01)
		{
			same_tag[i] = 1;
		}
	}
	///////Check Max Axis
	CPointSet ret[3];
	long num_axis[3] = {0}; // ret[0].OnGetPointNum();
	long max_num = 0;
	long max_num_axis = 0;

	// double acc = vel * 10;
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
		double *p = ret[max_num_axis].OnGetPoint(i);
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

	// set 4 same point
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
				long serial = 0;
				double tmpy = 0;
				for (i = 0; i < num_axis[dof] - 3; i += 2)
				{
					double *p1 = ret[dof].OnGetPoint(i);
					double *p2 = ret[dof].OnGetPoint(i + 1);
					double *p3 = ret[dof].OnGetPoint(i + 2);
					double *p4 = ret[dof].OnGetPoint(i + 3);

					double x[4] = {0};
					double y[4] = {0};
					double xpara[10] = {0};
					double retpara[4] = {0};

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
						// for (j = 0; j < 3; j++)
						for (; tmpy < x[3]; tmpy = serial * step)
						{
							double sloy = CO3Polynorm::CalPnY(retpara, tmpy);
							double *p = pset.OnGetPoint(serial);

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
							double *p = pset.OnGetPoint(serial - k);
							if (p != NULL)
							{
								double r1 = j;
								double r2 = 0.0;
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
							double *p = pset.OnGetPoint(serial);

							serial++;
							tmpy += step;
							if (sloy < x[3] && tmpy > x[3])
							{
								// add last point
								end_tag = true;
							}

							if (p != NULL)
							{
								p[dof] = sloy;
							}
						}

						if (end_tag == true)
						{
							double *p = pset.OnGetPoint(serial);
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
					double *p = pset.OnGetPoint(i);
					if (p != NULL)
					{
						p[dof] = pg_start[dof][3];
					}
				}
			}
		}
	}

	FX_INT32L num = 0;
	num = pset.OnGetPointNum();
	FX_InvKineSolvePara sp;
	sp.m_DGR1 = 10;
	sp.m_DGR2 = 10;
	sp.m_DGR3 = 10;

	double last_joint[7] = {0};
	for (i = 0; i < 7; i++)
	{
		last_joint[i] = startjoints[i];
		sp.m_Input_IK_RefJoint[i] = startjoints[i];
		sp.m_Output_RetJoint[i] = startjoints[i];
	}

	for (i = 0; i < num; i++)
	{
		double *p = pset.OnGetPoint(i);
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
			double cur_ext = sp.m_Output_JntExdABS;
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
					return false;
				}
				dir = -1;
			}
			else
			{

				if (cur_ext < t_ext1)
				{
					return false;
				}
			}

			sp.m_Input_ZSP_Angle = dir;
			while (cur_ext > 0.00001)
			{
				FX_Robot_Kine_IK_NSP(RobotSerial, &sp);
				cur_ext = sp.m_Output_JntExdABS;
				if (FX_Fabs(sp.m_Input_ZSP_Angle) > 360)
				{
					return false;
				}
				sp.m_Input_ZSP_Angle += dir;
			}

			sp.m_Input_ZSP_Angle -= dir;
			p[17] = sp.m_Input_ZSP_Angle;
		}

		for (j = 0; j < 7; j++)
		{
			p[j + 19] = sp.m_Output_RetJoint[j];
		}

		retJoints.OnSetPoint(&p[19]);
	}
	retJoints.OnSave(path);

	return true;
}

bool CAxisPln::OnMovL_KeepJ_CutA(long RobotSerial, double startjoints[7], double stopjoints[7], double vel, double acc, CPointSet *ret_pset)
{

	FX_INT32L i = 0;
	FX_INT32L j = 0;
	ret_pset->OnInit(PotT_9d);
	ret_pset->OnEmpty();

	// Pset for XYZ Q NSP
	CPointSet pset;
	pset.OnInit(PotT_40d);

	// XYZ Q NSP
	Matrix4 pg_start = {{0}};
	Matrix4 pg_stop = {{0}};
	Matrix3 nspg_start = {{0}};
	Matrix3 nspg_stop = {{0}};
	FX_Robot_Kine_FK_NSP(RobotSerial, startjoints, pg_start, nspg_start);
	FX_Robot_Kine_FK_NSP(RobotSerial, stopjoints, pg_stop, nspg_stop);

	Quaternion q_start = {0};
	Quaternion q_stop = {0};

	Quaternion q_nsp_start = {0};
	Quaternion q_nsp_stop = {0};

	FX_Matrix2Quaternion4(pg_start, q_start);
	FX_Matrix2Quaternion4(pg_stop, q_stop);

	FX_Matrix2Quaternion3(nspg_start, q_nsp_start);
	FX_Matrix2Quaternion3(nspg_stop, q_nsp_stop);
	/////////////
	///////determine same points:For XYZ
	long same_tag[3] = {0};
	for (i = 0; i < 3; i++)
	{
		if (fabs(pg_start[i][3] - pg_stop[i][3]) < 0.01)
		{
			same_tag[i] = 1;
		}
	}
	///////Check Max Axis
	CPointSet ret[3];
	long num_axis[3] = {0}; // ret[0].OnGetPointNum();
	long max_num = 0;
	long max_num_axis = 0;

	// double acc = vel * 10;
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
		double *p = ret[max_num_axis].OnGetPoint(i);
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

	// set 4 same point
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
				long serial = 0;
				double tmpy = 0;
				for (i = 0; i < num_axis[dof] - 3; i += 2)
				{
					double *p1 = ret[dof].OnGetPoint(i);
					double *p2 = ret[dof].OnGetPoint(i + 1);
					double *p3 = ret[dof].OnGetPoint(i + 2);
					double *p4 = ret[dof].OnGetPoint(i + 3);

					double x[4] = {0};
					double y[4] = {0};
					double xpara[10] = {0};
					double retpara[4] = {0};

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
						// for (j = 0; j < 3; j++)
						for (; tmpy < x[3]; tmpy = serial * step)
						{
							double sloy = CO3Polynorm::CalPnY(retpara, tmpy);
							double *p = pset.OnGetPoint(serial);

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
							double *p = pset.OnGetPoint(serial - k);
							if (p != NULL)
							{
								double r1 = j;
								double r2 = 0.0;
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
							double *p = pset.OnGetPoint(serial);

							serial++;
							tmpy += step;
							if (sloy < x[3] && tmpy > x[3])
							{
								// add last point
								end_tag = true;
							}

							if (p != NULL)
							{
								p[dof] = sloy;
							}
						}

						if (end_tag == true)
						{
							double *p = pset.OnGetPoint(serial);
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
					double *p = pset.OnGetPoint(i);
					if (p != NULL)
					{
						p[dof] = pg_start[dof][3];
					}
				}
			}
		}
	}

	FX_INT32L num = 0;
	num = pset.OnGetPointNum();
	FX_InvKineSolvePara sp;
	sp.m_DGR1 = 10;
	sp.m_DGR2 = 10;
	sp.m_DGR3 = 10;

	double last_joint[7] = {0};
	for (i = 0; i < 7; i++)
	{
		last_joint[i] = startjoints[i];
		sp.m_Input_IK_RefJoint[i] = startjoints[i];
		sp.m_Output_RetJoint[i] = startjoints[i];
	}

	for (i = 0; i < num; i++)
	{
		double *p = pset.OnGetPoint(i);
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
			double cur_ext = sp.m_Output_JntExdABS;
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
					return false;
				}
				dir = -1;
			}
			else
			{

				if (cur_ext < t_ext1)
				{
					return false;
				}
			}

			sp.m_Input_ZSP_Angle = dir;
			while (cur_ext > 0.00001)
			{
				FX_Robot_Kine_IK_NSP(RobotSerial, &sp);
				cur_ext = sp.m_Output_JntExdABS;
				if (FX_Fabs(sp.m_Input_ZSP_Angle) > 360)
				{
					return false;
				}
				sp.m_Input_ZSP_Angle += dir;
			}

			sp.m_Input_ZSP_Angle -= dir;
			p[17] = sp.m_Input_ZSP_Angle;
		}

		for (j = 0; j < 7; j++)
		{
			p[j + 19] = sp.m_Output_RetJoint[j];
		}

		ret_pset->OnSetPoint(&p[19]);
	}

	// VA check
	OnMovCheckVA(ret_pset, m_cycle, m_Joint_Acc_Lmt, m_Joint_Vel_Lmt);

	return true;
}

bool CAxisPln::OnMovJ(long RobotSerial, double start_joint[7], double end_joint[7], double vel, double acc, double jerk, char *path)
{
	///////determine same joints
	long i = 0;
	long j = 0;
	long same_tag[7] = {0};

	for (i = 0; i < 7; i++)
	{
		if (fabs(end_joint[i] - start_joint[i]) < 0.01)
		{
			same_tag[i] = 1;
		}
	}

	CPointSet ret[7];
	long num[7] = {0};
	long max_num = 0;
	for (i = 0; i < 7; i++)
	{
		if (!same_tag[i])
		{
			OnPln(start_joint[i], end_joint[i], vel, acc, jerk, &ret[i]);
			num[i] = ret[i].OnGetPointNum();
			if (num[i] > max_num)
			{
				max_num = num[i];
			}
		}
	}

	CPointSet final_ret;
	final_ret.OnInit(PotT_9d);
	double out_joints[9] = {0};
	for (i = 0; i < max_num; i++)
	{
		for (j = 0; j < 7; j++)
		{
			if (i < num[j])
			{
				double *p = ret[j].OnGetPoint(i);
				out_joints[j] = p[0];
			}
			else
			{
				out_joints[j] = end_joint[j];
			}
		}
		final_ret.OnSetPoint(out_joints);
	}
	if (final_ret.OnSave(path) == false)
	{
		printf("num= %ld false\n", final_ret.OnGetPointNum());
	}

	return true;
}

static bool OnPointSetCopy(CPointSet *src, CPointSet *dst)
{
	if (src == NULL || dst == NULL)
	{
		return false;
	}

	dst->OnInit(PotT_7d);
	dst->OnEmpty();
	long point_num = src->OnGetPointNum();
	for (long i = 0; i < point_num; i++)
	{
		double *src_point = src->OnGetPoint(i);
		double point[7] = {0};
		if (src_point == NULL)
		{
			return false;
		}
		for (long j = 0; j < 7; j++)
		{
			point[j] = src_point[j];
		}
		dst->OnSetPoint(point);
	}

	return point_num > 0;
}

static double OnMapLiner2ratio(double value)
{
	if (value <= 0.0) return 0.0;
    if (value >= 1000.0) return 1.0;
    return value * 0.001;
}

bool CAxisPln::OnMovTarget(long RobotSerial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, CPointSet *ret_pset)
{
	if (ref_joints == NULL || start_pos == NULL || end_pos == NULL || ret_pset == NULL)
	{
		return false;
	}

	if (!m_Set_Freq)
	{
		OnSetFreq(500);
	}

	ret_pset->OnInit(PotT_7d);
	ret_pset->OnEmpty();

	// Stage 1: Try to solve MOVL directly, if it works, return the result
	CPointSet movl_pset;
	if (OnMovL(RobotSerial, ref_joints, start_pos, end_pos, vel, acc, jerk, &movl_pset) && OnMovCheckVA(&movl_pset, m_cycle, m_Joint_Acc_Lmt, m_Joint_Vel_Lmt))
	{
		return OnPointSetCopy(&movl_pset, ret_pset);
	}
	else
	{
		printf("[warning]OnMovL failed, try another method.\n");
	}

	// Stage 2: Try to solve MOVL-KeppJ, if it works, return the result
	// Solve End_Joints according to end_pos
	Vect7 end_joint = {0};
	FX_InvKineSolvePara sp_ = {0};

	sp_.m_DGR1 = 0.5;
	sp_.m_Input_IK_ZSPType = 0;
	for (long i = 0; i < 6; i++)
	{
		sp_.m_Input_IK_ZSPPara[i] = 0.0;
	}
	for (long i = 0; i < 7; i++)
	{
		sp_.m_Input_IK_RefJoint[i] = ref_joints[i];
	}
	FX_XYZABC2Matrix4DEG(end_pos, sp_.m_Input_IK_TargetTCP);

	if (FX_Robot_Kine_IK(RobotSerial, &sp_) == FX_FALSE)
	{
		printf("End Posture is out of robot's workspace. Please check the input.\n");
		return false;
	}
	if (sp_.m_Output_IsJntExd == FX_TRUE)
	{
		printf("Invalid Input. No solution for MOVL. \n");
		return false;
	}

	for (long i = 0; i < 7; i++)
	{
		end_joint[i] = sp_.m_Output_RetJoint[i];
	}

	// try Movl-keepJ
	CPointSet keepj_raw;
	CPointSet keepj_pset;
	if (OnMovL_KeepJ_CutA(RobotSerial, ref_joints, end_joint, vel, acc, &keepj_raw) && OnMovCheckVA(&keepj_raw, m_cycle, m_Joint_Acc_Lmt, m_Joint_Vel_Lmt))
	{
		return OnPointSetCopy(&keepj_raw, ret_pset);
	}
	else
	{
		printf("[warning]OnMovL_KeepJ_CutA failed, motion palnning in joint space and can not keep linear plan.\n");
	}

	// Stage 3: Motion Planning in Joint Space
	CAxisJointPln joint_pln;
	double vel_ratio = OnMapLiner2ratio(vel);
	double acc_ratio = OnMapLiner2ratio(acc);

	if (joint_pln.OnSetLmt(7, m_Joint_Pos_Neg, m_Joint_Pos_Pos, m_Joint_Vel_Lmt, m_Joint_Acc_Lmt) == FX_FALSE)
	{
		return false;
	}

	joint_pln.OnSetFreq((FX_INT32)(m_freq + 0.5));

	return joint_pln.OnMovJoint((FX_INT32)RobotSerial, ref_joints, end_joint, vel_ratio, acc_ratio, ret_pset) == FX_TRUE;
}

// Multi-Point Motion Planning
bool CAxisPln::OnInit_MOVL_ZSP()
{
	// Intern Parameter Initialization
	m_output_pset.OnEmpty();
	m_output_pset.OnInit(PotT_7d);

	Overlap_Tag = false;
	Overlap_Num = 0;

	for (long i = 0; i < 6; i++)
	{
		next_start_pos[i] = 0.0;
		last_jv[i] = 0.0;
	}
	last_jv[6] = 0.0;
	return true;
}

bool CAxisPln::OnGetRatioByCntScale(long total_cnt, long cur_cnt, double &ratio1, double &ratio2)
{
	long tcnt = total_cnt;
	long ccnt = cur_cnt;
	ratio1 = 0.5;
	ratio2 = 0.5;
	if (tcnt < 2)
	{
		return false;
	}
	if (ccnt < 0)
	{
		ccnt = 0;
	}
	if (ccnt > tcnt)
	{
		ccnt = tcnt;
	}

	double dcnt = ccnt;
	double dtcnt = tcnt;

	double r = dcnt / dtcnt;
	r *= 300;
	long rl = r;
	if (rl >= 300)
	{
		ratio1 = 0.0;
		ratio2 = 1.0;
		return true;
	}

	double relic = r - rl;

	if (rl >= 300)
	{
		ratio1 = 0.0;
		ratio2 = 1.0;
		return true;
	}

	double t2 = relic;
	double t1 = 1.0 - relic;
	ratio2 = _ratio301[rl] * t1 + _ratio301[rl + 1] * t2;
	ratio1 = 1.0 - ratio2;
	return true;
}

bool CAxisPln::OnMovL_ZSP(long RobotSerial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, long ZSP_type, double ZSP_para[6], double Allow_Range, long Point_State)
{
	///////Calculate composite axis motion length
	long i = 0;
	long j = 0;
	CPointSet com_axis;

	// XYZ Offset
	double com_axis_len_ = 0.0;
	double xyz_len_square_ = 0.0;
	double xyz_len_ = 0.0;
	double diff = 0.0;

	// Record the start point of current path, which is used to calculate the length of current path
	if (Point_State != FX_MOVL_START)
	{
		for (i = 0; i < 6; i++)
		{
			start_pos[i] = next_start_pos[i];
			ref_joints[i] = last_jv[i];
		}
		ref_joints[i] = last_jv[i];
	}

	// Record the end point for next path
	for (i = 0; i < 6; i++)
	{
		next_start_pos[i] = end_pos[i];
	}

	for (i = 0; i < 3; i++)
	{
		diff = end_pos[i] - start_pos[i];
		xyz_len_square_ += diff * diff;
	}
	xyz_len_ = sqrt(xyz_len_square_);

	// Cuter Euler-Angle based on Base_Coordinate
	double Q_start[4] = {0};
	double Q_end[4] = {0};

	FX_ABC2Quaternions(start_pos, Q_start);
	FX_ABC2Quaternions(end_pos, Q_end);

	// Calculate Quaternions Angle
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

	if (fabs(cosangle) > 1.0)
	{
		cosangle /= fabs(cosangle);
	}
	double qangle = acos(cosangle) * 2 * FXARM_R2D;

	com_axis_len_ = sqrt(xyz_len_square_ + qangle * qangle);

	OnPln(0, com_axis_len_, vel, acc, jerk, &com_axis);

	long num_points = com_axis.OnGetPointNum();

	// Cut Cartesian trajectory
	CPointSet cartesian_traj;
	long overlap_num = 0;
	cartesian_traj.OnInit(PotT_8d); // Position & Quaternion
	cartesian_traj.OnEmpty();

	double pose[8] = {0};
	for (i = 0; i < num_points; i++)
	{
		double *vel_data = com_axis.OnGetPoint(i);
		double ratio = vel_data[0] / com_axis_len_;

		// Record the number of overlap points & Overlap Tag
		diff = fabs(vel_data[0] - xyz_len_);
		if (diff < Allow_Range)
		{
			if (!Overlap_Tag)
			{
				Overlap_Tag = true;
			}
			pose[7] = 1.0;
			overlap_num++;
		}

		for (j = 0; j < 3; j++)
		{
			pose[j] = start_pos[j] * (1 - ratio) + end_pos[j] * ratio;
		}

		FX_QuaternionSlerp(Q_start, Q_end, ratio, &pose[3]);
		cartesian_traj.OnSetPoint(pose);
	}

	long final_num = cartesian_traj.OnGetPointNum();
	////////////////////InvKine//////////////
	FX_InvKineSolvePara sp;

	sp.m_Input_IK_ZSPType = ZSP_type;
	for (i = 0; i < 6; i++)
	{
		sp.m_Input_IK_ZSPPara[i] = ZSP_para[i];
	}
	for (i = 0; i < 7; i++)
	{
		sp.m_Input_IK_RefJoint[i] = ref_joints[i];
	}
	sp.m_Input_ZSP_Angle = 0.0;

	////////////////////////////////////////
	CPointSet output;
	output.OnInit(PotT_8d);

	CPointSet tmp_out;
	tmp_out.OnInit(PotT_8d);

	double tmppoints[8] = {0};
	double TCP[4][4] = {{0}};
	double ret_joints[9] = {0};
	// initial
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			TCP[i][j] = 0;
		}
	}

	for (i = 0; i < final_num; i++)
	{
		double *pp = cartesian_traj.OnGetPoint(i);
		tmppoints[0] = pp[0];
		tmppoints[1] = pp[1];
		tmppoints[2] = pp[2];
		tmppoints[3] = pp[3];
		tmppoints[4] = pp[4];
		tmppoints[5] = pp[5];
		tmppoints[6] = pp[6];
		tmppoints[7] = pp[7];

		FX_Quaternions2ABCMatrix(&tmppoints[3], &tmppoints[0], TCP);
		for (long dof = 0; dof < 4; dof++)
		{
			for (j = 0; j < 4; j++)
			{
				sp.m_Input_IK_TargetTCP[dof][j] = TCP[dof][j];
			}
		}

		if (i == 0)
		{
			for (j = 0; j < 7; j++)
			{
				sp.m_Input_IK_RefJoint[j] = ref_joints[j];
			}
		}
		else
		{
			for (j = 0; j < 7; j++)
			{
				sp.m_Input_IK_RefJoint[j] = ret_joints[j];
			}
		}

		if (FX_Robot_Kine_IK(RobotSerial, &sp) == false)
		{
			return FX_FALSE;
		}

		// Error feedback
		for (long kk = 0; kk < 7; kk++)
		{
			if (sp.m_Output_JntExdTags[kk] == FX_TRUE)
			{
				printf("Joint %ld exceed limit \n", kk);
				return FX_FALSE;
			}
		}

		if (sp.m_Output_IsOutRange == FX_TRUE)
		{
			printf("Input Position over reachable space\n");
			return FX_FALSE;
		}

		ret_joints[0] = sp.m_Output_RetJoint[0];
		ret_joints[1] = sp.m_Output_RetJoint[1];
		ret_joints[2] = sp.m_Output_RetJoint[2];
		ret_joints[3] = sp.m_Output_RetJoint[3];
		ret_joints[4] = sp.m_Output_RetJoint[4];
		ret_joints[5] = sp.m_Output_RetJoint[5];
		ret_joints[6] = sp.m_Output_RetJoint[6];

		output.OnSetPoint(ret_joints);
		CMovingAverageFilter filter;
		if (!filter.FilterPointSet(&output, &tmp_out))
		{
			printf("failed\n");
		}
	}

	long tmp_out_num = tmp_out.OnGetPointNum();

	// Save last jv
	double *last_jv_tmp = tmp_out.OnGetPoint(tmp_out_num - 1);
	last_jv[0] = last_jv_tmp[0];
	last_jv[1] = last_jv_tmp[1];
	last_jv[2] = last_jv_tmp[2];
	last_jv[3] = last_jv_tmp[3];
	last_jv[4] = last_jv_tmp[4];
	last_jv[5] = last_jv_tmp[5];
	last_jv[6] = last_jv_tmp[6];

	if (Point_State == FX_MOVL_START)
	{
		for (long ii = 0; ii < tmp_out_num; ii++)
		{
			double *p = tmp_out.OnGetPoint(ii);
			m_output_pset.OnSetPoint(p);
		}
	}
	else
	{
		if (Overlap_Tag && tmp_out_num > Overlap_Num)
		{
			// Deal with Overlap Part
			double r1 = 0.0;
			double r2 = 0.0;
			long num1 = m_output_pset.OnGetPointNum();
			for (i = 0; i <= Overlap_Num; i++)
			{
				OnGetRatioByCntScale(Overlap_Num, i, r1, r2);
				double *p1 = m_output_pset.OnGetPoint(num1 + i - 1 - Overlap_Num);
				double *p2 = tmp_out.OnGetPoint(i);
				for (long kk = 0; kk < 7; kk++)
				{
					p1[kk] = p1[kk] * r1 + p2[kk] * r2;
				}
				// printf("r1=%f  r2=%f\n", r1, r2);
			}
			for (; i < tmp_out_num; i++)
			{
				double *p = tmp_out.OnGetPoint(i);
				m_output_pset.OnSetPoint(p);
			}
		}
		else
		{
			for (long ii = 0; ii < tmp_out_num; ii++)
			{
				double *p = tmp_out.OnGetPoint(ii);
				m_output_pset.OnSetPoint(p);
			}
		}
	}

	Overlap_Num = overlap_num;

	return true;
}

bool CAxisPln::OnSendPoints(CPointSet *out)
{
	long num = m_output_pset.OnGetPointNum();
	out->OnEmpty();
	out->OnInit(PotT_7d);

	for (long i = 0; i < num; i++)
	{
		double *p = m_output_pset.OnGetPoint(i);
		out->OnSetPoint(p);
	}

	// VA check
	OnMovCheckVA(out, m_cycle, m_Joint_Acc_Lmt, m_Joint_Vel_Lmt);
	return true;
}

/////////////////////////////////////////////
CAxisJointPln::CAxisJointPln()
{
	m_dof = 0;
	m_ts = 0.02;
}

CAxisJointPln::~CAxisJointPln()
{
}

FX_BOOL CAxisJointPln::OnMovJoint(FX_INT32 RobotSerial, Vect7 start_joint, Vect7 end_joint, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio, CPointSet *ret_pset)
{
	FX_INT32 i = 0;
	FX_DOUBLE vr = vel_ratio;
	FX_DOUBLE ar = acc_ratio;
	if (vr < 0.01)
		vr = 0.01;
	if (vr > 1.0)
		vr = 1.0;
	if (ar < 0.01)
		ar = 0.01;
	if (ar > 1.0)
		ar = 1.0;

	Vect8 sta = {0};
	Vect8 sto = {0};
	for (i = 0; i < 7; i++)
	{
		sta[i] = start_joint[i];
		sto[i] = end_joint[i];
	}
	FX_INT32 num = OnPln(sta, sto, vr, ar);
	if (num <= 0)
	{
		return FX_FALSE;
	}

	Vect8 retp = {0};
	ret_pset->OnInit(PotT_7d);
	ret_pset->OnEmpty();

	for (i = 0; i < num; i++)
	{
		OnCut(retp);
		// Joint Limit Check
		for (FX_INT32 j = 0; j < 7; j++)
		{
			if (retp[j] < m_PosNeg[j] || retp[j] > m_PosPos[j])
			{
				printf("OnMovJoint: Joint %d exceeds limit", j + 1);
				return FX_FALSE;
			}
		}
		ret_pset->OnSetPoint(retp);
	}
	return FX_TRUE;
}

FX_BOOL CAxisJointPln::OnSetLmt(FX_INT32 dof, Vect8 PosNeg, Vect8 PosPos, Vect8 VelLmt, Vect8 AccLmt)
{
	if (dof <= 0 || m_dof > 8)
	{
		printf("OnSetLmt: Invalid DOF value: %d", dof);
		return FX_FALSE;
	}
	m_dof = dof;
	for (FX_INT32 i = 0; i < dof; i++)
	{
		m_PosNeg[i] = PosNeg[i];
		m_PosPos[i] = PosPos[i];
		m_VelLmt[i] = VelLmt[i];
		m_AccLmt[i] = AccLmt[i];
	}

	return FX_TRUE;
}

FX_VOID CAxisJointPln::OnSetFreq(FX_INT32 freq)
{
	const FX_DOUBLE base_freq = 1000.0;
	FX_DOUBLE ratio = base_freq / (FX_DOUBLE)freq;
	FX_DOUBLE rounded = round(ratio);

	// An improperly set frequency will lead to uneven performance in each control cycle.
	if (fabs(ratio - rounded) < 1e-3)
	{
		FX_DOUBLE freq_double_ = (FX_DOUBLE)freq;
		m_ts = 1 / freq_double_;
	}
	else
	{
		// default 500Hz
		m_ts = 0.02;
	}
}

FX_INT32 CAxisJointPln::OnPln(Vect8 startp, Vect8 stopp, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio)
{
	if (m_dof <= 0)
	{
		return -1;
	}
	{
		FX_INT32 i;
		for (i = 0; i < m_dof; i++)
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
	FX_INT32 i;

	FX_DOUBLE vr = vel_ratio;
	if (vr < 0.01)
	{
		vr = 0.01;
	}
	if (vr > 1)
	{
		vr = 1;
	}

	FX_DOUBLE ar = vel_ratio;
	if (acc_ratio < 0.01)
	{
		acc_ratio = 0.01;
	}
	if (acc_ratio > 1)
	{
		acc_ratio = 1;
	}

	FX_DOUBLE t_max = 0;
	FX_INT32 t_max_axis_num = -1;
	for (i = 0; i < m_dof; i++)
	{
		m_start[i] = startp[i];
		m_stop[i] = stopp[i];
		FX_DOUBLE dsp = startp[i] - stopp[i];

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
			FX_DOUBLE v = m_VelLmt[i] * vr;
			FX_DOUBLE a = m_AccLmt[i] * ar;
			FX_DOUBLE sa = 0.5 * v * v / a;

			if (sa * 2.0 >= dsp)
			{
				m_Pln_Type[i] = 1;
				FX_DOUBLE t1 = sqrt(dsp / a);
				FX_DOUBLE t = 2 * t1;

				m_Pln_T[i] = t;
				FX_DOUBLE v = t1 * a;
				m_Pln_P1[i][0] = 0;
				m_Pln_P1[i][1] = 0;
				m_Pln_P1[i][2] = a;
				m_Pln_P1[i][3] = t1;

				m_Pln_P3[i][0] = dsp * 0.5;
				m_Pln_P3[i][1] = v;
				m_Pln_P3[i][2] = -a;
				m_Pln_P3[i][3] = t;

				if (t >= t_max)
				{
					t_max = t;
					t_max_axis_num = i;
				}
			}
			else
			{
				m_Pln_Type[i] = 2;
				FX_DOUBLE t1 = v / a;
				FX_DOUBLE t2 = (dsp - 2 * sa) / v;
				FX_DOUBLE t = 2 * t1 + t2;

				m_Pln_T[i] = t;
				m_Pln_P1[i][0] = 0;
				m_Pln_P1[i][1] = 0;
				m_Pln_P1[i][2] = a;
				m_Pln_P1[i][3] = t1;

				m_Pln_P2[i][0] = sa;
				m_Pln_P2[i][1] = v;
				m_Pln_P2[i][2] = 0;
				m_Pln_P2[i][3] = t1 + t2;

				m_Pln_P3[i][0] = dsp - sa;
				m_Pln_P3[i][1] = v;
				m_Pln_P3[i][2] = -a;
				m_Pln_P3[i][3] = t;

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

	for (i = 0; i < m_dof; i++)
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

	m_FristTag = FX_TRUE;
	return t_max / m_ts + 6;
}

FX_BOOL CAxisJointPln::OnCut(Vect8 retp)
{
	FX_INT32 i;
	FX_INT32 j;
	for (i = 0; i < m_dof; i++)
	{
		FX_DOUBLE cut_t = m_cur_t;
		if (m_Pln_Type[i] == 0)
		{
			FX_DOUBLE r = cut_t / m_totl_t;
			retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
		}
		if (m_Pln_Type[i] == 1)
		{
			cut_t *= m_Pln_TRatio[i];

			if (cut_t < m_Pln_P1[i][3])
			{
				FX_DOUBLE len = 0.5 * m_Pln_P1[i][2] * cut_t * cut_t;
				FX_DOUBLE r = len / m_Pln_Len[i];
				retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
			}
			else
			{
				cut_t -= m_Pln_P1[i][3];
				FX_DOUBLE len = m_Pln_P3[i][0] + m_Pln_P3[i][1] * cut_t + 0.5 * m_Pln_P3[i][2] * cut_t * cut_t;

				FX_DOUBLE r = len / m_Pln_Len[i];
				retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
			}
		}
		if (m_Pln_Type[i] == 2)
		{
			cut_t *= m_Pln_TRatio[i];

			if (cut_t < m_Pln_P1[i][3])
			{
				FX_DOUBLE len = 0.5 * m_Pln_P1[i][2] * cut_t * cut_t;
				FX_DOUBLE r = len / m_Pln_Len[i];
				retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
				// printf("r1 %lf ",r);
			}
			else
			{
				if (cut_t < m_Pln_P2[i][3])
				{
					cut_t -= m_Pln_P1[i][3];
					FX_DOUBLE len = m_Pln_P2[i][0] + m_Pln_P2[i][1] * cut_t;
					FX_DOUBLE r = len / m_Pln_Len[i];
					retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
					// printf("r2 %lf ", r);
				}
				else
				{
					cut_t -= m_Pln_P2[i][3];
					FX_DOUBLE len = m_Pln_P3[i][0] + m_Pln_P3[i][1] * cut_t + 0.5 * m_Pln_P3[i][2] * cut_t * cut_t;
					FX_DOUBLE r = len / m_Pln_Len[i];
					retp[i] = r * m_stop[i] + (1.0 - r) * m_start[i];
					// printf("r3 %lf ", r);
				}
			}
		}
	}

	m_cur_t += m_ts;
	if (m_cur_t >= m_totl_t)
	{
		m_cur_t = m_totl_t;
	}

	if (m_FristTag == FX_TRUE)
	{
		m_FristTag = FX_FALSE;
		m_wpos = 0;
		for (i = 0; i < m_dof; i++)
		{
			for (j = 0; j < 5; j++)
			{
				m_value[i][j] = retp[i];
			}
		}
		return FX_TRUE;
	}
	else
	{
		for (i = 0; i < m_dof; i++)
		{
			m_value[i][m_wpos] = retp[i];
			retp[i] = 0;
			for (j = 0; j < 5; j++)
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

	return FX_TRUE;
}

/////////////////////////////////////////////

CMovingAverageFilter::CMovingAverageFilter()
{
}

CMovingAverageFilter::~CMovingAverageFilter()
{
}

bool CMovingAverageFilter::FilterPointSet(CPointSet *input, CPointSet *output)
{
	if (input == NULL || output == NULL)
	{
		return false;
	}

	long point_count = input->OnGetPointNum();
	if (point_count < WINDOW_SIZE)
	{
		// 如果点数少于窗口大小，直接复�?		output->OnEmpty();
		for (long i = 0; i < point_count; i++)
		{
			double *p = input->OnGetPoint(i);
			if (p != NULL)
			{
				output->OnSetPoint(p);
			}
		}
		return true;
	}

	// 获取点的维数
	double *first_point = input->OnGetPoint(0);
	if (first_point == NULL)
	{
		return false;
	}

	output->OnEmpty();

	// 对每个点进行滤波处理
	for (long i = 0; i < point_count; i++)
	{
		long start_idx = i - WINDOW_SIZE / 2;		// 窗口起始索引
		long end_idx = start_idx + WINDOW_SIZE - 1; // 窗口结束索引

		// 边界处理：确保窗口不超出数组范围
		if (start_idx < 0)
		{
			start_idx = 0;
			end_idx = WINDOW_SIZE - 1;
		}
		if (end_idx >= point_count)
		{
			end_idx = point_count - 1;
			start_idx = end_idx - WINDOW_SIZE + 1;
			if (start_idx < 0)
			{
				start_idx = 0;
			}
		}

		// 获取窗口内所有点
		double *p = input->OnGetPoint(i);
		if (p == NULL)
		{
			return false;
		}

		// 初始化滤波后的点
		double filtered[7] = {0};
		long window_count = end_idx - start_idx + 1; // 实际窗口大小

		long dim = 7; // 默认�?维（关节�?
		for (long d = 0; d < dim; d++)
		{
			double sum = 0.0;
			for (long j = start_idx; j <= end_idx; j++)
			{
				double *pj = input->OnGetPoint(j);
				if (pj != NULL)
				{
					sum += pj[d];
				}
			}
			filtered[d] = sum / window_count;
		}

		output->OnSetPoint(filtered);
	}

	return true;
}

bool CMovingAverageFilter::FilterSinglePoint(double **points, long index,
											 long point_count, long point_dim,
											 double *filtered_point)
{
	if (points == NULL || filtered_point == NULL || point_count < WINDOW_SIZE)
	{
		return false;
	}

	long start_idx = index - WINDOW_SIZE / 2;
	long end_idx = start_idx + WINDOW_SIZE - 1;

	// 边界处理
	if (start_idx < 0)
	{
		start_idx = 0;
		end_idx = WINDOW_SIZE - 1;
	}
	if (end_idx >= point_count)
	{
		end_idx = point_count - 1;
		start_idx = end_idx - WINDOW_SIZE + 1;
		if (start_idx < 0)
		{
			start_idx = 0;
		}
	}

	long window_count = end_idx - start_idx + 1;

	// dimensions moving-average filter
	for (long d = 0; d < point_dim; d++)
	{
		double sum = 0.0;
		for (long j = start_idx; j <= end_idx; j++)
		{
			sum += points[j][d];
		}
		filtered_point[d] = sum / window_count;
	}

	return true;
}
