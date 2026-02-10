
#ifndef _AXISPLN_H_
#define _AXISPLN_H_

#include "PointSet.h"

class CAxisPln
{
public:
	CAxisPln();
	virtual ~CAxisPln();

	bool OnMovL(long RobotSetial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, char* path);
	bool OnMovL(long RobotSetial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, CPointSet * ret_pset);
	bool OnMovJ(long RobotSetial, double start_joint[7], double end_joint[7], double vel, double acc, double jerk, char* path);
	bool OnMovL_KeepJ_Cut(long RobotSerial, double startjoints[7], double stopjoints[7], double vel, double acc, char* path);
	bool OnMovL_KeepJ_CutA(long RobotSerial, double startjoints[7], double stopjoints[7], double vel, double acc, CPointSet * ret_pset);

protected:
	bool OnPln(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet* ret);
	bool OnPlnAcc(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet* ret);
	bool OnPlnAccNew(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet* ret, CPointSet* ret1);
	bool OnPlnAccSimple(double start_pos, double vel, double acc, CPointSet* ret);
	bool InitPln(double s, double v, double a, double j);
	long OnGetPlnNum();
	double OnGetPln(double* ret_v);
	double m_s;
	double m_v;
	double m_a;
	double m_j;
	double m_cur_time;
	double m_time_acc;
	double m_time_dacc;
	double m_time_vel;

	double m_filt_value[500];
	long m_filt_cnt;
	long m_filt_pos;
};

class CMovingAverageFilter
{
public:
	CMovingAverageFilter();
	~CMovingAverageFilter();

	bool FilterPointSet(CPointSet* input, CPointSet* output);
	bool FilterSinglePoint(double** points, long index, long point_count,
		long point_dim, double* filtered_point);

private:
	static const long WINDOW_SIZE = 5; 
};
#endif 
