
#ifndef _AXISPLN_H_
#define _AXISPLN_H_

#include "PointSet.h"
#include "math.h"
#include "O3Polynorm.h"
#include "FxRobot.h"
#include "FXMatrix.h"

class CAxisPln
{
public:
	CAxisPln();
	virtual ~CAxisPln();

	void OnSetFreq(long freq);
	void OnSetPNVA(double joint_pos[7], double joint_neg[7], double vel_lmt[7], double acc_lmt[7]);

	bool OnMovL(long RobotSerial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, char *path);
	bool OnMovL(long RobotSerial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, CPointSet *ret_pset);
	bool OnMovJ(long RobotSerial, double start_joint[7], double end_joint[7], double vel, double acc, double jerk, char *path);
	bool OnMovL_KeepJ_Cut(long RobotSerial, double startjoints[7], double stopjoints[7], double vel, double acc, char *path);
	bool OnMovL_KeepJ_CutA(long RobotSerial, double startjoints[7], double stopjoints[7], double vel, double acc, CPointSet *ret_pset);
	bool OnMovTarget(long RobotSerial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, CPointSet *ret_pset);

	// Multi-Point Motion Planning
	bool OnInit_MOVL_ZSP();
	bool OnMovL_ZSP(long RobotSerial, double ref_joints[7], double start_pos[6], double end_pos[6], double vel, double acc, double jerk, long ZSP_type, double ZSP_para[6], double Allow_Range, long Point_State);
	bool OnSendPoints(CPointSet *out);

	CPointSet m_output_pset;
	bool Overlap_Tag;
	long Overlap_Num;
	double last_jv[7];
	double next_start_pos[6];

protected:
	bool OnPln(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet *ret);
	bool OnPlnAcc(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet *ret);
	bool OnPlnAccNew(double start_pos, double end_pos, double vel, double acc, double jerk, CPointSet *ret, CPointSet *ret1);
	bool OnPlnAccSimple(double start_pos, double vel, double acc, CPointSet *ret);
	bool InitPln(double s, double v, double a, double j);
	long OnGetPlnNum();
	double OnGetPln(double *ret_v);
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

	bool m_Set_Freq;
	double m_freq;
	double m_cycle; // frequency to cycle

	double m_Joint_Pos_Neg[7];
	double m_Joint_Pos_Pos[7];
	double m_Joint_Vel_Lmt[7];
	double m_Joint_Acc_Lmt[7];

	bool OnGetRatioByCntScale(long total_cnt, long cur_cnt, double &ratio1, double &ratio2);
};

class CAxisJointPln
{
public:
	CAxisJointPln();
	virtual ~CAxisJointPln();
	FX_BOOL OnMovJoint(FX_INT32 RobotSerial, Vect7 start_joint, Vect7 end_joint, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio, CPointSet *ret_pset);

	FX_BOOL OnSetLmt(FX_INT32 dof, Vect8 PosNeg, Vect8 PosPos, Vect8 VelLmt, Vect8 AccLmt);
	FX_VOID OnSetFreq(FX_INT32 freq);
	FX_INT32 OnPln(Vect8 startp, Vect8 stopp, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio);
	FX_BOOL OnCut(Vect8 retp);

protected:
	FX_INT32 m_dof;
	Vect8 m_PosNeg;
	Vect8 m_PosPos;
	Vect8 m_VelLmt;
	Vect8 m_AccLmt;

	Vect8 m_start;
	Vect8 m_stop;
	FX_INT32 m_Pln_Type[8];
	Vect8 m_Pln_Len;
	Vect8 m_Pln_TRatio;
	Vect8 m_Pln_T;
	FX_DOUBLE m_Pln_P1[8][6]; // start_pos vel acc len t r
	FX_DOUBLE m_Pln_P2[8][6];
	FX_DOUBLE m_Pln_P3[8][6];

	FX_DOUBLE m_totl_t;
	FX_DOUBLE m_cur_t;

	FX_DOUBLE m_value[8][10];
	FX_INT32 m_wpos;
	FX_BOOL m_FristTag;

	FX_DOUBLE m_ts;
	FX_INT32 m_LastError;
};

class CMovingAverageFilter
{
public:
	CMovingAverageFilter();
	~CMovingAverageFilter();

	bool FilterPointSet(CPointSet *input, CPointSet *output);
	bool FilterSinglePoint(double **points, long index, long point_count,
						   long point_dim, double *filtered_point);

private:
	static const long WINDOW_SIZE = 5;
};
#endif
