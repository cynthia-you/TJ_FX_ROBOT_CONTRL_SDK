
#ifndef _AXISPLN_H_
#define _AXISPLN_H_

//#include "FXMatrix.h"
#include "FxKinematics.h"

enum FX_POINT_STATE
{
	FX_MOVL_START = 81,
	FX_MOVL_NEXT  = 82,
};

struct DualArm_FixedBody
{
	////////////////////Input
	///////////Max_Body
	Vect3 Max_Body_Start_PRR;
	//////////Left_Arm
	Vect6 Left_Arm_Start_XYZABC;
	Vect6 Left_Arm_End_XYZABC;
	Vect7 Left_Arm_Ref_Joints;

	FX_INT32 Left_Arm_ZSP_Type;
	Vect6 Left_Arm_ZSP_Para;
	//////////Right_Arm
	Vect6 Right_Arm_Start_XYZABC;
	Vect6 Right_Arm_End_XYZABC;
	Vect7 Right_Arm_Ref_Joints;

	FX_INT32 Right_Arm_ZSP_Type;
	Vect6 Right_Arm_ZSP_Para;

	//////////Common
	FX_INT32 Freq;  
	FX_DOUBLE Vel;
	FX_DOUBLE Acc;
	FX_INT32 Sync_Type; 
	FX_BOOL   World_Co_Flag; 
};

class CAxisPln
{
public:
	CAxisPln();
	virtual ~CAxisPln();

	FX_VOID OnSetFreq(FX_INT32 freq);

	FX_BOOL OnMovL(FX_INT32 RobotSerial, Vect7 ref_joints, Vect6 start_pos, Vect6 end_pos, FX_DOUBLE vel, FX_DOUBLE acc, FX_DOUBLE jerk, FX_CHAR* path);
	FX_BOOL OnMovL(FX_INT32 RobotSerial, Vect7 ref_joints, Vect6 start_pos, Vect6 end_pos, FX_DOUBLE vel, FX_DOUBLE acc, FX_DOUBLE jerk, CPointSet * ret_pset);
	FX_BOOL OnMovJ(FX_INT32 RobotSerial, Vect7 start_joint, Vect7 end_joint, FX_DOUBLE vel, FX_DOUBLE acc, FX_DOUBLE jerk, FX_CHAR* path);
	FX_BOOL OnMovL_KeepJ_Cut(FX_INT32 RobotSerial, Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_DOUBLE acc, FX_CHAR* path);
	FX_BOOL OnMovL_KeepJ_CutA(FX_INT32 RobotSerial, Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_DOUBLE acc, CPointSet * ret_pset);

	//Multi-Point Motion Planning
	FX_BOOL OnInit_MOVL_ZSP();
	FX_BOOL OnMovL_ZSP(FX_INT32 RobotSetial, Vect7 ref_joints, Vect6 start_pos, Vect6 end_pos, FX_DOUBLE vel, FX_DOUBLE acc, FX_DOUBLE jerk, FX_INT32 ZSP_type, Vect6 ZSP_para, FX_DOUBLE Allow_Range, FX_INT32 Point_State);
	FX_BOOL OnSendPoints(CPointSet* out);

	//Dual-Arms Synchronous Motion Planning with Fixed Body
	FX_BOOL OnMovL_DualArm_FixBody(DualArm_FixedBody* DA_FB,CPointSet* Left_Arm_Pln_Path, CPointSet* Right_Arm_Pln_Path);

	CPointSet m_output_pset;
	FX_BOOL Overlap_Tag;
	FX_INT32 Overlap_Num;
	Vect7 last_jv;
	Vect6 next_start_pos;
protected:
	FX_BOOL InitPln(FX_DOUBLE s, FX_DOUBLE v, FX_DOUBLE a, FX_DOUBLE j);
	FX_BOOL OnPln(FX_DOUBLE start_pos, FX_DOUBLE end_pos, FX_DOUBLE vel, FX_DOUBLE acc, FX_DOUBLE jerk, CPointSet* ret);
	
	FX_INT32 OnGetPlnNum();
	FX_DOUBLE OnGetPln(FX_DOUBLE* ret_v);
	
	FX_DOUBLE m_s;
	FX_DOUBLE m_v;
	FX_DOUBLE m_a;
	FX_DOUBLE m_j;
	FX_DOUBLE m_cur_time;
	FX_DOUBLE m_time_acc;
	FX_DOUBLE m_time_dacc;
	FX_DOUBLE m_time_vel;

	FX_DOUBLE m_filt_value[500];
	FX_INT32 m_filt_cnt;
	FX_INT32 m_filt_pos;

	FX_BOOL   m_Set_Freq;
	FX_DOUBLE m_freq;
	FX_DOUBLE m_cycle;  //frequency to cycle

	FX_BOOL OnGetRatioByCntScale(FX_INT32 total_cnt, FX_INT32 cur_cnt, FX_DOUBLE& ratio1, FX_DOUBLE& ratio2);

	FX_DOUBLE OnGetLength(Vect6 start_pos, Vect6 end_pos, Quaternion Q_start, Quaternion Q_end);
	FX_INT32 OnXYZQ2Joint(DualArm_FixedBody* DA_FB, CPointSet* cartesian_traj, CPointSet* output, FX_INT32 RobotSerial);
};

class CAxisJointPln
{
public:
	CAxisJointPln();
	virtual ~CAxisJointPln();
	FX_BOOL OnMovJoint(FX_INT32 RobotSerial, Vect7 start_joint, Vect7 end_joint, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio, CPointSet* ret_pset);

	FX_BOOL OnSetLmt(FX_INT32 dof,Vect8 PosNeg, Vect8 PosPos, Vect8 VelLmt, Vect8 AccLmt);
	FX_INT32 OnPln(Vect8 startp,Vect8 stopp,FX_DOUBLE vel_ratio,FX_DOUBLE acc_ratio);
	FX_BOOL OnCut(Vect8 retp);

protected:
	FX_INT32 m_dof;
	Vect8 m_PosNeg;
	Vect8 m_PosPos;
	Vect8 m_VelLmt;
	Vect8 m_AccLmt;

	Vect8 m_start;
	Vect8 m_stop;
	FX_INT32   m_Pln_Type[8];
	Vect8 m_Pln_Len;
	Vect8 m_Pln_TRatio;
	Vect8   m_Pln_T;
	FX_DOUBLE   m_Pln_P1[8][6]; // start_pos vel acc len t r
	FX_DOUBLE   m_Pln_P2[8][6];
	FX_DOUBLE   m_Pln_P3[8][6];

	FX_DOUBLE m_totl_t;
	FX_DOUBLE m_cur_t;


	FX_DOUBLE m_value[8][10];
	FX_INT32   m_wpos;
	FX_BOOL   m_FristTag;

	FX_DOUBLE m_ts;
	
};

class CMovingAverageFilter
{
public:
	CMovingAverageFilter();
	~CMovingAverageFilter();

	FX_BOOL FilterPointSet(CPointSet* input, CPointSet* output);
	FX_BOOL FilterSinglePoint(FX_DOUBLE** points, FX_INT32 index, FX_INT32 point_count,
		FX_INT32 point_dim, FX_DOUBLE* filtered_point);

private:
	static const FX_INT32 WINDOW_SIZE = 5; 
};
#endif 
