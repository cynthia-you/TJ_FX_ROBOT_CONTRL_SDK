#include "Motion_Planner.h"
#include "MAX_Body_Kinematics.h"
// #include "FxKinematics.h"
#include "FXMatrix.h"

static FX_BOOL Pln_Log_Switch = FX_FALSE;

CFxPln::CFxPln()
{
}

CFxPln::~CFxPln()
{
}

FX_VOID CFxPln::L0_OnLogSwitch(FX_INT32 log_tag_input)
{
	FX_LOG_SWITCH(log_tag_input);
	Pln_Log_Switch = (log_tag_input == 1) ? FX_TRUE : FX_FALSE;
}

FX_BOOL CFxPln::L0_OnInitEnv_SingleArm(FX_INT32 RobotSerial, FX_INT32 *type, FX_DOUBLE DH[8][4], FX_DOUBLE PNVA[8][4], FX_DOUBLE BOUND[4][3])
{
	if (RobotSerial < 0 || RobotSerial > 1)
	{
		return FX_FALSE;
	}
	FX_DOUBLE GRV[3]={0};
	FX_DOUBLE MASS[7]={0};
	FX_DOUBLE MCP[7][3]={{0}};
	FX_DOUBLE I[7][6]={{0}};
	FX_BOOL result = m_KineIF.L0_OnInitEnv(RobotSerial, type, DH, PNVA, BOUND, GRV, MASS, MCP, I);
	return result;
}

FX_BOOL CFxPln::L0_OnInitEnv_DualArm(FX_INT32 type[2], FX_DOUBLE DH[2][8][4], FX_DOUBLE PNVA[2][8][4], FX_DOUBLE BOUND[2][4][3])
{
	FX_DOUBLE GRV[3]={0};
	FX_DOUBLE MASS[7]={0};
	FX_DOUBLE MCP[7][3]={{0}};
	FX_DOUBLE I[7][6]={{0}};

	FX_BOOL result_left = m_Kine_Left_Arm.L0_OnInitEnv(0, &type[0], DH[0], PNVA[0], BOUND[0], GRV, MASS, MCP, I);
	FX_BOOL result_right = m_Kine_Right_Arm.L0_OnInitEnv(1, &type[1], DH[1], PNVA[1], BOUND[1], GRV, MASS, MCP, I);

	return (result_left && result_right);
}

FX_BOOL CFxPln::L0_OnMovJ(Vect7 start_joint, Vect7 end_joint, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio, CPointSet *ret_pset)
{
	if (m_KineIF.m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (m_KineIF.Kine_Serial < 0 || m_KineIF.Kine_Serial > 1)
	{
		return FX_FALSE;
	}

	// get joint limits for motion planning
	FX_INT32 type = 0;
	FX_DOUBLE lmt_pos[8] = {0};
	FX_DOUBLE lmt_neg[8] = {0};
	FX_DOUBLE lmt_vel[8] = {0};
	FX_DOUBLE lmt_acc[8] = {0};
	m_KineIF.OnGetArmLmt(type, lmt_neg, lmt_pos, lmt_vel, lmt_acc);

	// set joint limits to joint space planner
	FX_INT32 dof = 0;
	if (type == FX_ROBOT_TYPE_PILOT_SRS || type == FX_ROBOT_TYPE_PILOT_CCS)
	{
		dof = 7;
	}
	else if (type == FX_ROBOT_TYPE_DL)
	{
		dof = 6;
	}
	m_AxisJointPln.OnSetLmt(dof, lmt_neg, lmt_pos, lmt_vel, lmt_acc);

	// plan in joint space
	FX_BOOL result = m_AxisJointPln.OnMovJoint(m_KineIF.Kine_Serial, start_joint, end_joint, vel_ratio, acc_ratio, ret_pset);
	return result;
}

FX_BOOL CFxPln::L0_OnMovL(Vect6 Start_XYZABC, Vect6 End_XYZABC, Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32 freq, CPointSet *pset)
{
	if (m_KineIF.m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (m_KineIF.Kine_Serial < 0 || m_KineIF.Kine_Serial > 1)
	{
		return FX_FALSE;
	}

	Vect6 start_pos = {0};
	Vect6 end_pos = {0};
	Vect7 refJ = {0};
	FX_INT32 i = 0;

	for (i = 0; i < 6; i++)
	{
		start_pos[i] = Start_XYZABC[i];
		end_pos[i] = End_XYZABC[i];
		refJ[i] = Ref_Joints[i];
	}
	refJ[i] = Ref_Joints[i];

	FX_DOUBLE jerk = ACC * 10;

	CAxisPln Spln;
	Spln.OnSetFreq(freq);
	FX_BOOL result = Spln.OnMovL(m_KineIF.Kine_Serial, refJ, start_pos, end_pos, Vel, ACC, jerk, pset);

	return result;
}

FX_BOOL CFxPln::L0_OnMovL_KeepJ(Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq, CPointSet *pset)
{
	if (m_KineIF.m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (m_KineIF.Kine_Serial < 0 || m_KineIF.Kine_Serial > 1)
	{
		return FX_FALSE;
	}

	Vect7 start_pos = {0};
	Vect7 end_pos = {0};
	FX_INT32 i = 0;

	for (i = 0; i < 7; i++)
	{
		start_pos[i] = startjoints[i];
		end_pos[i] = stopjoints[i];
	}

	CAxisPln Spln;
	Spln.OnSetFreq(freq);
	FX_BOOL result = Spln.OnMovL_KeepJ_CutA(m_KineIF.Kine_Serial, start_pos, end_pos, vel, acc, pset);
	return result;
}

FX_VOID CFxPln::L0_XYZABC2Matrix4_DEG(FX_DOUBLE xyzabc[6], FX_DOUBLE m[4][4])
{
	FX_DOUBLE angx = xyzabc[3];
	FX_DOUBLE angy = xyzabc[4];
	FX_DOUBLE angz = xyzabc[5];
	FX_DOUBLE sa = 0.0;
	FX_DOUBLE sb = 0.0;
	FX_DOUBLE sr = 0.0;
	FX_DOUBLE ca = 0.0;
	FX_DOUBLE cb = 0.0;
	FX_DOUBLE cr = 0.0;

	FX_SIN_COS_DEG(angx, &sr, &cr);
	FX_SIN_COS_DEG(angy, &sb, &cb);
	FX_SIN_COS_DEG(angz, &sa, &ca);

	m[0][0] = ca * cb;
	m[0][1] = ca * sb * sr - sa * cr;
	m[0][2] = ca * sb * cr + sa * sr;

	m[1][0] = sa * cb;
	m[1][1] = sa * sb * sr + ca * cr;
	m[1][2] = sa * sb * cr - ca * sr;

	m[2][0] = -sb;
	m[2][1] = cb * sr;
	m[2][2] = cb * cr;

	m[0][3] = xyzabc[0];
	m[1][3] = xyzabc[1];
	m[2][3] = xyzabc[2];

	m[3][0] = 0;
	m[3][1] = 0;
	m[3][2] = 0;
	m[3][3] = 1;
}

FX_VOID CFxPln::L0_Matrix42XYZABC_DEG(FX_DOUBLE m[4][4], FX_DOUBLE xyzabc[6])
{
	FX_DOUBLE r = FX_Sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0]);
	xyzabc[4] = FX_ATan2(-m[2][0], r);
	if (r <= FXARM_EPS && r >= -FXARM_EPS)
	{
		xyzabc[5] = 0;
		if (xyzabc[4] > 0)
		{
			xyzabc[3] = FX_ATan2(m[0][1], m[1][1]);
		}
		else
		{
			xyzabc[3] = -FX_ATan2(m[0][1], m[1][1]);
		}
	}
	else
	{
		xyzabc[5] = FX_ATan2(m[1][0], m[0][0]);
		xyzabc[3] = FX_ATan2(m[2][1], m[2][2]);
	}
	xyzabc[0] = m[0][3];
	xyzabc[1] = m[1][3];
	xyzabc[2] = m[2][3];

	xyzabc[3] = xyzabc[3] * FXARM_R2D;
	xyzabc[4] = xyzabc[4] * FXARM_R2D;
	xyzabc[5] = xyzabc[5] * FXARM_R2D;
}

FX_BOOL CFxPln::L0_MultiPoints_Set_MovL_Start(Vect7 refjoints, Vect6 Start_XYZABC, Vect6 End_XYZABC,
											  FX_DOUBLE Allow_Range, FX_INT32 ZSP_Type, Vect6 ZSP_Para, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32 freq)
{
	Vect6 start_pos = {0};
	Vect6 end_pos = {0};
	Vect7 refJ = {0};
	FX_INT32 i = 0;

	for (i = 0; i < 6; i++)
	{
		start_pos[i] = Start_XYZABC[i];
		end_pos[i] = End_XYZABC[i];
		refJ[i] = refjoints[i];
	}
	refJ[i] = refjoints[i];

	FX_DOUBLE jerk = ACC * 10;
	m_AxisPln.OnInit_MOVL_ZSP();
	m_AxisPln.OnSetFreq(freq);
	return m_AxisPln.OnMovL_ZSP(m_KineIF.Kine_Serial, refJ, start_pos, end_pos, Vel, ACC, jerk, ZSP_Type, ZSP_Para, Allow_Range, FX_MOVL_START);
}

FX_BOOL CFxPln::L0_MultiPoints_Set_MovL_NextPoints(Vect6 Next_XYZABC,
												   FX_DOUBLE Allow_Range, FX_INT32 ZSP_Type, Vect6 ZSP_Para, FX_DOUBLE Vel, FX_DOUBLE ACC)
{
	Vect6 start_pos = {0};
	Vect6 end_pos = {0};
	Vect7 refJ = {0};
	FX_INT32 i = 0;

	for (i = 0; i < 6; i++)
	{
		start_pos[i] = 0.0;
		end_pos[i] = Next_XYZABC[i];
		refJ[i] = 0.0;
	}
	refJ[i] = 0.0;

	FX_DOUBLE jerk = ACC * 10;

	return m_AxisPln.OnMovL_ZSP(m_KineIF.Kine_Serial, refJ, start_pos, end_pos, Vel, ACC, jerk, ZSP_Type, ZSP_Para, Allow_Range, FX_MOVL_NEXT);
}

FX_BOOL CFxPln::L0_MultiPoints_Get_MovL_Path(CPointSet *ret_pset)
{
	return m_AxisPln.OnSendPoints(ret_pset);
}

//////////////////////////////DualArm with FixBody
FX_BOOL CFxPln::L0_OnMovL_DualArm_FixBody(DualArm_FixedBody *DA_FB, CPointSet *Left_Arm_Pln_Path, CPointSet *Right_Arm_Pln_Path)
{
	if (!m_Kine_Left_Arm.m_InitTag || !m_Kine_Right_Arm.m_InitTag)
	{
		return FX_FALSE;
	}

	if (DA_FB->World_Co_Flag)
	{
		// Convert XYZABC to matrix
		Matrix4 Left_Arm_Start_EE, Left_Arm_End_EE;
		L0_XYZABC2Matrix4_DEG(DA_FB->Left_Arm_Start_XYZABC, Left_Arm_Start_EE);
		L0_XYZABC2Matrix4_DEG(DA_FB->Left_Arm_End_XYZABC, Left_Arm_End_EE);

		Matrix4 Right_Arm_Start_EE, Right_Arm_End_EE;
		L0_XYZABC2Matrix4_DEG(DA_FB->Right_Arm_Start_XYZABC, Right_Arm_Start_EE);
		L0_XYZABC2Matrix4_DEG(DA_FB->Right_Arm_End_XYZABC, Right_Arm_End_EE);

		// Calculate Shoulder TCP
		Matrix4 Shoulder_left, Shoulder_right;
		CFxKineMAX km;
		km.L0_OnKineLR(DA_FB->Max_Body_Start_PRR, Shoulder_left, Shoulder_right);

		// Calculate the inverse of Shoulder TCP
		Matrix4 Shoulder_left_inv, Shoulder_right_inv;
		MatrixInv44(Shoulder_left, Shoulder_left_inv);
		MatrixInv44(Shoulder_right, Shoulder_right_inv);

		// Convert World Coordinate to Base(Shoulder) Coordinate
		Matrix4 Left_Arm_Start_Base, Left_Arm_End_Base;
		FX_MMM44(Shoulder_left_inv, Left_Arm_Start_EE, Left_Arm_Start_Base);
		FX_MMM44(Shoulder_left_inv, Left_Arm_End_EE, Left_Arm_End_Base);

		Matrix4 Right_Arm_Start_Base, Right_Arm_End_Base;
		FX_MMM44(Shoulder_right_inv, Right_Arm_Start_EE, Right_Arm_Start_Base);
		FX_MMM44(Shoulder_right_inv, Right_Arm_End_EE, Right_Arm_End_Base);

		// Real Dual_Arm TCP
		L0_Matrix42XYZABC_DEG(Left_Arm_Start_Base, DA_FB->Left_Arm_Start_XYZABC);
		L0_Matrix42XYZABC_DEG(Left_Arm_End_Base, DA_FB->Left_Arm_End_XYZABC);
		L0_Matrix42XYZABC_DEG(Right_Arm_Start_Base, DA_FB->Right_Arm_Start_XYZABC);
		L0_Matrix42XYZABC_DEG(Right_Arm_End_Base, DA_FB->Right_Arm_End_XYZABC);
	}

	CAxisPln Spln;
	Spln.OnSetFreq(DA_FB->Freq);

	return Spln.OnMovL_DualArm_FixBody(DA_FB, Left_Arm_Pln_Path, Right_Arm_Pln_Path);
}
