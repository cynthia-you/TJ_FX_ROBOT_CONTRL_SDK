#include "Arm_Kinematics.h"
#include "FXMatrix.h"
#include <stdio.h>
#include <stdlib.h>

static FX_BOOL KineIF_Log_Switch = FX_FALSE;

CFxKineIF::CFxKineIF()
{
	m_InitTag = FX_FALSE;
}

CFxKineIF::~CFxKineIF()
{
}

FX_VOID CFxKineIF::L0_OnLogSwitch(FX_INT32 log_tag_input)
{
	FX_LOG_SWITCH(log_tag_input);
	m_Env.On_Log_Switch(log_tag_input);
	KineIF_Log_Switch = (log_tag_input == 1) ? FX_TRUE : FX_FALSE;
}

FX_BOOL CFxKineIF::L0_OnInitEnv(FX_INT32 RobotSerial, FX_INT32 *type, FX_DOUBLE DH[8][4], FX_DOUBLE PNVA[8][4], FX_DOUBLE BOUND[4][3],
								FX_DOUBLE GRV[3], FX_DOUBLE MASS[7], FX_DOUBLE MCP[7][3], FX_DOUBLE I[7][6])
{
	m_InitTag = FX_FALSE;
	if (m_Env.OnInitEnv(RobotSerial, type, DH, PNVA, BOUND, GRV, MASS, MCP, I) == FX_FALSE)
	{
		return FX_FALSE;
	}
	{
		Kine_Serial = RobotSerial;
		FX_INT32 RobotType;
		RobotType = *type;
		
		Matrix4 tool;
		FX_IdentM44(tool);
		if (KineIF_Log_Switch)
		{
			printf("OnInitEnv RobotType=%d\n", RobotType);
		}
		FX_Robot_Init_Type(Kine_Serial, RobotType);
		FX_Robot_Init_Kine(Kine_Serial, DH);
		FX_Robot_Init_Lmt(Kine_Serial, PNVA, BOUND);
		FX_Robot_Tool_Set(Kine_Serial, tool);
	}

	m_InitTag = FX_TRUE;
	return FX_TRUE;
}

FX_BOOL CFxKineIF::L0_OnSetTool(Matrix4 tool)
{
	if (m_InitTag == FX_FALSE)
	{
		printf("Please initialize kinematics environment first\n");
		return FX_FALSE;
	}
	if (Kine_Serial < 0 || Kine_Serial > 1)
	{
		printf("Invalid Robot Serial %d\n", Kine_Serial);
		return FX_FALSE;
	}
	return FX_Robot_Tool_Set(Kine_Serial, tool);
}

FX_VOID CFxKineIF::L0_OnRmvTool()
{
	if (m_InitTag == FX_FALSE)
	{
		printf("Please initialize kinematics environment first\n");
		return;
	}
	if (Kine_Serial < 0 || Kine_Serial > 1)
	{
		printf("Invalid Robot Serial %d\n", Kine_Serial);
		return;
	}
	FX_Robot_Tool_Rmv(Kine_Serial);
}

FX_BOOL CFxKineIF::L0_OnSolveArmFK(Vect7 joints, Matrix4 pgos)
{
	if (m_InitTag == FX_FALSE)
	{
		printf("Please initialize kinematics environment first\n");
		return FX_FALSE;
	}
	if (Kine_Serial < 0 || Kine_Serial > 1)
	{
		printf("Invalid Robot Serial %d\n", Kine_Serial);
		return FX_FALSE;
	}
	return FX_Robot_Kine_FK(Kine_Serial, joints, pgos);
}

FX_BOOL CFxKineIF::L0_OnSolveArmJcb(Vect7 joints, FX_DOUBLE jcb[6][7])
{

	FX_Jacobi jcb_t;
	if (m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (Kine_Serial < 0 || Kine_Serial > 1)
	{
		return FX_FALSE;
	}
	if (FX_Robot_Kine_Jacb(Kine_Serial, joints, &jcb_t) == FX_FALSE)
	{
		return FX_FALSE;
	}
	FX_M67Copy(jcb_t.m_Jcb, jcb);
	return FX_TRUE;
}

FX_BOOL CFxKineIF::L0_OnSolveArmIK(FX_InvKineSolvePara *solve_para)
{
	if (m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (Kine_Serial < 0 || Kine_Serial > 1)
	{
		return FX_FALSE;
	}
	return FX_Robot_Kine_IK(Kine_Serial, solve_para);
}

FX_INT32 CFxKineIF::L0_OnIdenDynaLoad(FX_CHAR *path, FX_DOUBLE *mass, Vect3 mr, Vect6 I)
{
	if (m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (Kine_Serial < 0 || Kine_Serial > 1)
	{
		return FX_FALSE;
	}

	FX_INT32 RobotType;
	if (m_Env.OnGetArmType(RobotType) == FX_FALSE)
	{
		printf("Failed to get arm type from environment\n");
		return FX_FALSE;
	}

	FX_INT32 Type = 0;
	if (RobotType == FX_ROBOT_TYPE_PILOT_CCS)
	{
		Type = 1;
	}
	else if (RobotType == FX_ROBOT_TYPE_PILOT_SRS)
	{
		Type = 2;
	}

	return FX_Robot_Iden_LoadDyn(Type, path, mass, mr, I);
}

FX_BOOL CFxKineIF::OnGetArmLmt(FX_INT32 &type, FX_DOUBLE PosNeg[8], FX_DOUBLE PosPos[8], FX_DOUBLE VelLmt[8], FX_DOUBLE AccLmt[8])
{
	return m_Env.OnGetArmLmt(type, PosNeg, PosPos, VelLmt, AccLmt);
}