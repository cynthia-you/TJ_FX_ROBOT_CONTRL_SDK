#include "FxRobot.h"
#include "FXMatrix.h"
#include "FXSpln.h"
#include "LoadIdenPub.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#define FX_LOG_INFO(format) printf(format)


FX_VOID PRINT44(double matrix[4][4])
{
    long i,j;
    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            printf("%lf ",matrix[i][j]);
        }
        printf("\n");
    }
}
//FX_VOID PRINT83(double matrix[8][3])
//{
//    long i,j;
//    for(i=0;i<8;i++)
//    {
//        for(j=0;j<3;j++)
//        {
//            printf("%lf ",matrix[i][j]);
//        }
//        printf("\n");
//    }
//}


#ifdef _USER_IF_TAG_

FX_BOOL  LOADMvCfg(FX_CHAR* path, FX_INT32L TYPE[2], FX_DOUBLE GRV[2][3], FX_DOUBLE DH[2][8][4], FX_DOUBLE PNVA[2][7][4], FX_DOUBLE BD[2][4][3],
	FX_DOUBLE Mass[2][7], FX_DOUBLE MCP[2][7][3], FX_DOUBLE I[2][7][6])
{	
	FX_LOG_INFO("[FxRobot - LOADMvCfg]\n");
	FX_INT32L i;
	FX_INT32L j;
	FX_CHAR   c;
	FILE* fp = fopen(path, "rb");
	if (fp == NULL)
	{
		return FX_FALSE;
	}

	for ( i = 0; i < 2; i++)
	{
		if (fscanf(fp, "%ld,%lf,%lf,%lf,%c", &TYPE[i], &GRV[i][0], &GRV[i][1], &GRV[i][2], &c) != 5)
		{
			fclose(fp);
			return FX_FALSE;
		}
		if (c != 0x0a)
		{
			fclose(fp);
			return FX_FALSE;
		}

		for ( j = 0; j < 7; j++)
		{
			if (fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%c", 
				&DH[i][j][0], &DH[i][j][1], &DH[i][j][2], &DH[i][j][3],
				&PNVA[i][j][1], &PNVA[i][j][0], &PNVA[i][j][2], &PNVA[i][j][3],
				&Mass[i][j], &MCP[i][j][0], &MCP[i][j][1], &MCP[i][j][2],
				&I[i][j][0], &I[i][j][1], &I[i][j][2], &I[i][j][3], &I[i][j][4], &I[i][j][5],
				&c) != 19)
			{
				fclose(fp);
				return FX_FALSE;
			}
			if (c != 0x0a)
			{
				fclose(fp);
				return FX_FALSE;
			}
		}

		if (fscanf(fp, "%lf,%lf,%lf,%lf,%c",&DH[i][7][0], &DH[i][7][1], &DH[i][7][2], &DH[i][7][3],
			&c) != 5)
		{
			fclose(fp);
			return FX_FALSE;
		}
		if (c != 0x0a)
		{
			fclose(fp);
			return FX_FALSE;
		}

		for (j = 0; j < 4; j++)
		{
			if (fscanf(fp, "%lf,%lf,%lf,%c",
				 &BD[i][j][0], &BD[i][j][1], &BD[i][j][2],&c) != 4)
			{
				fclose(fp);
				return FX_FALSE;
			}
			if (c != 0x0a)
			{
				fclose(fp);
				return FX_FALSE;
			}
		}
	}

	fclose(fp);

	{
		printf("TYPE=[%d %d]\n", TYPE[0], TYPE[1]);

	}
	return FX_TRUE;
}
#endif

static FX_Robot			m_Robot[MAX_RUN_ROBOT_NUM];
static FX_KineSPC_Pilot	m_Robot_SPC_Pilot[MAX_RUN_ROBOT_NUM];
static FX_KineSPC_DL       m_Robot_SPC_DL[MAX_RUN_ROBOT_NUM];

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FX_BOOL  FX_Robot_Init_Type(FX_INT32L RobotSerial, FX_INT32L RobotType)
{
	FX_LOG_INFO("[FxRobot - FX_Robot_Init_Type]\n");

	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	if (RobotType == FX_ROBOT_TYPE_DL)
	{
		m_Robot[RobotSerial].m_RobotType = FX_ROBOT_TYPE_DL;
		m_Robot[RobotSerial].m_RobotDOF = 6;
		m_Robot[RobotSerial].m_KineSPC = (FX_VOID*)&(m_Robot_SPC_DL[RobotSerial]);
		return FX_TRUE;
	}
	else if (RobotType == FX_ROBOT_TYPE_PILOT_SRS)
	{
		m_Robot[RobotSerial].m_RobotType = FX_ROBOT_TYPE_PILOT_SRS;
		m_Robot[RobotSerial].m_RobotDOF = 7;
		m_Robot[RobotSerial].m_KineSPC = (FX_VOID*)&(m_Robot_SPC_Pilot[RobotSerial]);
		m_Robot_SPC_Pilot[RobotSerial].m_IsCross = FX_FALSE;
		m_Robot_SPC_Pilot[RobotSerial].m_nsp.m_IsCorss = FX_FALSE;
		return FX_TRUE;
	}
	else if (RobotType == FX_ROBOT_TYPE_PILOT_CCS)
	{
		m_Robot[RobotSerial].m_RobotType = FX_ROBOT_TYPE_PILOT_CCS;
		m_Robot[RobotSerial].m_KineSPC = (FX_VOID*)&(m_Robot_SPC_Pilot[RobotSerial]);
		m_Robot_SPC_Pilot[RobotSerial].m_IsCross = FX_TRUE;
		m_Robot_SPC_Pilot[RobotSerial].m_nsp.m_IsCorss = FX_TRUE;
		return FX_TRUE;
	}
	else
	{
		return FX_FALSE;
	}
	return FX_FALSE;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

FX_BOOL  FX_Init_Robot_Kine_DL(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4])
{
	return FX_FALSE;
}

FX_BOOL  FX_Init_Robot_Kine_Pilot_SRS(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4])
{
	FX_LOG_INFO("[FxRobot - FX_Init_Robot_Kine_Pilot_SRS]\n");
	FX_INT32 i;
	
	FX_DOUBLE L1;
	FX_DOUBLE L2;
	FX_DOUBLE L3;
	FX_DOUBLE D;
	FX_DOUBLE Flan;
	
	FX_Robot* pRobot;
	FX_KineSPC_Pilot* SPC;
	
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	SPC = (FX_KineSPC_Pilot*)(pRobot->m_KineSPC);
	SPC->m_IsCross = FX_FALSE;

	
	
	for (i = 0; i < 7; i++) {
		FX_IdentM44(pRobot->m_KineBase.m_AxisRotBase[i]);
		FX_IdentM44(pRobot->m_KineBase.m_JointPG[i]);
		FX_IdentM44(pRobot->m_KineBase.m_AxisRotTip[i]);
	}

	FX_IdentM44(pRobot->m_KineBase.m_Flange);
	FX_IdentM44(pRobot->m_KineBase.m_InvFlange);
	FX_IdentM44(pRobot->m_KineBase.m_Tool);
	FX_IdentM44(pRobot->m_KineBase.m_InvTool);
	FX_IdentM44(pRobot->m_KineBase.m_TCP);

	L1 = DH[0][2];
	L2 = DH[2][2];
	L3 = DH[4][2];
	D = DH[3][1];
	Flan = DH[7][2];

	pRobot->m_KineBase.m_AxisRotBase[0][2][3] = L1;

	pRobot->m_KineBase.m_AxisRotBase[1][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[1][1][2] = -1;
	pRobot->m_KineBase.m_AxisRotBase[1][2][1] = 1; pRobot->m_KineBase.m_AxisRotBase[1][2][2] = 0;

	pRobot->m_KineBase.m_AxisRotBase[2][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[2][1][2] = 1;
	pRobot->m_KineBase.m_AxisRotBase[2][2][1] = -1; pRobot->m_KineBase.m_AxisRotBase[2][2][2] = 0;

	pRobot->m_KineBase.m_AxisRotBase[3][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[3][1][2] = -1;
	pRobot->m_KineBase.m_AxisRotBase[3][2][1] = 1; pRobot->m_KineBase.m_AxisRotBase[3][2][2] = 0;
	pRobot->m_KineBase.m_AxisRotBase[3][0][3] = D;
	pRobot->m_KineBase.m_AxisRotBase[3][2][3] = L2;

	pRobot->m_KineBase.m_AxisRotBase[4][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[4][1][2] = 1;
	pRobot->m_KineBase.m_AxisRotBase[4][2][1] = -1; pRobot->m_KineBase.m_AxisRotBase[4][2][2] = 0;
	pRobot->m_KineBase.m_AxisRotBase[4][0][3] = -D;
	pRobot->m_KineBase.m_AxisRotBase[4][1][3] = L3;

	pRobot->m_KineBase.m_AxisRotBase[5][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[5][1][2] = -1;
	pRobot->m_KineBase.m_AxisRotBase[5][2][1] = 1; pRobot->m_KineBase.m_AxisRotBase[5][2][2] = 0;

	{
		pRobot->m_KineBase.m_AxisRotBase[6][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[6][1][2] = 1;
		pRobot->m_KineBase.m_AxisRotBase[6][2][1] = -1; pRobot->m_KineBase.m_AxisRotBase[6][2][2] = 0;
		pRobot->m_KineBase.m_Flange[2][3] = Flan;
		FX_PGMatrixInv(pRobot->m_KineBase.m_Flange, pRobot->m_KineBase.m_InvFlange);
	}


	// Inverse Kinematics
	SPC->L1 = FX_Sqrt(L2 * L2 + D * D);
	SPC->L2 = FX_Sqrt(L3 * L3 + D * D);
	// initial angle of t4
	SPC->Ang1 = FX_ATan2(D, L2) * FXARM_R2D;
	SPC->Ang2 = FX_ATan2(D, L3) * FXARM_R2D;
	SPC->Angt = 180.0 - SPC->Ang1 - SPC->Ang2;
	SPC->m_J4_Bound = -SPC->Ang1;

	SPC->cart_len = FX_Sqrt(SPC->L1 * SPC->L1 + SPC->L2 * SPC->L2);

	printf("EG:DH[0]=[%lf %lf %lf %lf]\n",DH[0][0], DH[0][1], DH[0][2], DH[0][3]);

	return FX_TRUE;
}

FX_BOOL  FX_Init_Robot_Kine_Pilot_CCS(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4])
{
    FX_LOG_INFO("[FxRobot - FX_Init_Robot_Kine_Pilot_CCS]\n");
	FX_INT32 i;
	FX_DOUBLE L1;
	FX_DOUBLE L2;
	FX_DOUBLE L3;
	FX_DOUBLE D;
	FX_DOUBLE Flan;
	FX_Robot* pRobot;
	FX_KineSPC_Pilot* SPC;
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	SPC = (FX_KineSPC_Pilot*)(pRobot->m_KineSPC);
	SPC->m_IsCross = FX_TRUE;
	
	L1 = DH[0][2];
	L2 = DH[2][2];
	L3 = DH[4][2];
	D = DH[3][1];
	Flan = DH[7][2];
	
	
	for (i = 0; i < 7; i++) {
		FX_IdentM44(pRobot->m_KineBase.m_AxisRotBase[i]);
		FX_IdentM44(pRobot->m_KineBase.m_JointPG[i]);
		FX_IdentM44(pRobot->m_KineBase.m_AxisRotTip[i]);
	}

	FX_IdentM44(pRobot->m_KineBase.m_Flange);
	FX_IdentM44(pRobot->m_KineBase.m_InvFlange);
	FX_IdentM44(pRobot->m_KineBase.m_Tool);
	FX_IdentM44(pRobot->m_KineBase.m_InvTool);
	FX_IdentM44(pRobot->m_KineBase.m_TCP);


	pRobot->m_KineBase.m_AxisRotBase[0][2][3] = L1;

	pRobot->m_KineBase.m_AxisRotBase[1][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[1][1][2] = -1;
	pRobot->m_KineBase.m_AxisRotBase[1][2][1] = 1; pRobot->m_KineBase.m_AxisRotBase[1][2][2] = 0;

	pRobot->m_KineBase.m_AxisRotBase[2][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[2][1][2] = 1;
	pRobot->m_KineBase.m_AxisRotBase[2][2][1] = -1; pRobot->m_KineBase.m_AxisRotBase[2][2][2] = 0;

	pRobot->m_KineBase.m_AxisRotBase[3][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[3][1][2] = -1;
	pRobot->m_KineBase.m_AxisRotBase[3][2][1] = 1; pRobot->m_KineBase.m_AxisRotBase[3][2][2] = 0;
	pRobot->m_KineBase.m_AxisRotBase[3][0][3] = D;
	pRobot->m_KineBase.m_AxisRotBase[3][2][3] = L2;

	pRobot->m_KineBase.m_AxisRotBase[4][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[4][1][2] = 1;
	pRobot->m_KineBase.m_AxisRotBase[4][2][1] = -1; pRobot->m_KineBase.m_AxisRotBase[4][2][2] = 0;
	pRobot->m_KineBase.m_AxisRotBase[4][0][3] = -D;
	pRobot->m_KineBase.m_AxisRotBase[4][1][3] = L3;

	pRobot->m_KineBase.m_AxisRotBase[5][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[5][1][2] = -1;
	pRobot->m_KineBase.m_AxisRotBase[5][2][1] = 1; pRobot->m_KineBase.m_AxisRotBase[5][2][2] = 0;

	{
		pRobot->m_KineBase.m_AxisRotBase[6][0][0] = 0; pRobot->m_KineBase.m_AxisRotBase[6][0][1] = 0; pRobot->m_KineBase.m_AxisRotBase[6][0][2] = 1;
		pRobot->m_KineBase.m_AxisRotBase[6][1][0] = 1; pRobot->m_KineBase.m_AxisRotBase[6][1][1] = 0; pRobot->m_KineBase.m_AxisRotBase[6][1][2] = 0;
		pRobot->m_KineBase.m_AxisRotBase[6][2][0] = 0; pRobot->m_KineBase.m_AxisRotBase[6][2][1] = 1; pRobot->m_KineBase.m_AxisRotBase[6][2][2] = 0;

		pRobot->m_KineBase.m_Flange[0][0] = 0; pRobot->m_KineBase.m_Flange[0][1] = 0;  pRobot->m_KineBase.m_Flange[0][2] = 1; pRobot->m_KineBase.m_Flange[0][3] = Flan;
		pRobot->m_KineBase.m_Flange[1][0] = 0; pRobot->m_KineBase.m_Flange[1][1] = -1; pRobot->m_KineBase.m_Flange[1][2] = 0;
		pRobot->m_KineBase.m_Flange[2][0] = 1; pRobot->m_KineBase.m_Flange[2][1] = 0;  pRobot->m_KineBase.m_Flange[2][2] = 0;

		FX_PGMatrixInv(pRobot->m_KineBase.m_Flange, pRobot->m_KineBase.m_InvFlange);
	}


	// Inverse Kinematics
	SPC->L1 = FX_Sqrt(L2 * L2 + D * D);
	SPC->L2 = FX_Sqrt(L3 * L3 + D * D);
	// initial angle of t4
	SPC->Ang1 = FX_ATan2(D, L2) * FXARM_R2D;
	SPC->Ang2 = FX_ATan2(D, L3) * FXARM_R2D;
	SPC->Angt = 180.0 - SPC->Ang1 - SPC->Ang2;
	SPC->m_J4_Bound = -SPC->Ang1;

	SPC->cart_len = FX_Sqrt(SPC->L1 * SPC->L1 + SPC->L2 * SPC->L2);

	printf("EG:DH[0]=[%lf %lf %lf %lf]\n", DH[0][0], DH[0][1], DH[0][2], DH[0][3]);
	return FX_TRUE;
}



FX_BOOL  FX_Robot_Init_Kine(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4])
{
	FX_LOG_INFO("[FxRobot - FX_Robot_Init_Kine]\n");

	FX_Robot* pRobot;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&(m_Robot[RobotSerial]);
	
	if (pRobot->m_RobotType == FX_ROBOT_TYPE_DL)
	{
		FX_INT32L i;
		Matrix4  pg;
		FX_DOUBLE jv[7] = {0};
		for ( i = 0; i < 7; i++)
		{
			pRobot->m_RobotDH[i][0] = DH[i][0];
			pRobot->m_RobotDH[i][1] = DH[i][1];
			pRobot->m_RobotDH[i][2] = DH[i][2];
			pRobot->m_RobotDH[i][3] = DH[i][3];
		}
		if (FX_Init_Robot_Kine_DL(RobotSerial, DH) == FX_FALSE)
		{
			return FX_FALSE;
		}
		pRobot->m_RobotDOF = 6;
		FX_Robot_Kine_FK(RobotSerial,jv,pg);
		return FX_TRUE;
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_SRS)
	{
		FX_INT32L i;
		Matrix4  pg;
		FX_DOUBLE jv[7] = { 0 };
		for (i = 0; i < 8; i++)
		{
			pRobot->m_RobotDH[i][0] = DH[i][0];
			pRobot->m_RobotDH[i][1] = DH[i][1];
			pRobot->m_RobotDH[i][2] = DH[i][2];
			pRobot->m_RobotDH[i][3] = DH[i][3];
		}
		if (FX_Init_Robot_Kine_Pilot_SRS(RobotSerial, DH) == FX_FALSE)
		{
			return FX_FALSE;
		}
		pRobot->m_RobotDOF = 7;
		FX_Robot_Kine_FK(RobotSerial, jv, pg);
		return FX_TRUE;
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_CCS)
	{
		FX_INT32L i;
		Matrix4  pg;
		FX_DOUBLE jv[7] = { 0 };
		for (i = 0; i < 8; i++)
		{
			pRobot->m_RobotDH[i][0] = DH[i][0];
			pRobot->m_RobotDH[i][1] = DH[i][1];
			pRobot->m_RobotDH[i][2] = DH[i][2];
			pRobot->m_RobotDH[i][3] = DH[i][3];
		}
		if (FX_Init_Robot_Kine_Pilot_CCS(RobotSerial, DH) == FX_FALSE)
		{
			return FX_FALSE;
		}
		pRobot->m_RobotDOF = 7;
		FX_Robot_Kine_FK(RobotSerial, jv, pg);
		return FX_TRUE;
	}
	else
	{
		return FX_FALSE;
	}
	return FX_FALSE;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

FX_BOOL  FX_Init_Robot_Lmt_DL(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4])
{
	FX_INT32L i;
	FX_Robot* pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	for ( i = 0; i < pRobot->m_RobotDOF; i++)
	{
		pRobot->m_Lmt.m_JLmtPos_P[i] = PNVA[i][0];
		pRobot->m_Lmt.m_JLmtPos_N[i] = PNVA[i][1];
		pRobot->m_Lmt.m_JLmtVel[i] = PNVA[i][2];
		pRobot->m_Lmt.m_JLmtAcc[i] = PNVA[i][3];
	}
	return FX_TRUE;
}

FX_BOOL  FX_Init_Robot_Lmt_SRS(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4])
{
    FX_LOG_INFO("[FxRobot - FX_Init_Robot_Lmt_SRS]\n");
	FX_INT32L i;
	FX_Robot* pRobot;
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	for (i = 0; i < pRobot->m_RobotDOF; i++)
	{
		pRobot->m_Lmt.m_JLmtPos_P[i] = PNVA[i][0];
		pRobot->m_Lmt.m_JLmtPos_N[i] = PNVA[i][1];
		pRobot->m_Lmt.m_JLmtVel[i] = PNVA[i][2];
		pRobot->m_Lmt.m_JLmtAcc[i] = PNVA[i][3];
	}

	printf("EG:PNVA[0]=[%lf %lf %lf %lf ]\n", PNVA[0][0], PNVA[0][1], PNVA[0][2], PNVA[0][3]);
	return FX_TRUE;
}


FX_BOOL  FX_Init_Robot_Lmt_CCS(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4], FX_DOUBLE J67[4][3])
{
	FX_LOG_INFO("[FxRobot - FX_Init_Robot_Lmt_CCS]\n");

	FX_INT32L i;
	FX_KineSPC_Pilot* SPC;
	FX_Robot* pRobot;
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	for (i = 0; i < pRobot->m_RobotDOF; i++)
	{
		pRobot->m_Lmt.m_JLmtPos_P[i] = PNVA[i][0];
		pRobot->m_Lmt.m_JLmtPos_N[i] = PNVA[i][1];
		pRobot->m_Lmt.m_JLmtVel[i] = PNVA[i][2];
		pRobot->m_Lmt.m_JLmtAcc[i] = PNVA[i][3];
	}

	SPC = (FX_KineSPC_Pilot*)(pRobot->m_KineSPC);

	for (i = 0; i < 3; i++)
	{
		SPC->lmtj67_pp[i] = J67[0][i];
		SPC->lmtj67_np[i] = J67[1][i];
		SPC->lmtj67_nn[i] = J67[2][i];
		SPC->lmtj67_pn[i] = J67[3][i];
	}

	printf("EG:PNVA[0]=[%lf %lf %lf %lf ]\n", PNVA[0][0], PNVA[0][1], PNVA[0][2], PNVA[0][3]);
	return FX_TRUE;
}

FX_BOOL  FX_Robot_Init_Lmt(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4], FX_DOUBLE J67[4][3])
{
	FX_LOG_INFO("[FxRobot - FX_Robot_Init_Lmt]\n");

	FX_Robot* pRobot;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	if (pRobot->m_RobotType == FX_ROBOT_TYPE_DL)
	{
		return FX_Init_Robot_Lmt_DL(RobotSerial, PNVA);
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_SRS)
	{
		return FX_Init_Robot_Lmt_SRS(RobotSerial, PNVA);
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_CCS)
	{
		return FX_Init_Robot_Lmt_CCS(RobotSerial, PNVA, J67);
	}
	else
	{
		return FX_FALSE;
	}
	return FX_FALSE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


FX_VOID FX_XYZMRot(FX_DOUBLE L[4][4], FX_DOUBLE cosv, FX_DOUBLE sinv, FX_DOUBLE T[4][4])
{
	FX_INT32L i;
	for (i = 0; i < 3; i++)
	{
		T[i][0] = L[i][0] * cosv + L[i][1] * sinv;
		T[i][1] = -L[i][0] * sinv + L[i][1] * cosv;
		T[i][2] = L[i][2];
		T[i][3] = L[i][3];
	}
	T[3][0] = 0;
	T[3][1] = 0;
	T[3][2] = 0;
	T[3][3] = 1;
}


FX_BOOL  FX_Robot_Kine_Piolt(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_DOUBLE pgos[4][4])
{
	FX_LOG_INFO("[FxRobot - FX_Robot_Kine_Piolt]\n");

	long i;
	FX_Robot* pRobot;
	FX_DOUBLE cosv, sinv;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];

	for (i = 0; i < 7; i++)
	{
		FX_SIN_COS_DEG(joints[i], &sinv, &cosv);
		FX_XYZMRot(pRobot->m_KineBase.m_AxisRotBase[i], cosv, sinv, pRobot->m_KineBase.m_AxisRotTip[i]);
	}

	FX_M44Copy(pRobot->m_KineBase.m_AxisRotTip[0], pRobot->m_KineBase.m_JointPG[0]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[0], pRobot->m_KineBase.m_AxisRotTip[1], pRobot->m_KineBase.m_JointPG[1]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[1], pRobot->m_KineBase.m_AxisRotTip[2], pRobot->m_KineBase.m_JointPG[2]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[2], pRobot->m_KineBase.m_AxisRotTip[3], pRobot->m_KineBase.m_JointPG[3]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[3], pRobot->m_KineBase.m_AxisRotTip[4], pRobot->m_KineBase.m_JointPG[4]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[4], pRobot->m_KineBase.m_AxisRotTip[5], pRobot->m_KineBase.m_JointPG[5]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[5], pRobot->m_KineBase.m_AxisRotTip[6], pRobot->m_KineBase.m_JointPG[6]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[6], pRobot->m_KineBase.m_Flange, pRobot->m_KineBase.m_FlangeTip);
	FX_PGMult(pRobot->m_KineBase.m_FlangeTip, pRobot->m_KineBase.m_Tool, pRobot->m_KineBase.m_TCP);
	FX_M44Copy(pRobot->m_KineBase.m_TCP, pgos);

	return FX_TRUE;
}

FX_BOOL  FX_Robot_Kine_Piolt_NSPG(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_DOUBLE pgos[4][4], Matrix3 nspg)
{
	long i;
	FX_Robot* pRobot;
	FX_DOUBLE cosv, sinv;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];


	for (i = 0; i < 7; i++)
	{
		FX_SIN_COS_DEG(joints[i], &sinv, &cosv);
		FX_XYZMRot(pRobot->m_KineBase.m_AxisRotBase[i], cosv, sinv, pRobot->m_KineBase.m_AxisRotTip[i]);
	}

	FX_M44Copy(pRobot->m_KineBase.m_AxisRotTip[0], pRobot->m_KineBase.m_JointPG[0]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[0], pRobot->m_KineBase.m_AxisRotTip[1], pRobot->m_KineBase.m_JointPG[1]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[1], pRobot->m_KineBase.m_AxisRotTip[2], pRobot->m_KineBase.m_JointPG[2]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[2], pRobot->m_KineBase.m_AxisRotTip[3], pRobot->m_KineBase.m_JointPG[3]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[3], pRobot->m_KineBase.m_AxisRotTip[4], pRobot->m_KineBase.m_JointPG[4]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[4], pRobot->m_KineBase.m_AxisRotTip[5], pRobot->m_KineBase.m_JointPG[5]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[5], pRobot->m_KineBase.m_AxisRotTip[6], pRobot->m_KineBase.m_JointPG[6]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[6], pRobot->m_KineBase.m_Flange, pRobot->m_KineBase.m_FlangeTip);
	FX_PGMult(pRobot->m_KineBase.m_FlangeTip, pRobot->m_KineBase.m_Tool, pRobot->m_KineBase.m_TCP);
	FX_M44Copy(pRobot->m_KineBase.m_TCP, pgos);

	{
		Vect3 zdir;
		Vect3 xdir;

		zdir[0] = pRobot->m_KineBase.m_JointPG[6][0][3] - pRobot->m_KineBase.m_JointPG[1][0][3];
		zdir[1] = pRobot->m_KineBase.m_JointPG[6][1][3] - pRobot->m_KineBase.m_JointPG[1][1][3];
		zdir[2] = pRobot->m_KineBase.m_JointPG[6][2][3] - pRobot->m_KineBase.m_JointPG[1][2][3];

		xdir[0] = pRobot->m_KineBase.m_JointPG[2][0][0];
		xdir[1] = pRobot->m_KineBase.m_JointPG[2][1][0];
		xdir[2] = pRobot->m_KineBase.m_JointPG[2][2][0];


		if (FX_MatrixNormZX(zdir, xdir, nspg) == FX_FALSE)
		{
			return FX_FALSE;
		}
	}


	return FX_TRUE;
}

FX_BOOL  FX_Robot_Kine_DL(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_DOUBLE pgos[4][4])
{

	long i;
	FX_Robot* pRobot;
	FX_DOUBLE cosv, sinv;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];

	for (i = 0; i < 6; i++)
	{
		FX_SIN_COS_DEG(joints[i], &sinv, &cosv);
		FX_XYZMRot(pRobot->m_KineBase.m_AxisRotBase[i], cosv, sinv, pRobot->m_KineBase.m_AxisRotTip[i]);
	}

	FX_M44Copy(pRobot->m_KineBase.m_AxisRotTip[0], pRobot->m_KineBase.m_JointPG[0]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[0], pRobot->m_KineBase.m_AxisRotTip[1], pRobot->m_KineBase.m_JointPG[1]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[1], pRobot->m_KineBase.m_AxisRotTip[2], pRobot->m_KineBase.m_JointPG[2]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[2], pRobot->m_KineBase.m_AxisRotTip[3], pRobot->m_KineBase.m_JointPG[3]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[3], pRobot->m_KineBase.m_AxisRotTip[4], pRobot->m_KineBase.m_JointPG[4]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[4], pRobot->m_KineBase.m_AxisRotTip[5], pRobot->m_KineBase.m_JointPG[5]);
	FX_PGMult(pRobot->m_KineBase.m_JointPG[5], pRobot->m_KineBase.m_Flange, pRobot->m_KineBase.m_FlangeTip);
	FX_PGMult(pRobot->m_KineBase.m_FlangeTip, pRobot->m_KineBase.m_Tool, pRobot->m_KineBase.m_TCP);
	FX_M44Copy(pRobot->m_KineBase.m_TCP, pgos);
	return FX_TRUE;
}

FX_BOOL  FX_Robot_Tool_Set(FX_INT32L RobotSerial, Matrix4 tool)
{
	FX_LOG_INFO("[FxRobot - FX_Robot_Tool_Set]\n");

	FX_INT32 i;
	FX_INT32 j;
	FX_Robot* pRobot;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 4; j++)
		{
			pRobot->m_KineBase.m_Tool[i][j] = tool[i][j];
		}
	}
	pRobot->m_KineBase.m_Tool[3][0] = 0;
	pRobot->m_KineBase.m_Tool[3][1] = 0;
	pRobot->m_KineBase.m_Tool[3][2] = 0;
	pRobot->m_KineBase.m_Tool[3][3] = 1;
	FX_PGMatrixInv(pRobot->m_KineBase.m_Tool, pRobot->m_KineBase.m_InvTool);

	printf("EG:TOOL=[%lf %lf %lf %lf\n %lf %lf %lf %lf\n %lf %lf %lf %lf\n %lf %lf %lf %lf]\n",
		pRobot->m_KineBase.m_Tool[0][0], pRobot->m_KineBase.m_Tool[0][1], pRobot->m_KineBase.m_Tool[0][2], pRobot->m_KineBase.m_Tool[0][3],
		pRobot->m_KineBase.m_Tool[1][0], pRobot->m_KineBase.m_Tool[1][1], pRobot->m_KineBase.m_Tool[1][2], pRobot->m_KineBase.m_Tool[1][3],
		pRobot->m_KineBase.m_Tool[2][0], pRobot->m_KineBase.m_Tool[2][1], pRobot->m_KineBase.m_Tool[2][2], pRobot->m_KineBase.m_Tool[2][3],
		pRobot->m_KineBase.m_Tool[3][0], pRobot->m_KineBase.m_Tool[3][1], pRobot->m_KineBase.m_Tool[3][2], pRobot->m_KineBase.m_Tool[3][3]);
	return FX_TRUE;
}

FX_BOOL  FX_Robot_Tool_Rmv(FX_INT32L RobotSerial)
{
	FX_LOG_INFO("[FxRobot - FX_Robot_Tool_Rmv]\n");

	FX_Robot* pRobot;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];

	FX_IdentM44(pRobot->m_KineBase.m_Tool);
	FX_IdentM44(pRobot->m_KineBase.m_InvTool);

	printf("EG:TOOL=[%lf %lf %lf %lf\n %lf %lf %lf %lf\n %lf %lf %lf %lf\n %lf %lf %lf %lf]\n",
		pRobot->m_KineBase.m_Tool[0][0], pRobot->m_KineBase.m_Tool[0][1], pRobot->m_KineBase.m_Tool[0][2], pRobot->m_KineBase.m_Tool[0][3],
		pRobot->m_KineBase.m_Tool[1][0], pRobot->m_KineBase.m_Tool[1][1], pRobot->m_KineBase.m_Tool[1][2], pRobot->m_KineBase.m_Tool[1][3],
		pRobot->m_KineBase.m_Tool[2][0], pRobot->m_KineBase.m_Tool[2][1], pRobot->m_KineBase.m_Tool[2][2], pRobot->m_KineBase.m_Tool[2][3],
		pRobot->m_KineBase.m_Tool[3][0], pRobot->m_KineBase.m_Tool[3][1], pRobot->m_KineBase.m_Tool[3][2], pRobot->m_KineBase.m_Tool[3][3]);
	return FX_TRUE;
}

FX_BOOL  FX_Robot_Kine_FK(FX_INT32L RobotSerial, FX_DOUBLE joints[7], Matrix4 pgos)
{
	FX_Robot* pRobot;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	if (pRobot->m_RobotType == FX_ROBOT_TYPE_DL)
	{
		return FX_Robot_Kine_DL(RobotSerial, joints, pgos);
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_SRS)
	{
		return FX_Robot_Kine_Piolt(RobotSerial, joints, pgos);
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_CCS)
	{
		return FX_Robot_Kine_Piolt(RobotSerial, joints, pgos);
	}
	else
	{
		return FX_FALSE;
	}
	return FX_FALSE;
}

FX_BOOL  FX_Robot_Kine_FK_NSP(FX_INT32L RobotSerial, FX_DOUBLE joints[7], Matrix4 pgos, Matrix3 nspg)
{
	FX_Robot* pRobot;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	FX_IdentM33(nspg);
	if (pRobot->m_RobotType == FX_ROBOT_TYPE_DL)
	{
		return FX_Robot_Kine_DL(RobotSerial, joints, pgos);
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_SRS)
	{
		return FX_Robot_Kine_Piolt_NSPG(RobotSerial, joints, pgos, nspg);
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_CCS)
	{
		return FX_Robot_Kine_Piolt_NSPG(RobotSerial, joints, pgos, nspg);
	}
	else
	{
		return FX_FALSE;
	}
	return FX_FALSE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

FX_BOOL  FX_Jacb_ROT_7(FX_INT32L RobotSerial, FX_DOUBLE jcb[6][7])
{
	FX_INT32 i;
	FX_DOUBLE Vm[3];
	FX_DOUBLE Vr[3];
	FX_DOUBLE V[3];
	FX_Robot* pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	for (i = 0; i < 7; i++)
	{
		Vm[0] = pRobot->m_KineBase.m_TCP[0][3] - pRobot->m_KineBase.m_JointPG[i][0][3];
		Vm[1] = pRobot->m_KineBase.m_TCP[1][3] - pRobot->m_KineBase.m_JointPG[i][1][3];
		Vm[2] = pRobot->m_KineBase.m_TCP[2][3] - pRobot->m_KineBase.m_JointPG[i][2][3];

		Vr[0] = pRobot->m_KineBase.m_JointPG[i][0][2];
		Vr[1] = pRobot->m_KineBase.m_JointPG[i][1][2];
		Vr[2] = pRobot->m_KineBase.m_JointPG[i][2][2];

		FX_VectCross(Vr, Vm, V);

		jcb[0][i] = V[0] * 0.001;
		jcb[1][i] = V[1] * 0.001;
		jcb[2][i] = V[2] * 0.001;
		jcb[3][i] = Vr[0];
		jcb[4][i] = Vr[1];
		jcb[5][i] = Vr[2];
	}	

	printf("EG:jcb[0]=[%lf %lf %lf %lf %lf %lf %lf]\n", jcb[0][0], jcb[0][1], jcb[0][2], jcb[0][3], jcb[0][4], jcb[0][5], jcb[0][6]);
	return FX_TRUE;
}


FX_BOOL  FX_Jacb_ROT_6(FX_INT32L RobotSerial, FX_DOUBLE jcb[6][6])
{
	FX_INT32 i;
	FX_DOUBLE Vm[3];
	FX_DOUBLE Vr[3];
	FX_DOUBLE V[3];
	FX_Robot* pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	for (i = 0; i < 6; i++)
	{
		Vm[0] = pRobot->m_KineBase.m_TCP[0][3] - pRobot->m_KineBase.m_JointPG[i][0][3];
		Vm[1] = pRobot->m_KineBase.m_TCP[1][3] - pRobot->m_KineBase.m_JointPG[i][1][3];
		Vm[2] = pRobot->m_KineBase.m_TCP[2][3] - pRobot->m_KineBase.m_JointPG[i][2][3];

		Vr[0] = pRobot->m_KineBase.m_JointPG[i][0][2];
		Vr[1] = pRobot->m_KineBase.m_JointPG[i][1][2];
		Vr[2] = pRobot->m_KineBase.m_JointPG[i][2][2];

		FX_VectCross(Vr, Vm, V);

		jcb[0][i] = V[0] * 0.001;
		jcb[1][i] = V[1] * 0.001;
		jcb[2][i] = V[2] * 0.001;
		jcb[3][i] = Vr[0];
		jcb[4][i] = Vr[1];
		jcb[5][i] = Vr[2];
	}
	return FX_TRUE;
}

FX_BOOL  FX_JacbAxis7(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_DOUBLE jcb[6][7])
{
	FX_Robot* pRobot = (FX_Robot*)&m_Robot[RobotSerial]; 
	
	if (	pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_SRS 
		||  pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_CCS)
	{
		Matrix4 pg;
		if (FX_Robot_Kine_FK(RobotSerial, joints, pg) == FX_FALSE)
		{
			return FX_FALSE;
		}
		return FX_Jacb_ROT_7(RobotSerial, jcb);
	}
	
	return FX_FALSE;
}


FX_BOOL  FX_JacbAixs6(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_DOUBLE jcb[6][6])
{
	FX_Robot* pRobot = (FX_Robot*)&m_Robot[RobotSerial];

	if (pRobot->m_RobotType == FX_ROBOT_TYPE_DL)
	{
		Matrix4 pg;
		if (FX_Robot_Kine_FK(RobotSerial, joints, pg) == FX_FALSE)
		{
			return FX_FALSE;
		}
		return FX_Jacb_ROT_6(RobotSerial, jcb);
	}
	return FX_FALSE;
}

FX_BOOL  FX_Robot_Kine_Jacb(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_Jacobi *jcb)
{
	FX_LOG_INFO("[FxRobot - FX_Robot_Kine_Jacb]\n");

	FX_Robot* pRobot;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_SRS
		|| pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_CCS)
	{
		jcb->m_AxisNum = 7;
		return FX_JacbAxis7(RobotSerial, joints, jcb->m_Jcb);
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_DL)
	{
		FX_INT32L i, j;
		FX_DOUBLE tmp_jcb[6][6];
		jcb->m_AxisNum = 6;
		if (FX_JacbAixs6(RobotSerial, joints, tmp_jcb) == FX_FALSE)
		{
			return FX_FALSE;
		}
		for ( i = 0; i < 6; i++)
		{
			for (j = 0; j < 6; j++) 
			{
				jcb->m_Jcb[i][j] = tmp_jcb[i][j];
			}
			jcb->m_Jcb[i][6] = 0;
		}
		return FX_TRUE;
	}
	else
	{
		return FX_FALSE;
	}
	return FX_FALSE;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


FX_BOOL  FX_Jacb_Dot_ROT_7(FX_INT32L RobotSerial, FX_DOUBLE jvel[7], FX_DOUBLE jcb_dot[6][7])
{
	FX_INT32 i;
	FX_INT32 dof = 7;
	FX_DOUBLE e[8][3] =  { {0},{0},{0},{0},{0},{0},{0},{0} };
	FX_DOUBLE w[8][3] =  { {0},{0},{0},{0},{0},{0},{0},{0} };
	FX_DOUBLE ed[8][3] =  { {0},{0},{0},{0},{0},{0},{0},{0} };
	FX_DOUBLE p[8][3] =  { {0},{0},{0},{0},{0},{0},{0},{0} };
	FX_DOUBLE pd[8][3] =  { {0},{0},{0},{0},{0},{0},{0},{0} };
	FX_DOUBLE v3tmp1[3] = { 0 };
	FX_DOUBLE v3tmp2[3] = { 0 };
	FX_Robot* pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	for (i = 0; i < dof; i++)
	{
		e[i][0] = pRobot->m_KineBase.m_JointPG[i][0][2];
		e[i][1] = pRobot->m_KineBase.m_JointPG[i][1][2];
		e[i][2] = pRobot->m_KineBase.m_JointPG[i][2][2];
	}

	w[0][0] = e[0][0] * jvel[0] * FXARM_D2R;
	w[0][1] = e[0][1] * jvel[0] * FXARM_D2R;
	w[0][2] = e[0][2] * jvel[0] * FXARM_D2R;

	for (i = 1; i < dof; i++)
	{
		w[i][0] = w[i - 1][0] + e[i][0] * jvel[i] * FXARM_D2R;
		w[i][1] = w[i - 1][1] + e[i][1] * jvel[i] * FXARM_D2R;
		w[i][2] = w[i - 1][2] + e[i][2] * jvel[i] * FXARM_D2R;
	}

	for (i = 0; i < dof; i++)
	{
		FX_VectCross(w[i], e[i], ed[i]);
	}

	for (i = 0; i < dof - 1; i++)
	{
		p[i][0] = (pRobot->m_KineBase.m_JointPG[i + 1][0][3] - pRobot->m_KineBase.m_JointPG[i][0][3]) * 0.001;
		p[i][1] = (pRobot->m_KineBase.m_JointPG[i + 1][1][3] - pRobot->m_KineBase.m_JointPG[i][1][3]) * 0.001;
		p[i][2] = (pRobot->m_KineBase.m_JointPG[i + 1][2][3] - pRobot->m_KineBase.m_JointPG[i][2][3]) * 0.001;
	}

	p[dof - 1][0] = (pRobot->m_KineBase.m_TCP[0][3] - pRobot->m_KineBase.m_JointPG[dof - 1][0][3]) * 0.001;
	p[dof - 1][1] = (pRobot->m_KineBase.m_TCP[1][3] - pRobot->m_KineBase.m_JointPG[dof - 1][1][3]) * 0.001;
	p[dof - 1][2] = (pRobot->m_KineBase.m_TCP[2][3] - pRobot->m_KineBase.m_JointPG[dof - 1][2][3]) * 0.001;


	FX_VectCross(p[dof - 1], w[dof - 1], pd[dof - 1]);
	for (i = dof - 2; i >= 0; i--)
	{
		FX_VectCross(p[i], w[i], pd[i]);
		pd[i][0] += pd[i + 1][0];
		pd[i][1] += pd[i + 1][1];
		pd[i][2] += pd[i + 1][2];
	}


	for (i = 0; i < dof; i++)
	{
		FX_VectCross(pd[i], e[i], v3tmp1);
		FX_VectCross(p[i], ed[i], v3tmp2);

		jcb_dot[0][i] = (v3tmp1[0] + v3tmp2[0]);
		jcb_dot[1][i] = (v3tmp1[1] + v3tmp2[1]);
		jcb_dot[2][i] = (v3tmp1[2] + v3tmp2[2]);

		jcb_dot[3][i] = ed[i][0];
		jcb_dot[4][i] = ed[i][1];
		jcb_dot[5][i] = ed[i][2];
	}
    printf("EG:jcb_dot[0]=%lf %lf %lf %lf %lf %lf %lf\n",jcb_dot[0][0],jcb_dot[0][1],jcb_dot[0][2],jcb_dot[0][3],jcb_dot[0][4],jcb_dot[0][5],jcb_dot[0][6]);
	return FX_TRUE;
}


FX_BOOL  FX_Jacb_Dot_ROT_6(FX_INT32L RobotSerial, FX_DOUBLE jvel[7], FX_DOUBLE jcb_dot[6][6])
{
	FX_INT32 i;
	FX_INT32 dof = 6;
	FX_DOUBLE e[8][3] =  { {0},{0},{0},{0},{0},{0},{0},{0} };
	FX_DOUBLE w[8][3] =  { {0},{0},{0},{0},{0},{0},{0},{0} };
	FX_DOUBLE ed[8][3] = { {0},{0},{0},{0},{0},{0},{0},{0} };
	FX_DOUBLE p[8][3] =  { {0},{0},{0},{0},{0},{0},{0},{0} };
	FX_DOUBLE pd[8][3] = { {0},{0},{0},{0},{0},{0},{0},{0} };
	FX_DOUBLE v3tmp1[3] = { 0 };
	FX_DOUBLE v3tmp2[3] = { 0 };
	FX_Robot* pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	for (i = 0; i < dof; i++)
	{
		e[i][0] = pRobot->m_KineBase.m_JointPG[i][0][2];
		e[i][1] = pRobot->m_KineBase.m_JointPG[i][1][2];
		e[i][2] = pRobot->m_KineBase.m_JointPG[i][2][2];
	}

	w[0][0] = e[0][0] * jvel[0] * FXARM_D2R;
	w[0][1] = e[0][1] * jvel[0] * FXARM_D2R;
	w[0][2] = e[0][2] * jvel[0] * FXARM_D2R;

	for (i = 1; i < dof; i++)
	{
		w[i][0] = w[i - 1][0] + e[i][0] * jvel[i] * FXARM_D2R;
		w[i][1] = w[i - 1][1] + e[i][1] * jvel[i] * FXARM_D2R;
		w[i][2] = w[i - 1][2] + e[i][2] * jvel[i] * FXARM_D2R;
	}

	for (i = 0; i < dof; i++)
	{
		FX_VectCross(w[i], e[i], ed[i]);
	}

	for (i = 0; i < dof - 1; i++)
	{
		p[i][0] = (pRobot->m_KineBase.m_JointPG[i + 1][0][3] - pRobot->m_KineBase.m_JointPG[i][0][3]) * 0.001;
		p[i][1] = (pRobot->m_KineBase.m_JointPG[i + 1][1][3] - pRobot->m_KineBase.m_JointPG[i][1][3]) * 0.001;
		p[i][2] = (pRobot->m_KineBase.m_JointPG[i + 1][2][3] - pRobot->m_KineBase.m_JointPG[i][2][3]) * 0.001;
	}

	p[dof - 1][0] = (pRobot->m_KineBase.m_TCP[0][3] - pRobot->m_KineBase.m_JointPG[dof - 1][0][3]) * 0.001;
	p[dof - 1][1] = (pRobot->m_KineBase.m_TCP[1][3] - pRobot->m_KineBase.m_JointPG[dof - 1][1][3]) * 0.001;
	p[dof - 1][2] = (pRobot->m_KineBase.m_TCP[2][3] - pRobot->m_KineBase.m_JointPG[dof - 1][2][3]) * 0.001;

	

	FX_VectCross(p[dof - 1], w[dof - 1], pd[dof - 1]);
	for (i = dof - 2; i >= 0; i--)
	{
		FX_VectCross(p[i], w[i], pd[i]);
		pd[i][0] += pd[i + 1][0];
		pd[i][1] += pd[i + 1][1];
		pd[i][2] += pd[i + 1][2];
	}


	for (i = 0; i < dof; i++)
	{
		FX_VectCross(pd[i], e[i], v3tmp1);
		FX_VectCross(p[i], ed[i], v3tmp2);

		jcb_dot[0][i] = (v3tmp1[0] + v3tmp2[0]);
		jcb_dot[1][i] = (v3tmp1[1] + v3tmp2[1]);
		jcb_dot[2][i] = (v3tmp1[2] + v3tmp2[2]);

		jcb_dot[3][i] = ed[i][0];
		jcb_dot[4][i] = ed[i][1];
		jcb_dot[5][i] = ed[i][2];
	}
	return FX_TRUE;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FX_BOOL  FX_InvKine_DL(FX_INT32L RobotSerial, FX_InvKineSolvePara * solve_para)
{
	return FX_FALSE;
}

FX_INT32  FX_GetJ4Type_Pilot_G( FX_INT32L RobotSerial,FX_DOUBLE jv4)
{
	FX_KineSPC_Pilot* SPC;
	FX_Robot* pRobot;
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	SPC = (FX_KineSPC_Pilot*)(pRobot->m_KineSPC);


	if (jv4 > SPC->m_J4_Bound + 0.00001)
	{
		return 1;
	}
	if (jv4 < SPC->m_J4_Bound - 0.00001)
	{
		return -1;
	}
	return 0;
}


FX_INT32  FX_GetJ4Type_Pilot(FX_KineSPC_Pilot* SPC, FX_DOUBLE jv4)
{
	if (jv4 > SPC->m_J4_Bound + 0.00001)
	{
		return 1;
	}
	if (jv4 < SPC->m_J4_Bound - 0.00001)
	{
		return -1;
	}
	return 0;
}


FX_BOOL  FX_SolveTrange2D(FX_DOUBLE x_pan, FX_DOUBLE r1x, FX_DOUBLE r1y, FX_DOUBLE r2x, FX_DOUBLE r2y, FX_DOUBLE ret_xy[2])
{
	FX_DOUBLE k1;
	FX_DOUBLE k2;

	if (FX_Fabs(r1x) < 0.00000001)
	{
		ret_xy[0] = 0;
		k2 = r2y / r2x;
		ret_xy[1] = -x_pan / k2;
		return FX_TRUE;
	}

	if (FX_Fabs(r2x) < 0.00000001)
	{
		ret_xy[0] = x_pan;
		k1 = r1y / r1x;
		ret_xy[1] = x_pan / k1;
		return FX_TRUE;
	}

	k1 = r1y / r1x;
	k2 = r2y / r2x;

	if (FX_Fabs(k1) < 0.01570731731182067575329535330991)
	{
		return FX_FALSE;
	}



	ret_xy[0] = -x_pan * k2 / (k1 - k2);
	ret_xy[1] = ret_xy[0] * k1;
	return FX_TRUE;
}

FX_BOOL FX_SolveJ123ZNYZ(NSPBase* pnspbase, FX_DOUBLE rot_angle, FX_DOUBLE ref[7], FX_DOUBLE ret_j[7], FX_BOOL* is_degen,FX_DOUBLE dgr)
{
	FX_DOUBLE m[3][3];

	FX_DOUBLE jtmp1[3];
	FX_DOUBLE jtmp2[3];
	FX_DOUBLE j[3];
	FX_DOUBLE j1[3];
	FX_DOUBLE j2[3];

	FX_DOUBLE udgr = dgr;
	if (udgr < 0.05)
	{
		udgr = 0.05;
	}
	if (udgr > 10)
	{
		udgr = 10;
	}
	FX_MatRotAxis(pnspbase->rot_axis, rot_angle, pnspbase->j123Base, m);
	if (FX_Matrix2ZYZ_DGR(m, udgr, j) == FX_TRUE)
	{
		FX_DOUBLE err1;
		FX_DOUBLE err2;
		*is_degen = FX_FALSE;
		j1[0] = j[0];
		j1[1] = -j[1];
		j1[2] = j[2];


		j2[0] = j[0] + 180;
		j2[1] = j[1];
		j2[2] = j[2] + 180;

		err1 = FX_MinDif_Circle(ref[0], &j1[0])
			+ FX_MinDif_Circle(ref[1], &j1[1])
			+ FX_MinDif_Circle(ref[2], &j1[2]);
		err2 = FX_MinDif_Circle(ref[0], &j2[0])
			+ FX_MinDif_Circle(ref[1], &j2[1])
			+ FX_MinDif_Circle(ref[2], &j2[2]);

		if (err1 <= err2)
		{
			ret_j[0] = j1[0];
			ret_j[1] = j1[1];
			ret_j[2] = j1[2];
		}
		else
		{
			ret_j[0] = j2[0];
			ret_j[1] = j2[1];
			ret_j[2] = j2[2];
		}
		return FX_TRUE;
	}

	*is_degen = FX_TRUE;

	{
		FX_DOUBLE err1;
		FX_DOUBLE err2;
		FX_MatRotAxis(pnspbase->rot_axis, rot_angle + 1, pnspbase->j123Base, m);

		if (FX_Matrix2ZYZ_DGR(m, udgr, j) == FX_FALSE)
		{
			return FX_FALSE;
		}

		j1[0] = j[0];
		j1[1] = -j[1];
		j1[2] = j[2];


		j2[0] = j[0] + 180;
		j2[1] = j[1];
		j2[2] = j[2] + 180;

		err1 = FX_MinDif_Circle(ref[0], &j1[0])
			+ FX_MinDif_Circle(ref[1], &j1[1])
			+ FX_MinDif_Circle(ref[2], &j1[2]);
		err2 = FX_MinDif_Circle(ref[0], &j2[0])
			+ FX_MinDif_Circle(ref[1], &j2[1])
			+ FX_MinDif_Circle(ref[2], &j2[2]);

		if (err1 <= err2)
		{
			jtmp1[0] = j1[0];
			jtmp1[1] = j1[1];
			jtmp1[2] = j1[2];
		}
		else
		{
			jtmp1[0] = j2[0];
			jtmp1[1] = j2[1];
			jtmp1[2] = j2[2];

		}

	}



	{
		FX_DOUBLE err1;
		FX_DOUBLE err2;
		FX_MatRotAxis(pnspbase->rot_axis, rot_angle - 1, pnspbase->j123Base, m);

		if (FX_Matrix2ZYZ_DGR(m, udgr, j) == FX_FALSE)
		{
			return FX_FALSE;
		}

		j1[0] = j[0];
		j1[1] = -j[1];
		j1[2] = j[2];


		j2[0] = j[0] + 180;
		j2[1] = j[1];
		j2[2] = j[2] + 180;

		err1 = FX_MinDif_Circle(ref[0], &j1[0])
			+ FX_MinDif_Circle(ref[1], &j1[1])
			+ FX_MinDif_Circle(ref[2], &j1[2]);
		err2 = FX_MinDif_Circle(ref[0], &j2[0])
			+ FX_MinDif_Circle(ref[1], &j2[1])
			+ FX_MinDif_Circle(ref[2], &j2[2]);

		if (err1 <= err2)
		{
			jtmp2[0] = j1[0];
			jtmp2[1] = j1[1];
			jtmp2[2] = j1[2];
		}
		else
		{

			jtmp2[0] = j2[0];
			jtmp2[1] = j2[1];
			jtmp2[2] = j2[2];
		}

	}

	ret_j[0] = (jtmp1[0] + jtmp2[0]) * 0.5;
	ret_j[1] = (jtmp1[1] + jtmp2[1]) * 0.5;
	ret_j[2] = (jtmp1[2] + jtmp2[2]) * 0.5;

	return FX_TRUE;

}



FX_BOOL FX_SolveJ567ZNYZ(NSPBase* pnspbase, FX_DOUBLE rot_angle, FX_DOUBLE ref[7], FX_DOUBLE ret_j[7], FX_BOOL* is_degen, FX_DOUBLE dgr)
{
	FX_DOUBLE m1[3][3];
	FX_DOUBLE m2[3][3];
	FX_DOUBLE m[3][3];
	FX_DOUBLE j[3];
	FX_DOUBLE j1[3];
	FX_DOUBLE j2[3];
	
	FX_DOUBLE jtmp1[3];
	FX_DOUBLE jtmp2[3];

	FX_DOUBLE udgr = dgr;
	if (udgr < 0.05)
	{
		udgr = 0.05;
	}
	if (udgr > 10)
	{
		udgr = 10;
	}

	FX_MatRotAxis(pnspbase->rot_axis, rot_angle, pnspbase->j567Base, m1);
	FX_M33Trans(m1, m2);
	FX_MMM33(m2, pnspbase->wristges, m);
	

	if (FX_Matrix2ZYZ_DGR(m, udgr, j) == FX_TRUE)
	{
		FX_DOUBLE err1;
		FX_DOUBLE err2;
		*is_degen = FX_FALSE;
		j1[0] = j[0];
		j1[1] = -j[1];
		j1[2] = j[2];


		j2[0] = j[0] + 180;
		j2[1] = j[1];
		j2[2] = j[2] + 180;

		err1 = FX_MinDif_Circle(ref[4], &j1[0])
			+ FX_MinDif_Circle(ref[5], &j1[1])
			+ FX_MinDif_Circle(ref[6], &j1[2]);
		err2 = FX_MinDif_Circle(ref[4], &j2[0])
			+ FX_MinDif_Circle(ref[5], &j2[1])
			+ FX_MinDif_Circle(ref[6], &j2[2]);

		if (err1 <= err2)
		{
			ret_j[4] = j1[0];
			ret_j[5] = j1[1];
			ret_j[6] = j1[2];
		}
		else
		{
			ret_j[4] = j2[0];
			ret_j[5] = j2[1];
			ret_j[6] = j2[2];
		}
		return FX_TRUE;
	}

	*is_degen = FX_TRUE;

	{
		FX_DOUBLE err1;
		FX_DOUBLE err2;
		FX_MatRotAxis(pnspbase->rot_axis, rot_angle + 1, pnspbase->j567Base, m1);
		FX_M33Trans(m1, m2);

		FX_MMM33(m2, pnspbase->wristges, m);

		if (FX_Matrix2ZYZ_DGR(m, udgr, j) == FX_FALSE)
		{
			return FX_FALSE;
		}

		j1[0] = j[0];
		j1[1] = -j[1];
		j1[2] = j[2];


		j2[0] = j[0] + 180;
		j2[1] = j[1];
		j2[2] = j[2] + 180;

		err1 = FX_MinDif_Circle(ref[4], &j1[0])
			+ FX_MinDif_Circle(ref[5], &j1[1])
			+ FX_MinDif_Circle(ref[6], &j1[2]);
		err2 = FX_MinDif_Circle(ref[4], &j2[0])
			+ FX_MinDif_Circle(ref[5], &j2[1])
			+ FX_MinDif_Circle(ref[6], &j2[2]);

		if (err1 <= err2)
		{
			jtmp1[0] = j1[0];
			jtmp1[1] = j1[1];
			jtmp1[2] = j1[2];
		}
		else
		{
			jtmp1[0] = j2[0];
			jtmp1[1] = j2[1];
			jtmp1[2] = j2[2];
		}

	}



	{
		FX_DOUBLE err1;
		FX_DOUBLE err2;
		FX_MatRotAxis(pnspbase->rot_axis, rot_angle + 1, pnspbase->j567Base, m1);
		FX_M33Trans(m1, m2);

		FX_MMM33(m2, pnspbase->wristges, m);

		if (FX_Matrix2ZYZ_DGR(m, udgr, j) == FX_FALSE)
		{
			return FX_FALSE;
		}

		j1[0] = j[0];
		j1[1] = -j[1];
		j1[2] = j[2];


		j2[0] = j[0] + 180;
		j2[1] = j[1];
		j2[2] = j[2] + 180;

		err1 = FX_MinDif_Circle(ref[4], &j1[0])
			+ FX_MinDif_Circle(ref[5], &j1[1])
			+ FX_MinDif_Circle(ref[6], &j1[2]);
		err2 = FX_MinDif_Circle(ref[4], &j2[0])
			+ FX_MinDif_Circle(ref[5], &j2[1])
			+ FX_MinDif_Circle(ref[6], &j2[2]);

		if (err1 <= err2)
		{
			jtmp2[0] = j1[0];
			jtmp2[1] = j1[1];
			jtmp2[2] = j1[2];
		}
		else
		{
			jtmp2[0] = j2[0];
			jtmp2[1] = j2[1];
			jtmp2[2] = j2[2];
		}


	}

	ret_j[4] = (jtmp1[0] + jtmp2[0]) * 0.5;
	ret_j[5] = (jtmp1[1] + jtmp2[1]) * 0.5;
	ret_j[6] = (jtmp1[2] + jtmp2[2]) * 0.5;

	return FX_TRUE;

}

FX_BOOL FX_SolveJ567ZNYX(NSPBase* pnspbase, FX_DOUBLE rot_angle, FX_DOUBLE ref[7], FX_DOUBLE ret_j[7], FX_BOOL* is_degen, FX_DOUBLE dgr)
{
	
	FX_DOUBLE m1[3][3];
	FX_DOUBLE m2[3][3];
	FX_DOUBLE m[3][3];
	FX_DOUBLE j[3];
	FX_DOUBLE j1[3];
	FX_DOUBLE udgr = dgr;
	if (udgr < 0.05)
	{
		udgr = 0.05;
	}
	if (udgr > 10)
	{
		udgr = 10;
	}
	FX_MatRotAxis(pnspbase->rot_axis, rot_angle, pnspbase->j567Base, m1);
	FX_M33Trans(m1, m2);

	FX_MMM33(m2, pnspbase->wristges, m);

	if (FX_Matrix2ZYX(m,j) == FX_TRUE)
	{
		*is_degen = FX_FALSE;
		j1[0] = j[0];
		j1[1] = -j[1];
		j1[2] = j[2];
		ret_j[4] = j1[0];
		ret_j[5] = j1[1];
		ret_j[6] = j1[2];
		return FX_TRUE;
	}

	return FX_FALSE;

}


FX_BOOL  FX_InvKine_Pilot(FX_INT32L RobotSerial, FX_InvKineSolvePara * solve_para)
{
	
	
	FX_Robot* pRobot;
	FX_KineSPC_Pilot * SPC;
	NSPBase* NSP;
	FX_INT32L i;
	FX_INT32L j;
	FX_DOUBLE m_flan[4][4];
	FX_DOUBLE m_wrist[4][4];
	FX_DOUBLE pa[3];
	FX_DOUBLE pb[3];
	FX_DOUBLE va2b[3];
	FX_DOUBLE va2b_norm[3];
	FX_DOUBLE ablen;
	FX_DOUBLE JV4_A;
	FX_DOUBLE JV4_B;
	FX_DOUBLE  Core[3];
	FX_DOUBLE ang;
	FX_BOOL   j4DegTag;

	FX_BOOL   J246_DEG_TAG_B = FX_FALSE;

	FX_INT32L result_num = 0;
	FX_INT32L result_num_b = 0;

	Matrix3 WristGes;
	Matrix3 A_M123;
	Matrix3 A_M567;
	Matrix3 B_M123;
	Matrix3 B_M567;

	Matrix3 flange;
	//////////////////////////////////////
	
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	SPC = (FX_KineSPC_Pilot*)(pRobot->m_KineSPC);
	NSP = &(SPC->m_nsp);
	
	solve_para->m_Output_IsOutRange = FX_FALSE;
	solve_para->m_Output_IsJntExd = FX_FALSE;
	solve_para->m_Input_ZSP_Angle = 0;
	for ( i = 0; i < 7; i++)
	{
		solve_para->m_Output_IsDeg[i] = FX_FALSE;
		solve_para->m_Output_JntExdTags[i] = FX_FALSE;
	}

	//init result matrix
	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < 8; j++)
		{
			solve_para->m_OutPut_AllJoint[i][j] = 0;
		}
	}
	
	FX_MMM44(solve_para->m_Input_IK_TargetTCP, pRobot->m_KineBase.m_InvTool, m_flan);
	FX_MMM44(m_flan, pRobot->m_KineBase.m_InvFlange, m_wrist);

	for ( i = 0; i < 3; i++)
	{
		for ( j = 0; j < 3; j++)
		{
			WristGes[i][j] = m_wrist[i][j];
			flange[i][j] = m_flan[i][j];
			SPC->m_nsp.wristges[i][j] = m_flan[i][j];
		}
	}

	//// caculate point a (shoulder point) and point b (wrist point)

	pa[0] = pRobot->m_KineBase.m_JointPG[1][0][3];
	pa[1] = pRobot->m_KineBase.m_JointPG[1][1][3];
	pa[2] = pRobot->m_KineBase.m_JointPG[1][2][3];

	pb[0] = m_wrist[0][3];
	pb[1] = m_wrist[1][3];
	pb[2] = m_wrist[2][3];

	va2b[0] = pb[0] - pa[0];
	va2b[1] = pb[1] - pa[1];
	va2b[2] = pb[2] - pa[2];

	//// caculate j4 
	ablen = FX_Sqrt(va2b[0] * va2b[0] + va2b[1] * va2b[1] + va2b[2] * va2b[2]);

	if (ablen + 0.1 >= SPC->L1 + SPC->L2)
	{
		solve_para->m_Output_IsOutRange = FX_TRUE;
		return FX_FALSE;
	}

	va2b_norm[0] = va2b[0] / ablen;
	va2b_norm[1] = va2b[1] / ablen;
	va2b_norm[2] = va2b[2] / ablen;

	{
		FX_INT32 j4type1;
		FX_INT32 j4type2;
		FX_DOUBLE t;
		FX_DOUBLE Jv3[2];
		t = SPC->L1 * SPC->L1 + SPC->L2 * SPC->L2 - ablen * ablen;
		ang = FX_ACOS(t / (2.0 * SPC->L1 * SPC->L2)) * FXARM_R2D;
		Jv3[0] = SPC->Angt - ang;
		Jv3[1] = SPC->Angt + ang - 360;
		j4type1 = FX_GetJ4Type_Pilot(SPC, Jv3[0]);
		j4type2 = FX_GetJ4Type_Pilot(SPC, Jv3[1]);
		if (j4type1 == 0 || j4type2 == 0 || j4type1 == j4type2)
		{
			j4DegTag = FX_TRUE;
		}
		else
		{
			j4DegTag = FX_FALSE;

			//degen 246
			if (FX_Fabs(Jv3[0]) < 0.02 || FX_Fabs(Jv3[1]) < 0.02)
			{
				J246_DEG_TAG_B = FX_TRUE;
				solve_para->m_Output_IsDeg[3] = FX_TRUE;
				solve_para->m_Output_RetJoint[0] = solve_para->m_Input_IK_RefJoint[0];
				solve_para->m_Output_RetJoint[1] = solve_para->m_Input_IK_RefJoint[1];
				solve_para->m_Output_RetJoint[2] = solve_para->m_Input_IK_RefJoint[2];
				solve_para->m_Output_RetJoint[3] = solve_para->m_Input_IK_RefJoint[3];
				solve_para->m_Output_RetJoint[4] = solve_para->m_Input_IK_RefJoint[4];
				solve_para->m_Output_RetJoint[5] = solve_para->m_Input_IK_RefJoint[5];
				solve_para->m_Output_RetJoint[6] = solve_para->m_Input_IK_RefJoint[6];

				return FX_FALSE;
			}
			
			if (-1 == j4type1)
			{
				JV4_A = Jv3[0];
				JV4_B = Jv3[1];
			}
			else
			{
				JV4_A = Jv3[1];
				JV4_B = Jv3[0];
			}
		}

	}

	Vect3 ref_vx;
	{
		FX_DOUBLE sinv;
		FX_DOUBLE cosv;
		Matrix4  m_AxisRotTip[4];
		Matrix4  m_JointPG[4];
		for (i = 0; i < 3; i++)
		{
			FX_SIN_COS_DEG(solve_para->m_Input_IK_RefJoint[i], &sinv, &cosv);
			FX_XYZMRot(pRobot->m_KineBase.m_AxisRotBase[i], cosv, sinv,m_AxisRotTip[i]);
		}

		FX_M44Copy(m_AxisRotTip[0], m_JointPG[0]);
		FX_PGMult(m_JointPG[0], m_AxisRotTip[1], m_JointPG[1]);
		FX_PGMult(m_JointPG[1], m_AxisRotTip[2], m_JointPG[2]);
		ref_vx[0] = m_JointPG[2][0][0];
		ref_vx[1] = m_JointPG[2][1][0];
		ref_vx[2] = m_JointPG[2][2][0];

		if (solve_para->m_Input_IK_ZSPType == FX_PILOT_NSP_TYPES_NEAR_DIR)
		{
			ref_vx[0] = solve_para->m_Input_IK_ZSPPara[0];
			ref_vx[1] = solve_para->m_Input_IK_ZSPPara[1];
			ref_vx[2] = solve_para->m_Input_IK_ZSPPara[2];
			FX_VectNorm(ref_vx);
		}

		{
			Vect3 ref_vy;
			FX_VectCross(va2b_norm, ref_vx, ref_vy);
			FX_VectNorm(ref_vy);
			FX_VectCross(ref_vy,va2b_norm, ref_vx);
			FX_VectNorm(ref_vx);

			NSP->rot_m[0][0] = ref_vx[0];
			NSP->rot_m[1][0] = ref_vx[1];
			NSP->rot_m[2][0] = ref_vx[2];
			NSP->rot_m[0][1] = ref_vy[0];
			NSP->rot_m[1][1] = ref_vy[1];
			NSP->rot_m[2][1] = ref_vy[2];
			NSP->rot_m[0][2] = va2b_norm[0];
			NSP->rot_m[1][2] = va2b_norm[1];
			NSP->rot_m[2][2] = va2b_norm[2];
		}
	}

	if (j4DegTag == FX_TRUE)
	{
		Vect3 min_A_123;
		Vect3 min_A_567;
		FX_DOUBLE min_err123_A;
		FX_DOUBLE min_err567_A;


		FX_BOOL J123_DEG_TAG_A;
		FX_BOOL J567_DEG_TAG_A;

		{
			Vect3 vect123_z;
			Vect3 vect123_x;
			Vect3 vect123_y;
			Vect3 vect567_z;
			Vect3 vect567_x;
			Vect3 vect567_y;



			vect123_z[0] = va2b_norm[0];
			vect123_z[1] = va2b_norm[1];
			vect123_z[2] = va2b_norm[2];

			FX_VectNorm(vect123_z);
			FX_VectCross(vect123_z, ref_vx, vect123_y);
			FX_VectNorm(vect123_y);
			FX_VectCross(vect123_y, vect123_z, vect123_x);
			FX_VectNorm(vect123_x);

			vect567_z[0] = va2b_norm[0];
			vect567_z[1] = va2b_norm[1];
			vect567_z[2] = va2b_norm[2];

			FX_VectNorm(vect567_z);
			FX_VectCross(vect567_z, ref_vx, vect567_y);
			FX_VectNorm(vect567_y);
			FX_VectCross(vect567_y, vect567_z, vect567_x);
			FX_VectNorm(vect567_x);

			for (i = 0; i < 3; i++)
			{
				A_M123[i][0] = vect123_x[i];
				A_M123[i][1] = vect123_y[i];
				A_M123[i][2] = vect123_z[i];

				A_M567[i][0] = vect567_x[i];
				A_M567[i][1] = vect567_y[i];
				A_M567[i][2] = vect567_z[i];
			}

			{// solve 123
				Vect3 solve123;
				if (FX_Matrix2ZYZ(A_M123, solve123) == FX_TRUE)
				{
					J123_DEG_TAG_A = FX_FALSE;
					FX_DOUBLE j1_a = solve123[0];
					FX_DOUBLE j2_a = -solve123[1];
					FX_DOUBLE j3_a = solve123[2];

					FX_DOUBLE j1_b = j1_a + 180;
					FX_DOUBLE j2_b = -j2_a;
					FX_DOUBLE j3_b = j3_a + 180;

					FX_DOUBLE err1;
					FX_DOUBLE err2;

					err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j1_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j2_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j3_a);
					err2 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j1_b)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j2_b)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j3_b);

					min_A_123[0] = j1_a;
					min_A_123[1] = j2_a;
					min_A_123[2] = j3_a;
					min_err123_A = err1;
					if (min_err123_A > err2)
					{

						min_A_123[0] = j1_b;
						min_A_123[1] = j2_b;
						min_A_123[2] = j3_b;
						min_err123_A = err2;
					}
				}
				else
				{
					J123_DEG_TAG_A = FX_TRUE;
					FX_DOUBLE j1_a = solve_para->m_Input_IK_RefJoint[0];
					FX_DOUBLE j2_a = -solve123[1];
					//FX_DOUBLE j3_a = solve123[2] - j1_a;
					FX_DOUBLE j3_a = solve_para->m_Input_IK_RefJoint[2];
					FX_DOUBLE err1;

					err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j1_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j2_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j3_a);
					min_A_123[0] = j1_a;
					min_A_123[1] = j2_a;
					min_A_123[2] = j3_a;
					min_err123_A = err1;
				}
			}

			{// solve 456
				Vect3 solve567;
				Matrix3 A_M567_inv;
				Matrix3 A_M567_tran;
				for (i = 0; i < 3; i++)
				{
					for (j = 0; j < 3; j++)
					{
						A_M567_inv[i][j] = A_M567[j][i];
					}
				}
				FX_MMM33( A_M567_inv, flange, A_M567_tran);
				if (SPC->m_IsCross == FX_FALSE)
				{
					if (FX_Matrix2ZYZ(A_M567_tran, solve567) == FX_TRUE)
					{
						J567_DEG_TAG_A = FX_FALSE;
						FX_DOUBLE j5_a = solve567[0];
						FX_DOUBLE j6_a = -solve567[1];
						FX_DOUBLE j7_a = solve567[2];

						FX_DOUBLE j5_b = j5_a + 180;
						FX_DOUBLE j6_b = -j6_a;
						FX_DOUBLE j7_b = j7_a + 180;

						FX_DOUBLE err1;
						FX_DOUBLE err2;

						err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[4], &j5_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[5], &j6_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[6], &j7_a);
						err2 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[4], &j5_b)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[5], &j6_b)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[6], &j7_b);

						min_A_567[0] = j5_a;
						min_A_567[1] = j6_a;
						min_A_567[2] = j7_a;
						min_err567_A = err1;
						if (min_err567_A > err2)
						{
							min_A_567[0] = j5_b;
							min_A_567[1] = j6_b;
							min_A_567[2] = j7_b;
							min_err567_A = err2;
						}
					}
					else
					{
						J567_DEG_TAG_A = FX_TRUE;
						FX_DOUBLE j5_a = solve_para->m_Input_IK_RefJoint[4];
						FX_DOUBLE j6_a = -solve567[1];
						FX_DOUBLE j7_a = solve567[2] - j5_a;
						FX_DOUBLE err1;

						err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j5_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j6_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j7_a);

						min_A_567[0] = j5_a;
						min_A_567[1] = j6_a;
						min_A_567[2] = j7_a;
						min_err567_A = err1;
					}
				}
				else
				{
					FX_Matrix2ZYX(A_M567_tran, solve567);
					J567_DEG_TAG_A = FX_FALSE;
					FX_DOUBLE j5_a = solve567[0];
					FX_DOUBLE j6_a = -solve567[1];
					FX_DOUBLE j7_a = solve567[2];
					FX_DOUBLE err1;
					err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[4], &j5_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[5], &j6_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[6], &j7_a);
					min_err567_A = err1;
					min_A_567[0] = j5_a;
					min_A_567[1] = j6_a;
					min_A_567[2] = j7_a;

				}
			}
		}

		solve_para->m_Output_RetJoint[0] = min_A_123[0];
		solve_para->m_Output_RetJoint[1] = min_A_123[1];
		solve_para->m_Output_RetJoint[2] = min_A_123[2];
		solve_para->m_Output_RetJoint[3] = 0;
		solve_para->m_Output_RetJoint[4] = min_A_567[0];
		solve_para->m_Output_RetJoint[5] = min_A_567[1];
		solve_para->m_Output_RetJoint[6] = min_A_567[2];

		solve_para->m_Output_IsDeg[0] = FX_FALSE;
		solve_para->m_Output_IsDeg[1] = J123_DEG_TAG_A;
		solve_para->m_Output_IsDeg[2] = FX_FALSE;
		solve_para->m_Output_IsDeg[3] = FX_TRUE;
		solve_para->m_Output_IsDeg[4] = FX_FALSE;
		solve_para->m_Output_IsDeg[5] = J567_DEG_TAG_A;
		solve_para->m_Output_IsDeg[6] = FX_FALSE;


		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				SPC->m_nsp.j123Base[i][j] = A_M123[i][j];
				SPC->m_nsp.j567Base[i][j] = A_M567[i][j];
			}
			SPC->m_nsp.rot_axis[i] = va2b_norm[i];
		}

		SPC->m_nsp.j4type = 0;
		SPC->m_nsp.j4v = 0;



	}
	else
	{
		Vect3 min_A_123;
		Vect3 min_A_567;
		Vect3 min_B_123;
		Vect3 min_B_567;
		FX_DOUBLE min_err123_A;
		FX_DOUBLE min_err4_A;
		FX_DOUBLE min_err567_A;
		FX_DOUBLE min_err123_B;
		FX_DOUBLE min_err4_B;
		FX_DOUBLE min_err567_B;
		

		FX_BOOL J123_DEG_TAG_A = FX_FALSE;
		FX_BOOL J123_DEG_TAG_B = FX_FALSE;
		FX_BOOL J567_DEG_TAG_A = FX_FALSE;
		FX_BOOL J567_DEG_TAG_B = FX_FALSE;

		if(1)/// solve A Case
		{
			Vect3 vect123_z;
			Vect3 vect123_x;
			Vect3 vect123_y;
			Vect3 vect567_z;
			Vect3 vect567_x;
			Vect3 vect567_y;


			FX_DOUBLE la;
			FX_DOUBLE lb;
			FX_DOUBLE lc;
			FX_DOUBLE t1;
			FX_DOUBLE anga;
			FX_DOUBLE angb;
			FX_DOUBLE pt[2];
			///////////////////////////////////////
			la = SPC->L1;
			lb = SPC->L2;
			lc = ablen;
			t1 = lc * lc + la * la - lb * lb;
			anga = FX_ACOS(t1 / (2.0 * lc * la)) * FXARM_R2D;
			angb = 180 - anga - ang;

			anga += SPC->Ang1;
			angb += SPC->Ang2;
			{
				FX_DOUBLE p1[3];
				FX_DOUBLE p2[3];
				FX_DOUBLE v1x, v1y;
				FX_DOUBLE v2x, v2y;
				p1[0] = 0; p1[1] = 0; p1[2] = 0;
				p2[0] = ablen; p2[1] = 0; p2[2] = 0;
				FX_SIN_COS_DEG(anga, &v1y, &v1x);
				FX_SIN_COS_DEG(180 - angb, &v2y, &v2x);
				/////  pt is axis_cross_eblow
				if (FX_SolveTrange2D(ablen, v1x, v1y, v2x, v2y, pt) == FX_FALSE)
				{
					pt[0] = FX_COS_DEG(SPC->Ang1) * SPC->L1;
					pt[1] = FX_COS_DEG(SPC->Ang2) * SPC->L2;
				}
			}


			Core[0] = pa[0] + va2b_norm[0] * pt[0];
			Core[1] = pa[1] + va2b_norm[1] * pt[0];
			Core[2] = pa[2] + va2b_norm[2] * pt[0];

			vect123_z[0] = (Core[0] - ref_vx[0] * pt[1]) - pa[0];
			vect123_z[1] = (Core[1] - ref_vx[1] * pt[1]) - pa[1];
			vect123_z[2] = (Core[2] - ref_vx[2] * pt[1]) - pa[2];

			FX_VectNorm(vect123_z);
			FX_VectCross(vect123_z, ref_vx, vect123_y);
			FX_VectNorm(vect123_y);
			FX_VectCross(vect123_y, vect123_z, vect123_x);
			FX_VectNorm(vect123_x);

			FX_DOUBLE det123x = FX_Sqrt(vect123_x[0] * vect123_x[0] + vect123_x[1] * vect123_x[1] + vect123_x[2] * vect123_x[2]);
			FX_DOUBLE det123y = FX_Sqrt(vect123_y[0] * vect123_y[0] + vect123_y[1] * vect123_y[1] + vect123_y[2] * vect123_y[2]);

			if (det123x < 0.001 && det123y < 0.001)
			{
				vect123_y[0] = 0;
				vect123_y[1] = 1;
				vect123_y[2] = 0;

				vect123_x[0] = 1;
				vect123_x[1] = 0;
				vect123_x[2] = 0;
			}

			vect567_z[0] = pb[0] - (Core[0] - ref_vx[0] * pt[1]);
			vect567_z[1] = pb[1] - (Core[1] - ref_vx[1] * pt[1]);
			vect567_z[2] = pb[2] - (Core[2] - ref_vx[2] * pt[1]);

			FX_VectNorm(vect567_z);
			FX_VectCross(vect567_z, ref_vx, vect567_y);
			FX_VectNorm(vect567_y);
			FX_VectCross(vect567_y, vect567_z, vect567_x);
			FX_VectNorm(vect567_x);

			det123x = FX_Sqrt(vect567_x[0] * vect567_x[0] + vect567_x[1] * vect567_x[1] + vect567_x[2] * vect567_x[2]);
			det123y = FX_Sqrt(vect567_y[0] * vect567_y[0] + vect567_y[1] * vect567_y[1] + vect567_y[2] * vect567_y[2]);

			if (det123x < 0.001 && det123y < 0.001)
			{
				vect567_y[0] = 0;
				vect567_y[1] = 1;
				vect567_y[2] = 0;

				vect567_x[0] = 1;
				vect567_x[1] = 0;
				vect567_x[2] = 0;
			}

			for (i = 0; i < 3; i++)
			{
				A_M123[i][0] = vect123_x[i];
				A_M123[i][1] = vect123_y[i];
				A_M123[i][2] = vect123_z[i];

				A_M567[i][0] = vect567_x[i];
				A_M567[i][1] = vect567_y[i];
				A_M567[i][2] = vect567_z[i];
			}

			{// solve 123
				Vect3 solve123;
				if (FX_Matrix2ZYZ(A_M123, solve123) == FX_TRUE)
				{
					J123_DEG_TAG_A = FX_FALSE;
					FX_DOUBLE j1_a = solve123[0];
					FX_DOUBLE j2_a = -solve123[1];
					FX_DOUBLE j3_a = solve123[2];

					FX_DOUBLE j1_b = j1_a + 180;
					FX_DOUBLE j2_b = -j2_a;
					FX_DOUBLE j3_b = j3_a + 180;

					result_num = 2;

					FX_DOUBLE err1;
					FX_DOUBLE err2;

					err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j1_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j2_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j3_a);
					err2 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j1_b)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j2_b)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j3_b);

					solve_para->m_OutPut_AllJoint[0][0] = j1_a;
					solve_para->m_OutPut_AllJoint[0][1] = j2_a;
					solve_para->m_OutPut_AllJoint[0][2] = j3_a;

					solve_para->m_OutPut_AllJoint[1][0] = j1_b;
					solve_para->m_OutPut_AllJoint[1][1] = j2_b;
					solve_para->m_OutPut_AllJoint[1][2] = j3_b;

					min_A_123[0] = j1_a;
					min_A_123[1] = j2_a;
					min_A_123[2] = j3_a;
					min_err123_A = err1;
					if (min_err123_A > err2)
					{

						min_A_123[0] = j1_b;
						min_A_123[1] = j2_b;
						min_A_123[2] = j3_b;
						min_err123_A = err2;
					}
				}
				else
				{
					J123_DEG_TAG_A = FX_TRUE;
					FX_DOUBLE j1_a = solve_para->m_Input_IK_RefJoint[0];
					FX_DOUBLE j2_a = -solve123[1];
					//FX_DOUBLE j3_a = solve123[2] - j1_a;
					FX_DOUBLE j3_a = solve_para->m_Input_IK_RefJoint[2];

					FX_DOUBLE err1;

					err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j1_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j2_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j3_a);
					min_A_123[0] = j1_a;
					min_A_123[1] = j2_a;
					min_A_123[2] = j3_a;
					min_err123_A = err1;

					result_num = 1;

					solve_para->m_OutPut_AllJoint[0][0] = j1_a;
					solve_para->m_OutPut_AllJoint[0][1] = j2_a;
					solve_para->m_OutPut_AllJoint[0][2] = j3_a;
				}
			}

			{// solve 456
				Vect3 solve567;
				Matrix3 A_M567_inv;
				Matrix3 A_M567_tran;
				for (i = 0; i < 3; i++)
				{
					for (j = 0; j < 3; j++)
					{
						A_M567_inv[i][j] = A_M567[j][i];
					}
				}
				FX_MMM33(A_M567_inv, flange, A_M567_tran);
				if (SPC->m_IsCross == FX_FALSE)
				{
					if (FX_Matrix2ZYZ(A_M567_tran, solve567) == FX_TRUE)
					{
						J567_DEG_TAG_A = FX_FALSE;
						FX_DOUBLE j5_a = solve567[0];
						FX_DOUBLE j6_a = -solve567[1];
						FX_DOUBLE j7_a = solve567[2];

						FX_DOUBLE j5_b = j5_a + 180;
						FX_DOUBLE j6_b = -j6_a;
						FX_DOUBLE j7_b = j7_a + 180;

						FX_DOUBLE err1;
						FX_DOUBLE err2;

						err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[4], &j5_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[5], &j6_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[6], &j7_a);
						err2 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[4], &j5_b)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[5], &j6_b)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[6], &j7_b);

						result_num *= 2;

						if (result_num == 2)
						{
							solve_para->m_OutPut_AllJoint[1][0] = solve_para->m_OutPut_AllJoint[0][0];
							solve_para->m_OutPut_AllJoint[1][1] = solve_para->m_OutPut_AllJoint[0][1];
							solve_para->m_OutPut_AllJoint[1][2] = solve_para->m_OutPut_AllJoint[0][2];

							solve_para->m_OutPut_AllJoint[0][4] = j5_a;
							solve_para->m_OutPut_AllJoint[0][5] = j6_a;
							solve_para->m_OutPut_AllJoint[0][6] = j7_a;

							solve_para->m_OutPut_AllJoint[1][4] = j5_b;
							solve_para->m_OutPut_AllJoint[1][5] = j6_b;
							solve_para->m_OutPut_AllJoint[1][6] = j7_b;
						}
						else
						{
							for (i = 0; i < 2; i++)
							{
								solve_para->m_OutPut_AllJoint[i + 2][0] = solve_para->m_OutPut_AllJoint[i][0];
								solve_para->m_OutPut_AllJoint[i + 2][1] = solve_para->m_OutPut_AllJoint[i][1];
								solve_para->m_OutPut_AllJoint[i + 2][2] = solve_para->m_OutPut_AllJoint[i][2];

								solve_para->m_OutPut_AllJoint[2 * i][4] = j5_a;
								solve_para->m_OutPut_AllJoint[2 * i][5] = j6_a;
								solve_para->m_OutPut_AllJoint[2 * i][6] = j7_a;

								solve_para->m_OutPut_AllJoint[2 * i + 1][4] = j5_b;
								solve_para->m_OutPut_AllJoint[2 * i + 1][5] = j6_b;
								solve_para->m_OutPut_AllJoint[2 * i + 1][6] = j7_b;
							}
						}

						min_A_567[0] = j5_a;
						min_A_567[1] = j6_a;
						min_A_567[2] = j7_a;
						min_err567_A = err1;
						if (min_err567_A > err2)
						{
							min_A_567[0] = j5_b;
							min_A_567[1] = j6_b;
							min_A_567[2] = j7_b;
							min_err567_A = err2;
						}
					}
					else
					{
						J567_DEG_TAG_A = FX_TRUE;
						FX_DOUBLE j5_a = solve_para->m_Input_IK_RefJoint[4];
						FX_DOUBLE j6_a = -solve567[1];
						FX_DOUBLE j7_a = solve567[2] - j5_a;
						FX_DOUBLE err1;

						err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j5_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j6_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j7_a);

						min_A_567[0] = j5_a;
						min_A_567[1] = j6_a;
						min_A_567[2] = j7_a;
						min_err567_A = err1;

						for (i = 0; i < result_num; i++)
						{
							solve_para->m_OutPut_AllJoint[i][4] = j5_a;
							solve_para->m_OutPut_AllJoint[i][5] = j6_a;
							solve_para->m_OutPut_AllJoint[i][6] = j7_a;
						}

					}
				}
				else
				{
					FX_Matrix2ZYX(A_M567_tran, solve567);
					J567_DEG_TAG_A = FX_FALSE;
					FX_DOUBLE j5_a = solve567[0];
					FX_DOUBLE j6_a = -solve567[1];
					FX_DOUBLE j7_a = solve567[2];
					FX_DOUBLE err1;
					err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[4], &j5_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[5], &j6_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[6], &j7_a);
					min_err567_A = err1;
					min_A_567[0] = j5_a;
					min_A_567[1] = j6_a;
					min_A_567[2] = j7_a;

					for (i = 0; i < result_num; i++)
					{
						solve_para->m_OutPut_AllJoint[i][4] = j5_a;
						solve_para->m_OutPut_AllJoint[i][5] = j6_a;
						solve_para->m_OutPut_AllJoint[i][6] = j7_a;
					}

				}
			}
			for (i = 0; i < result_num; i++)
			{
				solve_para->m_OutPut_AllJoint[i][3] = JV4_A;
			}

		}
		if(1)/// solve B Case
		{
			Vect3 vect123_z;
			Vect3 vect123_x;
			Vect3 vect123_y;
			Vect3 vect567_z;
			Vect3 vect567_x;
			Vect3 vect567_y;

			FX_DOUBLE la;
			FX_DOUBLE lb;
			FX_DOUBLE lc;
			FX_DOUBLE t1;
			FX_DOUBLE anga;
			FX_DOUBLE angb;
			FX_DOUBLE pt[2];
			///////////////////////////////////////
			la = SPC->L1;
			lb = SPC->L2;
			lc = ablen;
			t1 = lc * lc + la * la - lb * lb;
			anga = FX_ACOS(t1 / (2.0 * lc * la)) * FXARM_R2D;
			angb = 180 - anga - ang;

			anga -= SPC->Ang1;
			angb -= SPC->Ang2;

			anga = FX_Fabs(anga);
			angb = FX_Fabs(angb);
			{
				FX_DOUBLE p1[3];
				FX_DOUBLE p2[3];
				FX_DOUBLE v1x, v1y;
				FX_DOUBLE v2x, v2y;
				p1[0] = 0; p1[1] = 0; p1[2] = 0;
				p2[0] = ablen; p2[1] = 0; p2[2] = 0;
				FX_SIN_COS_DEG(anga, &v1y, &v1x);
				FX_SIN_COS_DEG(180 - angb, &v2y, &v2x);
				/////  pt is axis_cross_eblow
				if (FX_SolveTrange2D(ablen, v1x, v1y, v2x, v2y, pt) == FX_FALSE)
				{
					pt[0] = FX_COS_DEG(SPC->Ang1) * SPC->L1;
					pt[1] = FX_COS_DEG(SPC->Ang2) * SPC->L2;
				}
			}
			
			Core[0] = pa[0] + va2b_norm[0] * pt[0];
			Core[1] = pa[1] + va2b_norm[1] * pt[0];
			Core[2] = pa[2] + va2b_norm[2] * pt[0];

			vect123_z[0] = (Core[0] + ref_vx[0] * pt[1]) - pa[0];
			vect123_z[1] = (Core[1] + ref_vx[1] * pt[1]) - pa[1];
			vect123_z[2] = (Core[2] + ref_vx[2] * pt[1]) - pa[2];

			FX_VectNorm(vect123_z);
			FX_VectCross(vect123_z, ref_vx, vect123_y);
			FX_VectNorm(vect123_y);
			FX_VectCross(vect123_y, vect123_z, vect123_x);
			FX_VectNorm(vect123_x);

			vect567_z[0] = pb[0] - (Core[0] + ref_vx[0] * pt[1]);
			vect567_z[1] = pb[1] - (Core[1] + ref_vx[1] * pt[1]);
			vect567_z[2] = pb[2] - (Core[2] + ref_vx[2] * pt[1]);

			FX_VectNorm(vect567_z);
			FX_VectCross(vect567_z, ref_vx, vect567_y);
			FX_VectNorm(vect567_y);
			FX_VectCross(vect567_y, vect567_z, vect567_x);
			FX_VectNorm(vect567_x);

			for (i = 0; i < 3; i++)
			{
				B_M123[i][0] = vect123_x[i];
				B_M123[i][1] = vect123_y[i];
				B_M123[i][2] = vect123_z[i];

				B_M567[i][0] = vect567_x[i];
				B_M567[i][1] = vect567_y[i];
				B_M567[i][2] = vect567_z[i];
			}

			{// solve 123
				Vect3 solve123;
				if (FX_Matrix2ZYZ(B_M123, solve123) == FX_TRUE)
				{
					J123_DEG_TAG_B = FX_FALSE;
					FX_DOUBLE j1_a = solve123[0];
					FX_DOUBLE j2_a = -solve123[1];
					FX_DOUBLE j3_a = solve123[2];

					FX_DOUBLE j1_b = j1_a + 180;
					FX_DOUBLE j2_b = -j2_a;
					FX_DOUBLE j3_b = j3_a + 180;

					FX_DOUBLE err1;
					FX_DOUBLE err2;

					result_num_b = 2;

					err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j1_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j2_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j3_a);
					err2 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j1_b)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j2_b)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j3_b);

					solve_para->m_OutPut_AllJoint[result_num][0] = j1_a;
					solve_para->m_OutPut_AllJoint[result_num][1] = j2_a;
					solve_para->m_OutPut_AllJoint[result_num][2] = j3_a;

					solve_para->m_OutPut_AllJoint[result_num + 1][0] = j1_b;
					solve_para->m_OutPut_AllJoint[result_num + 1][1] = j2_b;
					solve_para->m_OutPut_AllJoint[result_num + 1][2] = j3_b;

					min_B_123[0] = j1_a;
					min_B_123[1] = j2_a;
					min_B_123[2] = j3_a;
					min_err123_B = err1;
					if (min_err123_B > err2)
					{
						min_B_123[0] = j1_b;
						min_B_123[1] = j2_b;
						min_B_123[2] = j3_b;
						min_err123_B = err2;
					}
				}
				else
				{
					J123_DEG_TAG_B = FX_TRUE;
					FX_DOUBLE j1_a = solve_para->m_Input_IK_RefJoint[0];
					FX_DOUBLE j2_a = -solve123[1];
					FX_DOUBLE j3_a = solve123[2] - j1_a;
					FX_DOUBLE err1;

					err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j1_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j2_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j3_a);
					min_B_123[0] = j1_a;
					min_B_123[1] = j2_a;
					min_B_123[2] = j3_a;
					min_err123_B = err1;

					result_num_b = 1;

					solve_para->m_OutPut_AllJoint[result_num][0] = j1_a;
					solve_para->m_OutPut_AllJoint[result_num][1] = j2_a;
					solve_para->m_OutPut_AllJoint[result_num][2] = j3_a;

				}
			}

			{// solve 456
				Vect3 solve567;
				Matrix3 B_M567_inv;
				Matrix3 B_M567_tran;
				for (i = 0; i < 3; i++)
				{
					for (j = 0; j < 3; j++)
					{
						B_M567_inv[i][j] = B_M567[j][i];
					}
				}
				FX_MMM33( B_M567_inv, flange,B_M567_tran);
				if (SPC->m_IsCross == FX_FALSE)
				{
					if (FX_Matrix2ZYZ(B_M567_tran, solve567) == FX_TRUE)
					{
						J567_DEG_TAG_B = FX_FALSE;
						FX_DOUBLE j5_a = solve567[0];
						FX_DOUBLE j6_a = -solve567[1];
						FX_DOUBLE j7_a = solve567[2];

						FX_DOUBLE j5_b = j5_a + 180;
						FX_DOUBLE j6_b = -j6_a;
						FX_DOUBLE j7_b = j7_a + 180;

						FX_DOUBLE err1;
						FX_DOUBLE err2;

						result_num_b *= 2;

						err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[4], &j5_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[5], &j6_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[6], &j7_a);
						err2 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[4], &j5_b)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[5], &j6_b)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[6], &j7_b);

						if (result_num_b == 2)
						{
							solve_para->m_OutPut_AllJoint[result_num + 1][0] = solve_para->m_OutPut_AllJoint[result_num][0];
							solve_para->m_OutPut_AllJoint[result_num + 1][1] = solve_para->m_OutPut_AllJoint[result_num][1];
							solve_para->m_OutPut_AllJoint[result_num + 1][2] = solve_para->m_OutPut_AllJoint[result_num][2];

							solve_para->m_OutPut_AllJoint[result_num][4] = j5_a;
							solve_para->m_OutPut_AllJoint[result_num][5] = j6_a;
							solve_para->m_OutPut_AllJoint[result_num][6] = j7_a;

							solve_para->m_OutPut_AllJoint[result_num + 1][4] = j5_b;
							solve_para->m_OutPut_AllJoint[result_num + 1][5] = j6_b;
							solve_para->m_OutPut_AllJoint[result_num + 1][6] = j7_b;
						}
						else
						{
							for (i = 0; i < 2; i++)
							{
								solve_para->m_OutPut_AllJoint[i + result_num + 2][0] = solve_para->m_OutPut_AllJoint[i + result_num][0];
								solve_para->m_OutPut_AllJoint[i + result_num + 2][1] = solve_para->m_OutPut_AllJoint[i + result_num][1];
								solve_para->m_OutPut_AllJoint[i + result_num + 2][2] = solve_para->m_OutPut_AllJoint[i + result_num][2];

								solve_para->m_OutPut_AllJoint[2 * i + result_num][4] = j5_a;
								solve_para->m_OutPut_AllJoint[2 * i + result_num][5] = j6_a;
								solve_para->m_OutPut_AllJoint[2 * i + result_num][6] = j7_a;

								solve_para->m_OutPut_AllJoint[2 * i + result_num + 1][4] = j5_b;
								solve_para->m_OutPut_AllJoint[2 * i + result_num + 1][5] = j6_b;
								solve_para->m_OutPut_AllJoint[2 * i + result_num + 1][6] = j7_b;
							}
						}

						min_B_567[0] = j5_a;
						min_B_567[1] = j6_a;
						min_B_567[2] = j7_a;
						min_err567_B = err1;
						if (min_err567_B > err2)
						{
							min_B_567[0] = j5_b;
							min_B_567[1] = j6_b;
							min_B_567[2] = j7_b;
							min_err567_B = err2;
						}
					}
					else
					{
						J567_DEG_TAG_B = FX_TRUE;
						FX_DOUBLE j5_a = solve_para->m_Input_IK_RefJoint[4];
						FX_DOUBLE j6_a = -solve567[1];
						FX_DOUBLE j7_a = solve567[2] - j5_a;
						FX_DOUBLE err1;

						err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[0], &j5_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[1], &j6_a)
							+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[2], &j7_a);
						min_B_567[0] = j5_a;
						min_B_567[1] = j6_a;
						min_B_567[2] = j7_a;
						min_err567_B = err1;

						for (i = 0; i < result_num_b; i++)
						{
							solve_para->m_OutPut_AllJoint[i + result_num][4] = j5_a;
							solve_para->m_OutPut_AllJoint[i + result_num][5] = j6_a;
							solve_para->m_OutPut_AllJoint[i + result_num][6] = j7_a;
						}
					}
				}
				else
				{
					J567_DEG_TAG_B = FX_FALSE;
					FX_Matrix2ZYX(B_M567_tran, solve567);
					FX_DOUBLE j5_a = solve567[0];
					FX_DOUBLE j6_a = -solve567[1];
					FX_DOUBLE j7_a = solve567[2];
					FX_DOUBLE err1;
					err1 = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[4], &j5_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[5], &j6_a)
						+ FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[6], &j7_a);

					min_B_567[0] = j5_a;
					min_B_567[1] = j6_a;
					min_B_567[2] = j7_a;
					min_err567_B = err1;

					for (i = 0; i < result_num_b; i++)
					{
						solve_para->m_OutPut_AllJoint[i+result_num][4] = j5_a;
						solve_para->m_OutPut_AllJoint[i+result_num][5] = j6_a;
						solve_para->m_OutPut_AllJoint[i+result_num][6] = j7_a;
					}

				}
			}
			for (i = 0; i < result_num_b; i++)
			{
				solve_para->m_OutPut_AllJoint[i+result_num][3] = JV4_B;
			}
		}

		solve_para->m_OutPut_Result_Num = result_num + result_num_b;
		
		min_err4_A = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[3], &JV4_A);
		min_err4_B = FX_MinDif_Circle(solve_para->m_Input_IK_RefJoint[3], &JV4_B);


		if (min_err123_A + min_err4_A + min_err567_A < min_err123_B + min_err4_B + min_err567_B)
		{
			solve_para->m_Output_RetJoint[0] = min_A_123[0];
			solve_para->m_Output_RetJoint[1] = min_A_123[1];
			solve_para->m_Output_RetJoint[2] = min_A_123[2];
			solve_para->m_Output_RetJoint[3] = JV4_A;
			solve_para->m_Output_RetJoint[4] = min_A_567[0];
			solve_para->m_Output_RetJoint[5] = min_A_567[1];
			solve_para->m_Output_RetJoint[6] = min_A_567[2];

			solve_para->m_Output_IsDeg[0] = FX_FALSE;
			solve_para->m_Output_IsDeg[1] = J123_DEG_TAG_A;
			solve_para->m_Output_IsDeg[2] = FX_FALSE;
			solve_para->m_Output_IsDeg[3] = J246_DEG_TAG_B;
			solve_para->m_Output_IsDeg[4] = FX_FALSE;
			solve_para->m_Output_IsDeg[5] = J567_DEG_TAG_A;
			solve_para->m_Output_IsDeg[6] = FX_FALSE;
			
			if (J246_DEG_TAG_B)
			{
				solve_para->m_Output_RetJoint[0] = solve_para->m_Input_IK_RefJoint[0];
				solve_para->m_Output_RetJoint[1] = solve_para->m_Input_IK_RefJoint[1];
				solve_para->m_Output_RetJoint[2] = solve_para->m_Input_IK_RefJoint[2];
				solve_para->m_Output_RetJoint[3] = solve_para->m_Input_IK_RefJoint[3];
				solve_para->m_Output_RetJoint[4] = solve_para->m_Input_IK_RefJoint[4];
				solve_para->m_Output_RetJoint[5] = solve_para->m_Input_IK_RefJoint[5];
				solve_para->m_Output_RetJoint[6] = solve_para->m_Input_IK_RefJoint[6];
			}

			for ( i = 0; i < 3; i++)
			{
				for (j = 0; j < 3; j++)
				{
					SPC->m_nsp.j123Base[i][j] = A_M123[i][j];
					SPC->m_nsp.j567Base[i][j] = A_M567[i][j];
				}
				SPC->m_nsp.rot_axis[i] = va2b_norm[i];
			}

			SPC->m_nsp.j4type = -1;
			SPC->m_nsp.j4v = JV4_A;
		}
		else
		{
			solve_para->m_Output_RetJoint[0] = min_B_123[0];
			solve_para->m_Output_RetJoint[1] = min_B_123[1];
			solve_para->m_Output_RetJoint[2] = min_B_123[2];
			solve_para->m_Output_RetJoint[3] = JV4_B;
			solve_para->m_Output_RetJoint[4] = min_B_567[0];
			solve_para->m_Output_RetJoint[5] = min_B_567[1];
			solve_para->m_Output_RetJoint[6] = min_B_567[2];

			solve_para->m_Output_IsDeg[0] = FX_FALSE;
			solve_para->m_Output_IsDeg[1] = J123_DEG_TAG_B;
			solve_para->m_Output_IsDeg[2] = FX_FALSE;
			solve_para->m_Output_IsDeg[3] = J246_DEG_TAG_B;
			solve_para->m_Output_IsDeg[4] = FX_FALSE;
			solve_para->m_Output_IsDeg[5] = J567_DEG_TAG_B;
			solve_para->m_Output_IsDeg[6] = FX_FALSE;

			if (J246_DEG_TAG_B)
			{
				solve_para->m_Output_RetJoint[0] = solve_para->m_Input_IK_RefJoint[0];
				solve_para->m_Output_RetJoint[1] = solve_para->m_Input_IK_RefJoint[1];
				solve_para->m_Output_RetJoint[2] = solve_para->m_Input_IK_RefJoint[2];
				solve_para->m_Output_RetJoint[3] = solve_para->m_Input_IK_RefJoint[3];
				solve_para->m_Output_RetJoint[4] = solve_para->m_Input_IK_RefJoint[4];
				solve_para->m_Output_RetJoint[5] = solve_para->m_Input_IK_RefJoint[5];
				solve_para->m_Output_RetJoint[6] = solve_para->m_Input_IK_RefJoint[6];
			}

			for (i = 0; i < 3; i++)
			{
				for (j = 0; j < 3; j++)
				{
					SPC->m_nsp.j123Base[i][j] = B_M123[i][j];
					SPC->m_nsp.j567Base[i][j] = B_M567[i][j];
				}

				SPC->m_nsp.rot_axis[i] = va2b_norm[i];
			}
			SPC->m_nsp.j4type = 1;

			SPC->m_nsp.j4v = JV4_B;

		}
	}

	return FX_TRUE;
}


FX_BOOL  FX_InvKine_Pilot_SRS(FX_INT32L RobotSerial, FX_InvKineSolvePara* solve_para)
{
	FX_Robot* pRobot;
	FX_KineSPC_Pilot* SPC;
	NSPBase* NSP;
	FX_INT32L i;
	//////////////////
	if (FX_InvKine_Pilot(RobotSerial, solve_para) == FX_FALSE)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	SPC = (FX_KineSPC_Pilot*)(pRobot->m_KineSPC);
	NSP = &(SPC->m_nsp);

	solve_para->m_Output_IsJntExd = FX_FALSE;
	solve_para->m_Output_JntExdABS = 0;
	FX_DOUBLE exdabs = 0;
	for ( i = 0; i < 7; i++)
	{
		solve_para->m_Output_RunLmtP[i] = pRobot->m_Lmt.m_JLmtPos_P[i];
		solve_para->m_Output_RunLmtN[i] = pRobot->m_Lmt.m_JLmtPos_N[i];
		if (solve_para->m_Output_RetJoint[i] < solve_para->m_Output_RunLmtN[i] )
		{
			solve_para->m_Output_IsJntExd = FX_TRUE;
			solve_para->m_Output_JntExdTags[i] = FX_TRUE;
			exdabs = FX_Fabs(solve_para->m_Output_RetJoint[i] - solve_para->m_Output_RunLmtN[i]);
			if (solve_para->m_Output_JntExdABS < exdabs)
			{
				solve_para->m_Output_JntExdABS = exdabs;
			}
		}
		else if ( solve_para->m_Output_RetJoint[i] > solve_para->m_Output_RunLmtP[i])
		{
			solve_para->m_Output_IsJntExd = FX_TRUE;
			solve_para->m_Output_JntExdTags[i] = FX_TRUE;
			exdabs = FX_Fabs(solve_para->m_Output_RetJoint[i] - solve_para->m_Output_RunLmtP[i]);
			if (solve_para->m_Output_JntExdABS < exdabs)
			{
				solve_para->m_Output_JntExdABS = exdabs;
			}

		}
		else
		{
			solve_para->m_Output_JntExdTags[i] = FX_FALSE;
		}
	}

	printf("EG:ret_J=[%lf %lf %lf %lf %lf %lf %lf]\n", solve_para->m_Output_RetJoint[0], solve_para->m_Output_RetJoint[1], solve_para->m_Output_RetJoint[2], solve_para->m_Output_RetJoint[3]
		, solve_para->m_Output_RetJoint[4], solve_para->m_Output_RetJoint[5], solve_para->m_Output_RetJoint[6]);
	return FX_TRUE;
}

FX_BOOL  FX_InvKine_Pilot_CCS(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para)
{
	FX_Robot* pRobot;
	FX_KineSPC_Pilot* SPC;
	NSPBase* NSP;
	FX_INT32L i;
	//////////////////
	if (FX_InvKine_Pilot(RobotSerial, solve_para) == FX_FALSE)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	SPC = (FX_KineSPC_Pilot*)(pRobot->m_KineSPC);
	NSP = &(SPC->m_nsp);

	solve_para->m_Output_IsJntExd = FX_FALSE;
	solve_para->m_Output_JntExdABS = 0;
	FX_DOUBLE exdabs = 0;
	for (i = 0; i < 7; i++)
	{
		solve_para->m_Output_RunLmtP[i] = pRobot->m_Lmt.m_JLmtPos_P[i];
		solve_para->m_Output_RunLmtN[i] = pRobot->m_Lmt.m_JLmtPos_N[i];
		if (solve_para->m_Output_RetJoint[i] < solve_para->m_Output_RunLmtN[i] )
		{
			solve_para->m_Output_IsJntExd = FX_TRUE;
			solve_para->m_Output_JntExdTags[i] = FX_TRUE;

			exdabs = FX_Fabs(solve_para->m_Output_RetJoint[i] - solve_para->m_Output_RunLmtN[i]);
			if (solve_para->m_Output_JntExdABS < exdabs)
			{
				solve_para->m_Output_JntExdABS = exdabs;
			}
		}
		else if (solve_para->m_Output_RetJoint[i] > solve_para->m_Output_RunLmtP[i])
		{
			solve_para->m_Output_IsJntExd = FX_TRUE;
			solve_para->m_Output_JntExdTags[i] = FX_TRUE;

			exdabs = FX_Fabs(solve_para->m_Output_RetJoint[i] - solve_para->m_Output_RunLmtP[i]);
			if (solve_para->m_Output_JntExdABS < exdabs)
			{
				solve_para->m_Output_JntExdABS = exdabs;
			}
		}
		else
		{
			solve_para->m_Output_JntExdTags[i] = FX_FALSE;
		}
	}
	printf("EG:ret_J=[%lf %lf %lf %lf %lf %lf %lf]\n", solve_para->m_Output_RetJoint[0], solve_para->m_Output_RetJoint[1], solve_para->m_Output_RetJoint[2], solve_para->m_Output_RetJoint[3]
		, solve_para->m_Output_RetJoint[4], solve_para->m_Output_RetJoint[5], solve_para->m_Output_RetJoint[6]);
	
	if (solve_para->m_Output_RetJoint[5] > 1)
	{
		FX_DOUBLE N;
		FX_DOUBLE P;
		FX_DOUBLE J6;
		
		J6 = solve_para->m_Output_RetJoint[5];
		if (J6 > solve_para->m_Output_RunLmtP[5])
		{
			J6 = solve_para->m_Output_RunLmtP[5];
		}
		P = SPC->lmtj67_pp[0] * J6 * J6 + SPC->lmtj67_pp[1] *  J6 + SPC->lmtj67_pp[2];
		if (solve_para->m_Output_RunLmtP[6] > P)
		{
			solve_para->m_Output_RunLmtP[6] = P;
		}
		printf("j7=%lf line:%lf\n",solve_para->m_Output_RunLmtP[6],solve_para->m_Output_RetJoint[6]);
		if (solve_para->m_Output_RetJoint[6] > solve_para->m_Output_RunLmtP[6])
		{
			solve_para->m_Output_IsJntExd = FX_TRUE;
			solve_para->m_Output_JntExdTags[6] = FX_TRUE;

			exdabs = FX_Fabs(solve_para->m_Output_RetJoint[6]- solve_para->m_Output_RunLmtP[6]);
			if (solve_para->m_Output_JntExdABS < exdabs)
			{
				solve_para->m_Output_JntExdABS = exdabs;
			}
		}

		N = SPC->lmtj67_pn[0] * J6 * J6 + SPC->lmtj67_pn[1] * J6 + SPC->lmtj67_pn[2];
		if (solve_para->m_Output_RunLmtN[6] < N)
		{
			solve_para->m_Output_RunLmtN[6] = N;
		}
		printf("j7=%lf line:%lf\n",solve_para->m_Output_RunLmtP[6],solve_para->m_Output_RetJoint[6]);
		if (solve_para->m_Output_RetJoint[6] < solve_para->m_Output_RunLmtN[6])
		{
			solve_para->m_Output_IsJntExd = FX_TRUE;
			solve_para->m_Output_JntExdTags[6] = FX_TRUE;
			exdabs = FX_Fabs(solve_para->m_Output_RetJoint[6] - solve_para->m_Output_RunLmtN[6]);
			if (solve_para->m_Output_JntExdABS < exdabs)
			{
				solve_para->m_Output_JntExdABS = exdabs;
			}
		}


		return FX_TRUE;
	}


	if (solve_para->m_Output_RetJoint[5] < -1)
	{
		FX_DOUBLE N;
		FX_DOUBLE P;
		FX_DOUBLE J6;
		J6 = solve_para->m_Output_RetJoint[5];
		if (J6 > solve_para->m_Output_RunLmtP[5])
		{
			J6 = solve_para->m_Output_RunLmtP[5];
		}
		P = SPC->lmtj67_np[0] * J6 * J6 + SPC->lmtj67_np[1] * J6 + SPC->lmtj67_np[2];
		if (solve_para->m_Output_RunLmtP[6] > P)
		{
			solve_para->m_Output_RunLmtP[6] = P;
		}
		printf("j7=%lf line:%lf\n",solve_para->m_Output_RunLmtP[6],solve_para->m_Output_RetJoint[6]);
		if (solve_para->m_Output_RetJoint[6] > solve_para->m_Output_RunLmtP[6])
		{
			solve_para->m_Output_IsJntExd = FX_TRUE;
			solve_para->m_Output_JntExdTags[6] = FX_TRUE;
			exdabs = FX_Fabs(solve_para->m_Output_RetJoint[6] - solve_para->m_Output_RunLmtP[6]);
			if (solve_para->m_Output_JntExdABS < exdabs)
			{
				solve_para->m_Output_JntExdABS = exdabs;
			}
		}

		N = SPC->lmtj67_nn[0] * J6 * J6 + SPC->lmtj67_nn[1] * J6 + SPC->lmtj67_nn[2];
		if (solve_para->m_Output_RunLmtN[6] < N)
		{
			solve_para->m_Output_RunLmtN[6] = N;
		}
		printf("j7=%lf line:%lf\n",solve_para->m_Output_RunLmtP[6],solve_para->m_Output_RetJoint[6]);
		if (solve_para->m_Output_RetJoint[6] < solve_para->m_Output_RunLmtN[6])
		{
			solve_para->m_Output_IsJntExd = FX_TRUE;
			solve_para->m_Output_JntExdTags[6] = FX_TRUE;
			exdabs = FX_Fabs(solve_para->m_Output_RetJoint[6] - solve_para->m_Output_RunLmtN[6]);
			if (solve_para->m_Output_JntExdABS < exdabs)
			{
				solve_para->m_Output_JntExdABS = exdabs;
			}

		}
		return FX_TRUE;
	}


	return FX_TRUE;
}

FX_BOOL  FX_Robot_Kine_IK(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para)
{
	FX_LOG_INFO("[FxRobot - FX_Robot_Kine_IK]\n");
	FX_Robot* pRobot;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	if (pRobot->m_RobotType == FX_ROBOT_TYPE_DL)
	{
		return FX_InvKine_DL(RobotSerial, solve_para);
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_SRS)
	{
		return FX_InvKine_Pilot_SRS(RobotSerial, solve_para);
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_CCS)
	{
		return FX_InvKine_Pilot_CCS(RobotSerial, solve_para);
	}
	else
	{
		return FX_FALSE;
	}
	return FX_FALSE;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

FX_BOOL  FX_InvKineNSP_Pilot(FX_INT32L RobotSerial, FX_InvKineSolvePara* solve_para)
{
	FX_Robot* pRobot;
	FX_KineSPC_Pilot* SPC;
	NSPBase* NSP;
	//////////////////////////////////////////////
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	SPC = (FX_KineSPC_Pilot*)(pRobot->m_KineSPC);
	NSP = &(SPC->m_nsp);

	solve_para->m_Output_IsDeg[1] = FX_FALSE;
	solve_para->m_Output_IsDeg[5] = FX_FALSE;
	

	solve_para->m_Output_RetJoint[3] = NSP->j4v;
	if (FX_SolveJ123ZNYZ(NSP, solve_para->m_Input_ZSP_Angle, solve_para->m_Input_IK_RefJoint, solve_para->m_Output_RetJoint, &(solve_para->m_Output_IsDeg[1]), solve_para->m_DGR1) == FX_FALSE)
	{
		return FX_FALSE;
	}

	solve_para->m_Output_JntExdABS = 0;
	FX_DOUBLE exdabs = 0;
	if (NSP->m_IsCorss == FX_TRUE)
	{
		FX_INT32L i;
		if (FX_SolveJ567ZNYX(NSP, solve_para->m_Input_ZSP_Angle, solve_para->m_Input_IK_RefJoint, solve_para->m_Output_RetJoint, &(solve_para->m_Output_IsDeg[5]), solve_para->m_DGR2) == FX_FALSE)
		{
			return FX_FALSE;
		}

		solve_para->m_Output_IsJntExd = FX_FALSE;
		for (i = 0; i < 7; i++)
		{
			solve_para->m_Output_RunLmtP[i] = pRobot->m_Lmt.m_JLmtPos_P[i];
			solve_para->m_Output_RunLmtN[i] = pRobot->m_Lmt.m_JLmtPos_N[i];
			if (solve_para->m_Output_RetJoint[i] < solve_para->m_Output_RunLmtN[i])
			{
				solve_para->m_Output_IsJntExd = FX_TRUE;
				solve_para->m_Output_JntExdTags[i] = FX_TRUE;
				exdabs =  FX_Fabs(solve_para->m_Output_RetJoint[i] - solve_para->m_Output_RunLmtN[i]);
				if (solve_para->m_Output_JntExdABS < exdabs)
				{
					solve_para->m_Output_JntExdABS = exdabs;
				}
			}
			else if ( solve_para->m_Output_RetJoint[i] > solve_para->m_Output_RunLmtP[i])
			{
				solve_para->m_Output_IsJntExd = FX_TRUE;
				solve_para->m_Output_JntExdTags[i] = FX_TRUE;
				exdabs = FX_Fabs(solve_para->m_Output_RetJoint[i] - solve_para->m_Output_RunLmtP[i]);
				if (solve_para->m_Output_JntExdABS < exdabs)
				{
					solve_para->m_Output_JntExdABS = exdabs;
				}
			}
			else
			{
				solve_para->m_Output_JntExdTags[i] = FX_FALSE;
			}
		}
		printf("EG:ret_J=[%lf %lf %lf %lf %lf %lf %lf]\n", solve_para->m_Output_RetJoint[0], solve_para->m_Output_RetJoint[1], solve_para->m_Output_RetJoint[2], solve_para->m_Output_RetJoint[3]
			, solve_para->m_Output_RetJoint[4], solve_para->m_Output_RetJoint[5], solve_para->m_Output_RetJoint[6]);

		if (solve_para->m_Output_RetJoint[5] > 1)
		{
			FX_DOUBLE N;
			FX_DOUBLE J6;
			FX_DOUBLE P;
			J6 = solve_para->m_Output_RetJoint[5];
			if (J6 > solve_para->m_Output_RunLmtP[5])
			{
				J6 = solve_para->m_Output_RunLmtP[5];
			}
			P = SPC->lmtj67_pp[0] * J6 * J6 + SPC->lmtj67_pp[1] * J6 + SPC->lmtj67_pp[2];
			if (solve_para->m_Output_RunLmtP[6] > P)
			{
				solve_para->m_Output_RunLmtP[6] = P;
			}
			if (solve_para->m_Output_RetJoint[6] > solve_para->m_Output_RunLmtP[6])
			{
				solve_para->m_Output_IsJntExd = FX_TRUE;
				solve_para->m_Output_JntExdTags[6] = FX_TRUE;
				exdabs = FX_Fabs(solve_para->m_Output_RetJoint[6] - solve_para->m_Output_RunLmtP[6]);
				if (solve_para->m_Output_JntExdABS < exdabs)
				{
					solve_para->m_Output_JntExdABS = exdabs;
				}
			}

			N = SPC->lmtj67_pn[0] * J6 * J6 + SPC->lmtj67_pn[1] * J6 + SPC->lmtj67_pn[2];
			if (solve_para->m_Output_RunLmtN[6] < N)
			{
				solve_para->m_Output_RunLmtN[6] = N;
			}
			if (solve_para->m_Output_RetJoint[6] < solve_para->m_Output_RunLmtN[6])
			{
				solve_para->m_Output_IsJntExd = FX_TRUE;
				solve_para->m_Output_JntExdTags[6] = FX_TRUE;
				exdabs = FX_Fabs(solve_para->m_Output_RetJoint[6] - solve_para->m_Output_RunLmtN[6]);
				if (solve_para->m_Output_JntExdABS < exdabs)
				{
					solve_para->m_Output_JntExdABS = exdabs;
				}
			}


			return FX_TRUE;
		}


		if (solve_para->m_Output_RetJoint[5] < -1)
		{
			FX_DOUBLE N ;
			FX_DOUBLE J6;
			FX_DOUBLE P;
			J6 = solve_para->m_Output_RetJoint[5];
			if (J6 > solve_para->m_Output_RunLmtP[5])
			{
				J6 = solve_para->m_Output_RunLmtP[5];
			}
			P = SPC->lmtj67_np[0] * J6 * J6 + SPC->lmtj67_np[1] * J6 + SPC->lmtj67_np[2];
			if (solve_para->m_Output_RunLmtP[6] > P)
			{
				solve_para->m_Output_RunLmtP[6] = P;
			}
			if (solve_para->m_Output_RetJoint[6] > solve_para->m_Output_RunLmtP[6])
			{
				solve_para->m_Output_IsJntExd = FX_TRUE;
				solve_para->m_Output_JntExdTags[6] = FX_TRUE;
				exdabs = FX_Fabs(solve_para->m_Output_RetJoint[6] - solve_para->m_Output_RunLmtP[6]);
				if (solve_para->m_Output_JntExdABS < exdabs)
				{
					solve_para->m_Output_JntExdABS = exdabs;
				}
			}

			N = SPC->lmtj67_nn[0] * J6 * J6 + SPC->lmtj67_nn[1] * J6 + SPC->lmtj67_nn[2];
			if (solve_para->m_Output_RunLmtN[6] < N)
			{
				solve_para->m_Output_RunLmtN[6] = N;
			}
			if (solve_para->m_Output_RetJoint[6] < solve_para->m_Output_RunLmtN[6])
			{
				solve_para->m_Output_IsJntExd = FX_TRUE;
				solve_para->m_Output_JntExdTags[6] = FX_TRUE;
				exdabs = FX_Fabs(solve_para->m_Output_RetJoint[6] - solve_para->m_Output_RunLmtN[6]);
				if (solve_para->m_Output_JntExdABS < exdabs)
				{
					solve_para->m_Output_JntExdABS = exdabs;
				}
			}


			return FX_TRUE;
		}
	}
	else
	{
		FX_INT32L i;
		
		if (FX_SolveJ567ZNYZ(NSP, solve_para->m_Input_ZSP_Angle, solve_para->m_Input_IK_RefJoint, solve_para->m_Output_RetJoint, &(solve_para->m_Output_IsDeg[5]), solve_para->m_DGR2) == FX_FALSE)
		{
			return FX_FALSE;
		}

		solve_para->m_Output_IsJntExd = FX_FALSE;
		for (i = 0; i < 7; i++)
		{
			solve_para->m_Output_RunLmtP[i] = pRobot->m_Lmt.m_JLmtPos_P[i];
			solve_para->m_Output_RunLmtN[i] = pRobot->m_Lmt.m_JLmtPos_N[i];
			if (solve_para->m_Output_RetJoint[i] < solve_para->m_Output_RunLmtN[i] )
			{
				solve_para->m_Output_IsJntExd = FX_TRUE;
				solve_para->m_Output_JntExdTags[i] = FX_TRUE;
				exdabs = FX_Fabs(solve_para->m_Output_RetJoint[i] - solve_para->m_Output_RunLmtN[i]);
				if (solve_para->m_Output_JntExdABS < exdabs)
				{
					solve_para->m_Output_JntExdABS = exdabs;
				}
			}
			else if (solve_para->m_Output_RetJoint[i] > solve_para->m_Output_RunLmtP[i])
			{
				solve_para->m_Output_IsJntExd = FX_TRUE;
				solve_para->m_Output_JntExdTags[i] = FX_TRUE;

				exdabs = FX_Fabs(solve_para->m_Output_RetJoint[i] - solve_para->m_Output_RunLmtP[i]);
				if (solve_para->m_Output_JntExdABS < exdabs)
				{
					solve_para->m_Output_JntExdABS = exdabs;
				}
			}
			else
			{
				solve_para->m_Output_JntExdTags[i] = FX_FALSE;
			}
		}

		printf("EG:ret_J=[%lf %lf %lf %lf %lf %lf %lf]\n", solve_para->m_Output_RetJoint[0], solve_para->m_Output_RetJoint[1], solve_para->m_Output_RetJoint[2], solve_para->m_Output_RetJoint[3]
			, solve_para->m_Output_RetJoint[4], solve_para->m_Output_RetJoint[5], solve_para->m_Output_RetJoint[6]);

	}

	return FX_TRUE;
}

FX_BOOL  FX_Robot_Kine_IK_NSP(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para)
{
	FX_LOG_INFO("[FxRobot - FX_Robot_Kine_IK_NSP]\n");

	FX_Robot* pRobot;
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	if (pRobot->m_RobotType == FX_ROBOT_TYPE_DL)
	{
		return FX_FALSE;
	}
	else if (pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_SRS || pRobot->m_RobotType == FX_ROBOT_TYPE_PILOT_CCS)
	{
		return FX_InvKineNSP_Pilot(RobotSerial, solve_para);
	}
	else
	{
		return FX_FALSE;
	}
	return FX_FALSE;
}


FX_VOID InertiaTran(Matrix3 input_i_b, Matrix3 input_b2a, Matrix3 output_i_a)
{
	{
		Matrix3 ma2b;
		Matrix3 mtmp;
		FX_M33Trans(input_b2a, ma2b);
		FX_MMM33(input_b2a, input_i_b, mtmp);
		FX_MMM33(mtmp, ma2b, output_i_a);
	}

	{
		Matrix3 ma2b;
		Matrix3 mtmp;
		FX_M33Trans(input_b2a, ma2b);
		FX_MMM33(input_b2a, input_i_b, mtmp);
		FX_MMM33(mtmp, ma2b, output_i_a);
	}
	
}




FX_BOOL  FX_Robot_Kine_GetLinkPG(FX_INT32L RobotSerial, FX_DOUBLE PG[7][4][4])
{
	FX_Robot* pRobot;
	FX_KineBase* kine;
	FX_INT32L i;
	
	if (RobotSerial < 0 || RobotSerial >= MAX_RUN_ROBOT_NUM)
	{
		return FX_FALSE;
	}
	pRobot = (FX_Robot*)&m_Robot[RobotSerial];
	kine = &pRobot->m_KineBase;
	for ( i = 0; i < 7; i++)
	{
		FX_M44Copy(kine->m_JointPG[i], PG[i]);
	}
	return FX_TRUE;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FX_BOOL  FX_Robot_PLN_MOVL(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC, Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT8* OutPutPath)
{
	FX_LOG_INFO("[FxRobot - FX_Robot_PLN_MOVL]\n");

	FX_DOUBLE start_pos[6];
	FX_DOUBLE end_pos[6];
	FX_DOUBLE refJ[7];
	FX_INT32L i = 0;

	for (i = 0; i < 6; i++)
	{
		start_pos[i] = Start_XYZABC[i];
		end_pos[i] = End_XYZABC[i];
		refJ[i] = Ref_Joints[i];
	}
	refJ[i] = Ref_Joints[i];

	FX_DOUBLE jerk = ACC*10;

	FXSpln Spln = AxisPln_Create();
	FX_INT32 result = AxisPln_OnMovL(Spln,RobotSerial, start_pos,end_pos,refJ,Vel,ACC,jerk,OutPutPath);
	AxisPln_Destroy(Spln);

	if (result == 1)
	{
		return FX_TRUE;
	}
	else
	{
		return FX_FALSE;
	}

	return FX_FALSE;
}

FX_BOOL  FX_Robot_PLN_MOVL_KeepJ(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_CHAR* OutPutPath)
{
	FX_LOG_INFO("[FxRobot - FX_Robot_PLN_MOVL - KeepJ]\n");

	Vect7 start_pos;
	Vect7 end_pos;
	FX_INT32L i = 0;

	for (i = 0; i < 7; i++)
	{
		start_pos[i] = startjoints[i];
		end_pos[i] = stopjoints[i];
	}

	FXSpln Spln = AxisPln_Create();
	FX_INT32 result = AxisPln_OnMovL_KeepJ(Spln, RobotSerial, start_pos, end_pos, vel, OutPutPath);
	AxisPln_Destroy(Spln);

	if (result == 1)
	{
		return FX_TRUE;
	}
	else
	{
		return FX_FALSE;
	}

	return FX_FALSE;
}

////Parameters Identification
FX_INT32  FX_Robot_Iden_LoadDyn(FX_INT32 TYPE, FX_CHAR* path, FX_DOUBLE* mass, Vect3 mr, Vect6 I)
{
	FX_LOG_INFO("[FxRobot - FX_Robot_Iden_LoadDyn]\n");

	LoadDynamicPara DynPara;
//	FX_INT32 robotType = 1;

//	if (Type == FX_ROBOT_TYPE_PILOT_CCS)
//	{
//		ISCCS = FX_TRUE;
//	}

	FX_INT32 ret = OnCalLoadDyn(&DynPara, TYPE, path);

	*mass = DynPara.m;

	mr[0] = DynPara.r[0];
	mr[1] = DynPara.r[1];
	mr[2] = DynPara.r[2];

	I[0] = DynPara.I[0];
	I[1] = DynPara.I[1];
	I[2] = DynPara.I[2];
	I[3] = DynPara.I[3];
	I[4] = DynPara.I[4];
	I[5] = DynPara.I[5];


	printf("FX_Robot_Iden_LoadDyn ret=%d\n",ret);

	return ret;
}

FX_VOID   FX_XYZABC2Matrix4DEG(FX_DOUBLE xyzabc[6], FX_DOUBLE m[4][4])
{
	FX_DOUBLE angx = xyzabc[3];
	FX_DOUBLE angy = xyzabc[4];
	FX_DOUBLE angz = xyzabc[5];
	FX_DOUBLE sa;
	FX_DOUBLE sb;
	FX_DOUBLE sr;
	FX_DOUBLE ca;
	FX_DOUBLE cb;
	FX_DOUBLE cr;

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

FX_BOOL FX_Matrix42XYZABCDEG(FX_DOUBLE m[4][4],FX_DOUBLE xyzabc[6])
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
    return FX_TRUE;
}
