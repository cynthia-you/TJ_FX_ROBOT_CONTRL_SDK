#include "FX_Env.h"
#include "stdio.h"
#include "stdlib.h"

static FX_BOOL IFEnv_Log_Switch = FX_FALSE;

CFxIFEnv::CFxIFEnv()
{
	m_IsEnvValid = FX_FALSE;
}

CFxIFEnv::~CFxIFEnv()
{
}

FX_VOID CFxIFEnv::On_Log_Switch(FX_INT32 log_tag_input)
{
	IFEnv_Log_Switch = (log_tag_input == 1) ? FX_TRUE : FX_FALSE;
}

FX_BOOL CFxIFEnv::OnInitEnv(FX_INT32 RobotSerial, FX_INT32 *type, FX_DOUBLE DH[8][4], FX_DOUBLE PNVA[8][4], FX_DOUBLE BOUND[4][3], FX_DOUBLE GRV[3], FX_DOUBLE MASS[7], FX_DOUBLE MCP[7][3], FX_DOUBLE I[7][6])
{
	m_RobotSerial = RobotSerial;
	m_TYPE[m_RobotSerial] = *type;

	FX_INT32 i = 0;
	FX_INT32 j = 0;

	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < 4; j++)
		{
			m_DH[m_RobotSerial][i][j] = DH[i][j];
			m_PNVA[m_RobotSerial][i][j] = PNVA[i][j];
		}
	}

	for (j = 0; j < 3; j++)
	{
		m_GRV[m_RobotSerial][j] = GRV[j];
	}

	for (i = 0; i < 7; i++)
	{
		m_Mass[m_RobotSerial][j] = MASS[j];

		for (j = 0; j < 3; j++)
		{
			m_MCP[m_RobotSerial][i][j] = MCP[i][j];
		}

		for (j = 0; j < 6; j++)
		{
			m_I[m_RobotSerial][i][j] = I[i][j];
		}
	}

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 3; j++)
		{
			m_BD[m_RobotSerial][i][j] = BOUND[i][j];
		}
	}

	if (IFEnv_Log_Switch)
	{
		printf("type[0]=%d  type[1]=%d\n", m_TYPE[0], m_TYPE[1]);
		printf("DH:%f %f %f %f\n", m_DH[0][0][0], m_DH[0][0][1], m_DH[0][0][2], m_DH[0][0][3]);
	}

	m_IsEnvValid = FX_TRUE;

	return FX_TRUE;
}

// FX_BOOL CFxIFEnv::OnInitEnv(FX_CHAR *env_path, FX_INT32 RobotSerial)
// {
// 	m_IsEnvValid = FX_FALSE;

// 	FX_BOOL ret = OnLoadArmCfg(env_path, m_TYPE, m_GRV, m_DH, m_PNVA, m_BD, m_Mass, m_MCP, m_I);
// 	m_RobotSerial = RobotSerial;

// 	if (IFEnv_Log_Switch)
// 	{
// 		printf("type[0]=%d  type[1]=%d\n", m_TYPE[0], m_TYPE[1]);
// 		printf("DH:%f %f %f %f\n", m_DH[0][0][0], m_DH[0][0][1], m_DH[0][0][2], m_DH[0][0][3]);
// 	}
// 	if (ret == FX_TRUE)
// 	{
// 		m_IsEnvValid = FX_TRUE;
// 	}
// 	else
// 	{
// 		m_IsEnvValid = FX_FALSE;
// 	}

// 	return FX_TRUE;
// }

FX_BOOL CFxIFEnv::OnGetArmType(FX_INT32 &type)
{
	if (m_IsEnvValid == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (m_RobotSerial < 0 || m_RobotSerial > 1)
	{
		return FX_FALSE;
	}
	type = m_TYPE[m_RobotSerial];
	return FX_TRUE;
}

FX_BOOL CFxIFEnv::OnGetArmLmt(FX_INT32 &type, FX_DOUBLE PosNeg[8], FX_DOUBLE PosPos[8], FX_DOUBLE VelLmt[8], FX_DOUBLE AccLmt[8])
{
	if (m_IsEnvValid == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (m_RobotSerial < 0 || m_RobotSerial > 1)
	{
		return FX_FALSE;
	}
	type = m_TYPE[m_RobotSerial];

	for (FX_INT32 i = 0; i < 7; i++)
	{
		PosNeg[i] = m_PNVA[m_RobotSerial][i][1];
		PosPos[i] = m_PNVA[m_RobotSerial][i][0];
		VelLmt[i] = m_PNVA[m_RobotSerial][i][2];
		AccLmt[i] = m_PNVA[m_RobotSerial][i][3];
	}

	return FX_TRUE;
}

FX_BOOL CFxIFEnv::OnGetArmKinePara(FX_INT32 &type, FX_DOUBLE dh[8][4], FX_DOUBLE pnva[7][4], FX_DOUBLE bd[4][3])
{
	FX_INT32 i;
	FX_INT32 j;
	if (m_IsEnvValid == FX_FALSE)
	{
		printf("EnvValid is false\n");
		return FX_FALSE;
	}
	if (m_RobotSerial < 0 || m_RobotSerial > 1)
	{
		printf("Invalid robot serial: %d\n", m_RobotSerial);
		return FX_FALSE;
	}

	type = m_TYPE[m_RobotSerial];

	if (IFEnv_Log_Switch)
		printf("OnGetArmKinePara type=%d   m_type=%d m_robotSerial=%d\n", type, m_TYPE[m_RobotSerial], m_RobotSerial);

	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < 4; j++)
		{
			dh[i][j] = m_DH[m_RobotSerial][i][j];
		}
	}

	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 4; j++)
		{
			pnva[i][j] = m_PNVA[m_RobotSerial][i][j];
		}
	}

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 3; j++)
		{
			bd[i][j] = m_BD[m_RobotSerial][i][j];
		}
	}

	return FX_TRUE;
}
FX_BOOL CFxIFEnv::OnGetArmDynPara(FX_DOUBLE grv[3], FX_DOUBLE mass[7], FX_DOUBLE mcp[7][3], FX_DOUBLE I[7][6])
{
	FX_INT32 i;
	FX_INT32 j;
	if (m_IsEnvValid == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (m_RobotSerial < 0 || m_RobotSerial > 1)
	{
		return FX_FALSE;
	}
	for (i = 0; i < 3; i++)
	{
		grv[i] = m_GRV[m_RobotSerial][i];
	}
	for (i = 0; i < 7; i++)
	{
		mass[i] = m_Mass[m_RobotSerial][i];
	}

	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 3; j++)
		{
			mcp[i][j] = m_MCP[m_RobotSerial][i][j];
		}

		for (j = 0; j < 6; j++)
		{
			I[i][j] = m_I[m_RobotSerial][i][j];
		}
	}

	return FX_TRUE;
}

FX_BOOL CFxIFEnv::OnCheckEnvValid()
{
	return m_IsEnvValid;
}

FX_BOOL CFxIFEnv::OnLoadArmCfg(FX_CHAR *path, FX_INT32 TYPE[2], FX_DOUBLE GRV[2][3], FX_DOUBLE DH[2][8][4], FX_DOUBLE PNVA[2][7][4], FX_DOUBLE BD[2][4][3],
							   FX_DOUBLE Mass[2][7], FX_DOUBLE MCP[2][7][3], FX_DOUBLE I[2][7][6])
{

	FX_INT32 i = 0;
	FX_INT32 j = 0;
	FX_CHAR c;
	FILE *fp = fopen(path, "rb");
	if (fp == NULL)
	{
		return FX_FALSE;
	}

	for (i = 0; i < 2; i++)
	{
		if (fscanf(fp, "%d,%lf,%lf,%lf,%c", &TYPE[i], &GRV[i][0], &GRV[i][1], &GRV[i][2], &c) != 5)
		{
			fclose(fp);
			return FX_FALSE;
		}
		if (c != 0x0a)
		{
			fclose(fp);
			return FX_FALSE;
		}

		for (j = 0; j < 7; j++)
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

		if (fscanf(fp, "%lf,%lf,%lf,%lf,%c", &DH[i][7][0], &DH[i][7][1], &DH[i][7][2], &DH[i][7][3],
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
					   &BD[i][j][0], &BD[i][j][1], &BD[i][j][2], &c) != 4)
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
	return FX_TRUE;
}
