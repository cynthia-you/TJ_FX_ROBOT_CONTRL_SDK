#include "FXEnviroment.h"
#include "FXLog.h"
#include "stdio.h"
#include "stdlib.h"

CFxIFEnv::CFxIFEnv()
{
	m_IsEnvValid = FX_FALSE;
}

CFxIFEnv::~CFxIFEnv()
{

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

	m_IsEnvValid = FX_TRUE;

	return FX_TRUE;
}

FX_BOOL CFxIFEnv::OnGetArmType(FX_INT32 &type)
{
	if (m_IsEnvValid == FX_FALSE)
	{
		FX_LOG_ERRO("OnGetArmType:EnvValid is false");
		return FX_FALSE;
	}
	if (m_RobotSerial < 0 || m_RobotSerial > 1)
	{
		FX_LOG_ERRO("OnGetArmType: Invalid robot serial: %d", m_RobotSerial);
		return FX_FALSE;
	}
	type = m_TYPE[m_RobotSerial];
	return FX_TRUE;
}

FX_BOOL CFxIFEnv::OnGetArmLmt(FX_INT32 &type, FX_DOUBLE PosNeg[8], FX_DOUBLE PosPos[8], FX_DOUBLE VelLmt[8], FX_DOUBLE AccLmt[8])
{
	if (m_IsEnvValid == FX_FALSE)
	{
		FX_LOG_ERRO("OnGetArmLmt:EnvValid is false");
		return FX_FALSE;
	}
	if (m_RobotSerial < 0 || m_RobotSerial > 1)
	{
		FX_LOG_ERRO("OnGetArmLmt: Invalid robot serial: %d", m_RobotSerial);
		return FX_FALSE;
	}
	type = m_TYPE[m_RobotSerial];

	for (FX_INT32 i = 0; i < 7; i++)
	{
		PosNeg[i] = m_PNVA[m_RobotSerial][i][1];
		PosPos[i] = m_PNVA[m_RobotSerial][i][0];
		VelLmt[i] = m_PNVA[m_RobotSerial][i][2];
		AccLmt[i] = m_PNVA[m_RobotSerial][i][3];
		FX_LOG_DEBG("<OnGetArmLmt> robot_serial=%d, joint=%d, pos_neg=%f, pos_pos=%f, vel_lmt=%f, acc_lmt=%f",
					m_RobotSerial, i, PosNeg[i], PosPos[i], VelLmt[i], AccLmt[i]);
	}

	return FX_TRUE;
}

FX_BOOL CFxIFEnv::OnGetArmKinePara(FX_INT32 &type, FX_DOUBLE dh[8][4], FX_DOUBLE pnva[7][4], FX_DOUBLE bd[4][3])
{
	FX_INT32 i;
	FX_INT32 j;
	if (m_IsEnvValid == FX_FALSE)
	{
		FX_LOG_ERRO("OnGetArmKinePara: EnvValid is false");
		return FX_FALSE;
	}
	if (m_RobotSerial < 0 || m_RobotSerial > 1)
	{
		FX_LOG_ERRO("OnGetArmKinePara: Invalid robot serial: %d", m_RobotSerial);
		return FX_FALSE;
	}

	type = m_TYPE[m_RobotSerial];

	FX_LOG_DEBG("<OnGetArmKinePara> type=%d   m_type=%d m_robotSerial=%d", type, m_TYPE[m_RobotSerial], m_RobotSerial);

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
		FX_LOG_ERRO("OnGetArmDynPara: EnvValid is false");
		return FX_FALSE;
	}
	if (m_RobotSerial < 0 || m_RobotSerial > 1)
	{
		FX_LOG_ERRO("OnGetArmDynPara: Invalid robot serial: %d", m_RobotSerial);
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