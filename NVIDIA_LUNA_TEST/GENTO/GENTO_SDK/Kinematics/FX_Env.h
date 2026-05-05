#ifndef _FX_IF_ENV_H_
#define _FX_IF_ENV_H_
#include "../Common/FxType.h"

class CFxIFEnv
{
public:
	CFxIFEnv();
	~CFxIFEnv();
	FX_BOOL OnInitEnv(FX_INT32 RobotSerial, FX_INT32 *type, FX_DOUBLE DH[8][4], FX_DOUBLE PNVA[8][4], FX_DOUBLE BOUND[4][3],
					  FX_DOUBLE GRV[3], FX_DOUBLE MASS[7], FX_DOUBLE MCP[7][3], FX_DOUBLE I[7][6]);
	FX_BOOL OnGetArmType(FX_INT32 &type);
	FX_BOOL OnGetArmLmt(FX_INT32 &type, FX_DOUBLE PosNeg[8], FX_DOUBLE PosPos[8], FX_DOUBLE VelLmt[8], FX_DOUBLE AccLmt[8]);
	FX_BOOL OnGetArmKinePara(FX_INT32 &type, FX_DOUBLE dh[8][4], FX_DOUBLE pnva[7][4], FX_DOUBLE bd[4][3]);
	FX_BOOL OnGetArmDynPara(FX_DOUBLE grv[3], FX_DOUBLE mass[7], FX_DOUBLE mcp[7][3], FX_DOUBLE I[7][6]);
	FX_BOOL OnCheckEnvValid();

	FX_VOID On_Log_Switch(FX_INT32 log_tag_input);

protected:
	FX_BOOL m_IsEnvValid;

	FX_BOOL OnLoadArmCfg(FX_CHAR *path, FX_INT32 TYPE[2], FX_DOUBLE GRV[2][3], FX_DOUBLE DH[2][8][4], FX_DOUBLE PNVA[2][7][4], FX_DOUBLE BD[2][4][3],
						 FX_DOUBLE Mass[2][7], FX_DOUBLE MCP[2][7][3], FX_DOUBLE I[2][7][6]);

protected:
	FX_INT32 m_RobotSerial;
	FX_INT32 m_TYPE[2];
	FX_DOUBLE m_GRV[2][3];
	FX_DOUBLE m_DH[2][8][4];
	FX_DOUBLE m_PNVA[2][7][4];
	FX_DOUBLE m_BD[2][4][3];
	FX_DOUBLE m_Mass[2][7];
	FX_DOUBLE m_MCP[2][7][3];
	FX_DOUBLE m_I[2][7][6];
};

#endif
