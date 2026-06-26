#include "FXArmKinematics.h"
#include "FXLog.h"
#include "FXMatrix.h"
#include <stdio.h>
#include <stdlib.h>

CFxKineIF::CFxKineIF()
{
	m_InitTag = FX_FALSE;
}

CFxKineIF::~CFxKineIF()
{

}

FX_BOOL CFxKineIF::OnInitEnv(FX_INT32 RobotSerial, FX_INT32 *type, FX_DOUBLE DH[8][4], FX_DOUBLE PNVA[8][4], FX_DOUBLE BOUND[4][3],
								FX_DOUBLE GRV[3], FX_DOUBLE MASS[7], FX_DOUBLE MCP[7][3], FX_DOUBLE I[7][6])
{
    m_InitTag = FX_FALSE;
    if (m_Env.OnInitEnv(RobotSerial, type, DH, PNVA, BOUND, GRV, MASS, MCP, I) == FX_FALSE)
    {
        FX_LOG_ERRO("OnInitEnv: Failed in Arm%d environment initialization", RobotSerial);
        return FX_FALSE;
    }

    Kine_Serial = RobotSerial;
    FX_INT32 RobotType = *type;

    Matrix4 tool;
    FX_IdentM44(tool);

    if (!FX_Robot_Init_Type(Kine_Serial, RobotType))
    {
        FX_LOG_ERRO("OnInitEnv: invalid robot type=%d for Arm%d", RobotType, RobotSerial);
        return FX_FALSE;
    }
    if (!FX_Robot_Init_Kine(Kine_Serial, DH))
    {
        FX_LOG_ERRO("OnInitEnv: failed to initialize DH paramters for Arm%d", RobotSerial);
        return FX_FALSE;
    }
    if (!FX_Robot_Init_Lmt(Kine_Serial, PNVA, BOUND))
    {
        FX_LOG_ERRO("OnInitEnv: failed to initialize limits for Arm%d", RobotSerial);
        return FX_FALSE;
    }
    if (!FX_Robot_Tool_Set(Kine_Serial, tool))
    {
        FX_LOG_ERRO("OnInitEnv: failed to set default tool for Arm%d", RobotSerial);
        return FX_FALSE;
    }

    m_InitTag = FX_TRUE;
    
    return FX_TRUE;
}

FX_BOOL CFxKineIF::OnSetTool(FX_INT32 Kine_Serial, Matrix4 tool)
{
    if (m_InitTag == FX_FALSE)
    {
        FX_LOG_ERRO("OnSetTool: called before initialization");
        return FX_FALSE;
    }
    if (Kine_Serial < 0 || Kine_Serial > 1)
    {
        FX_LOG_ERRO("OnSetTool: invalid robot serial=%d", Kine_Serial);
        return FX_FALSE;
    }
    if (!FX_Robot_Tool_Set(Kine_Serial, tool))
    {
        FX_LOG_ERRO("OnSetTool: failed, robot_serial=%d", Kine_Serial);
        return FX_FALSE;
    }
    
    return FX_TRUE;
}

FX_VOID CFxKineIF::OnRmvTool(FX_INT32 Kine_Serial)
{
    if (m_InitTag == FX_FALSE)
    {
        FX_LOG_ERRO("OnRmvTool: called before initialization");
        return;
    }
    if (Kine_Serial < 0 || Kine_Serial > 1)
    {
        FX_LOG_ERRO("OnRmvTool: invalid robot serial=%d", Kine_Serial);
        return;
    }
    FX_Robot_Tool_Rmv(Kine_Serial);
    
}

FX_BOOL CFxKineIF::OnSolveArmFK(Vect7 joints, Matrix4 pgos)
{
    if (m_InitTag == FX_FALSE)
    {
        FX_LOG_ERRO("OnSolveArmFK: called before initialization");
        return FX_FALSE;
    }
    if (Kine_Serial < 0 || Kine_Serial > 1)
    {
        FX_LOG_ERRO("OnSolveArmFK: invalid robot serial=%d", Kine_Serial);
        return FX_FALSE;
    }
    if (!FX_Robot_Kine_FK(Kine_Serial, joints, pgos))
    {
        FX_LOG_ERRO("OnSolveArmFK: failed, robot_serial=%d", Kine_Serial);
        return FX_FALSE;
    }
    
    return FX_TRUE;
}

FX_BOOL CFxKineIF::OnSolveArmJcb(Vect7 joints, FX_DOUBLE jcb[6][7])
{
    FX_Jacobi jcb_t;
    if (m_InitTag == FX_FALSE)
    {
        FX_LOG_ERRO("OnSolveArmJcb: called before initialization");
        return FX_FALSE;
    }
    if (Kine_Serial < 0 || Kine_Serial > 1)
    {
        FX_LOG_ERRO("OnSolveArmJcb: invalid robot serial=%d", Kine_Serial);
        return FX_FALSE;
    }
    if (FX_Robot_Kine_Jacb(Kine_Serial, joints, &jcb_t) == FX_FALSE)
    {
        FX_LOG_ERRO("OnSolveArmJcb: failed, robot_serial=%d", Kine_Serial);
        return FX_FALSE;
    }
    FX_M67Copy(jcb_t.m_Jcb, jcb);

    return FX_TRUE;
}

FX_BOOL CFxKineIF::OnSolveArmIK(FX_InvKineSolvePara *solve_para)
{
    if (m_InitTag == FX_FALSE)
    {
        FX_LOG_ERRO("OnSolveArmIK: called before initialization");
        return FX_FALSE;
    }
    if (Kine_Serial < 0 || Kine_Serial > 1)
    {
        FX_LOG_ERRO("OnSolveArmIK: invalid robot serial=%d", Kine_Serial);
        return FX_FALSE;
    }
    if (!FX_Robot_Kine_IK(Kine_Serial, solve_para))
    {
        FX_LOG_WARN("CFxKineIF::OnSolveArmIK failed, robot_serial=%d", Kine_Serial);
        return FX_FALSE;
    }
    
    return FX_TRUE;
}

FX_BOOL CFxKineIF::OnGetArmLmt(FX_INT32 &type, FX_DOUBLE PosNeg[8], FX_DOUBLE PosPos[8], FX_DOUBLE VelLmt[8], FX_DOUBLE AccLmt[8])
{
    return m_Env.OnGetArmLmt(type, PosNeg, PosPos, VelLmt, AccLmt);
}
