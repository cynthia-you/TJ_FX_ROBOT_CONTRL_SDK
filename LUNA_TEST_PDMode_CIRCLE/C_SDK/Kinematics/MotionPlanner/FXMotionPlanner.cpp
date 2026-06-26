#include "FXMotionPlanner.h"
#include "FXSkyeBodyKinematics.h"
#include "FXMatrix.h"
#include "FXLog.h"

CFxPln::CFxPln()
{
    m_LastError = FX_PLANNER_SUCCESS;
}

CFxPln::~CFxPln()
{
}

FX_INT32 CFxPln::GetLastError() const
{
    return m_LastError;
}
FX_BOOL CFxPln::OnInitEnv_SingleArm(FX_INT32 RobotSerial, FX_INT32 *type, FX_DOUBLE DH[8][4], FX_DOUBLE PNVA[8][4], FX_DOUBLE BOUND[4][3])
{
    if (RobotSerial < 0 || RobotSerial > 1)
    {
        FX_LOG_ERRO("OnInitEnv_SingleArm: invalid robot_serial=%d", RobotSerial);
        return FX_FALSE;
    }

    FX_DOUBLE GRV[3] = {0};
    FX_DOUBLE MASS[7] = {0};
    FX_DOUBLE MCP[7][3] = {{0}};
    FX_DOUBLE I[7][6] = {{0}};
    FX_BOOL result = m_KineIF.OnInitEnv(RobotSerial, type, DH, PNVA, BOUND, GRV, MASS, MCP, I);
    if (result == FX_FALSE)
    {
        FX_LOG_ERRO("OnInitEnv_SingleArm: shared kinematics init failed, robot_serial=%d", RobotSerial);
    }

    if (RobotSerial == 0)
    {
        FX_BOOL result_left = m_Kine_Left_Arm.OnInitEnv(0, type, DH, PNVA, BOUND, GRV, MASS, MCP, I);
        if (!(result_left && result))
        {
            FX_LOG_ERRO("OnInitEnv_SingleArm: left arm init failed");
        }
        return (result_left && result);
    }
    else if (RobotSerial == 1)
    {
        FX_BOOL result_right = m_Kine_Right_Arm.OnInitEnv(1, type, DH, PNVA, BOUND, GRV, MASS, MCP, I);
        if (!(result_right && result))
        {
            FX_LOG_ERRO("OnInitEnv_SingleArm: right arm init failed");
        }
        return (result_right && result);
    }

    return FX_FALSE;
}

FX_BOOL CFxPln::OnPlnSetTool(FX_INT32 RobotSerial, Matrix4 tool)
{
    if (!m_KineIF.OnSetTool(RobotSerial, tool))
    {
        FX_LOG_ERRO("OnPlnSetTool: failed in shared kinematics, robot_serial=%d", RobotSerial);
        return FX_FALSE;
    }

    if (RobotSerial == 0)
    {
        if (!m_Kine_Left_Arm.OnSetTool(0, tool))
        {
            FX_LOG_ERRO("OnPlnSetTool: failed on left arm");
            return FX_FALSE;
        }
    }
    else
    {
        if (!m_Kine_Right_Arm.OnSetTool(1, tool))
        {
            FX_LOG_ERRO("OnPlnSetTool: failed on right arm");
            return FX_FALSE;
        }
    }

    return FX_TRUE;
}

FX_VOID CFxPln::OnPlnRemoveTool(FX_INT32 RobotSerial)
{
    m_KineIF.OnRmvTool(RobotSerial);

    if (RobotSerial == 0)
    {
        m_Kine_Left_Arm.OnRmvTool(0);
    }
    else
    {
        m_Kine_Right_Arm.OnRmvTool(1);
    }
}

FX_BOOL CFxPln::OnMovJ(Vect7 start_joint, Vect7 end_joint, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio, FX_INT32 freq, CPointSet *ret_pset)
{
    m_LastError = FX_PLANNER_ERROR;
    if (m_KineIF.m_InitTag == FX_FALSE)
    {
        FX_LOG_ERRO("OnMovJ: called before initialization");
        return FX_FALSE;
    }
    if (m_KineIF.Kine_Serial < 0 || m_KineIF.Kine_Serial > 1)
    {
        FX_LOG_ERRO("OnMovJ: invalid robot_serial=%d", m_KineIF.Kine_Serial);
        return FX_FALSE;
    }

    FX_INT32 type = 0;
    FX_DOUBLE lmt_pos[8] = {0};
    FX_DOUBLE lmt_neg[8] = {0};
    FX_DOUBLE lmt_vel[8] = {0};
    FX_DOUBLE lmt_acc[8] = {0};
    if (!m_KineIF.OnGetArmLmt(type, lmt_neg, lmt_pos, lmt_vel, lmt_acc))
    {
        FX_LOG_ERRO("OnMovJ: failed to get arm limits, robot_serial=%d", m_KineIF.Kine_Serial);
        return FX_FALSE;
    }

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
    m_AxisJointPln.OnSetFreq(freq);

    FX_BOOL result = m_AxisJointPln.OnMovJoint(m_KineIF.Kine_Serial, start_joint, end_joint, vel_ratio, acc_ratio, ret_pset);
    if (result == FX_FALSE)
    {
        m_LastError = m_AxisJointPln.GetLastError();
        FX_LOG_WARN("OnMovJ: planning failed, robot_serial=%d", m_KineIF.Kine_Serial);
        return FX_FALSE;
    }

    m_LastError = FX_PLANNER_SUCCESS;
    return FX_TRUE;
}

FX_BOOL CFxPln::OnMovL(Vect6 Start_XYZABC, Vect6 End_XYZABC, Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32 freq, CPointSet *pset)
{
    m_LastError = FX_PLANNER_ERROR;
    if (m_KineIF.m_InitTag == FX_FALSE)
    {
        FX_LOG_ERRO("OnMovL: called before initialization");
        return FX_FALSE;
    }
    if (m_KineIF.Kine_Serial < 0 || m_KineIF.Kine_Serial > 1)
    {
        FX_LOG_ERRO("OnMovL: invalid robot_serial=%d", m_KineIF.Kine_Serial);
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
    if (result == FX_FALSE)
    {
        m_LastError = Spln.GetLastError();
        FX_LOG_WARN("OnMovL: planning failed, robot_serial=%d", m_KineIF.Kine_Serial);
        return FX_FALSE;
    }

    m_LastError = FX_PLANNER_SUCCESS;
    return FX_TRUE;
}

FX_BOOL CFxPln::OnMovL_KeepJ(Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq, CPointSet *pset)
{
    m_LastError = FX_PLANNER_ERROR;
    if (m_KineIF.m_InitTag == FX_FALSE)
    {
        FX_LOG_ERRO("OnMovL_KeepJ: called before initialization");
        return FX_FALSE;
    }
    if (m_KineIF.Kine_Serial < 0 || m_KineIF.Kine_Serial > 1)
    {
        FX_LOG_ERRO("OnMovL_KeepJ: invalid robot_serial=%d", m_KineIF.Kine_Serial);
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
    if (result == FX_FALSE)
    {
        m_LastError = Spln.GetLastError();
        FX_LOG_WARN("OnMovL_KeepJ: planning failed, robot_serial=%d", m_KineIF.Kine_Serial);
        return FX_FALSE;
    }

    m_LastError = FX_PLANNER_SUCCESS;
    return FX_TRUE;
}

FX_VOID CFxPln::XYZABC2Matrix4_DEG(FX_DOUBLE xyzabc[6], FX_DOUBLE m[4][4])
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

FX_VOID CFxPln::Matrix42XYZABC_DEG(FX_DOUBLE m[4][4], FX_DOUBLE xyzabc[6])
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

FX_BOOL CFxPln::MultiPoints_Set_MovL_Start(Vect7 refjoints, Vect6 Start_XYZABC, Vect6 End_XYZABC,
                                              FX_DOUBLE Allow_Range, FX_INT32 ZSP_Type, Vect6 ZSP_Para, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32 freq)
{
    m_LastError = FX_PLANNER_ERROR;
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
    FX_BOOL result = m_AxisPln.OnMovL_ZSP(m_KineIF.Kine_Serial, refJ, start_pos, end_pos, Vel, ACC, jerk, ZSP_Type, ZSP_Para, Allow_Range, FX_MOVL_START);
    if (result == FX_FALSE)
    {
        m_LastError = m_AxisPln.GetLastError();
        return FX_FALSE;
    }
    m_LastError = FX_PLANNER_SUCCESS;
    return FX_TRUE;
}

FX_BOOL CFxPln::MultiPoints_Set_MovL_NextPoints(Vect6 Next_XYZABC,
                                                   FX_DOUBLE Allow_Range, FX_INT32 ZSP_Type, Vect6 ZSP_Para, FX_DOUBLE Vel, FX_DOUBLE ACC)
{
    m_LastError = FX_PLANNER_ERROR;
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

    FX_BOOL result = m_AxisPln.OnMovL_ZSP(m_KineIF.Kine_Serial, refJ, start_pos, end_pos, Vel, ACC, jerk, ZSP_Type, ZSP_Para, Allow_Range, FX_MOVL_NEXT);
    if (result == FX_FALSE)
    {
        m_LastError = m_AxisPln.GetLastError();
        return FX_FALSE;
    }
    m_LastError = FX_PLANNER_SUCCESS;
    return FX_TRUE;
}

FX_BOOL CFxPln::MultiPoints_Get_MovL_Path(CPointSet *ret_pset)
{
    m_LastError = FX_PLANNER_ERROR;
    FX_BOOL result = m_AxisPln.OnSendPoints(ret_pset);
    if (result == FX_FALSE)
    {
        m_LastError = m_AxisPln.GetLastError();
        FX_LOG_WARN("MultiPoints_Get_MovL_Path: returned no path points");
        return FX_FALSE;
    }
    m_LastError = FX_PLANNER_SUCCESS;
    return FX_TRUE;
}

//////////////////////////////DualArm with FixBody
FX_BOOL CFxPln::OnMovL_DualArm_FixBody(ArmsSynchronousPlanningParams *ASPP, CPointSet *Arm0_Pln_Path, CPointSet *Arm1_Pln_Path)
{
    m_LastError = FX_PLANNER_ERROR;
    if (!m_Kine_Left_Arm.m_InitTag || !m_Kine_Right_Arm.m_InitTag)
    {
        FX_LOG_ERRO("OnMovL_DualArm_FixBody: called before both arms are initialized");
        return FX_FALSE;
    }

    if (ASPP->World_Co_Flag)
    {
        Matrix4 Left_Arm_Start_EE, Left_Arm_End_EE;
        XYZABC2Matrix4_DEG(ASPP->Arm0_Start_XYZABC, Left_Arm_Start_EE);
        XYZABC2Matrix4_DEG(ASPP->Arm0_End_XYZABC, Left_Arm_End_EE);

        Matrix4 Right_Arm_Start_EE, Right_Arm_End_EE;
        XYZABC2Matrix4_DEG(ASPP->Arm1_Start_XYZABC, Right_Arm_Start_EE);
        XYZABC2Matrix4_DEG(ASPP->Arm1_End_XYZABC, Right_Arm_End_EE);

        Matrix4 Shoulder_left, Shoulder_right;
        CFxKineMAX km;
        km.OnKineLR(ASPP->Skye_Body_Start_PRR, Shoulder_left, Shoulder_right);

        Matrix4 Shoulder_left_inv, Shoulder_right_inv;
        MatrixInv44(Shoulder_left, Shoulder_left_inv);
        MatrixInv44(Shoulder_right, Shoulder_right_inv);

        Matrix4 Left_Arm_Start_Base, Left_Arm_End_Base;
        FX_MMM44(Shoulder_left_inv, Left_Arm_Start_EE, Left_Arm_Start_Base);
        FX_MMM44(Shoulder_left_inv, Left_Arm_End_EE, Left_Arm_End_Base);

        Matrix4 Right_Arm_Start_Base, Right_Arm_End_Base;
        FX_MMM44(Shoulder_right_inv, Right_Arm_Start_EE, Right_Arm_Start_Base);
        FX_MMM44(Shoulder_right_inv, Right_Arm_End_EE, Right_Arm_End_Base);

        Matrix42XYZABC_DEG(Left_Arm_Start_Base, ASPP->Arm0_Start_XYZABC);
        Matrix42XYZABC_DEG(Left_Arm_End_Base, ASPP->Arm0_End_XYZABC);
        Matrix42XYZABC_DEG(Right_Arm_Start_Base, ASPP->Arm1_Start_XYZABC);
        Matrix42XYZABC_DEG(Right_Arm_End_Base, ASPP->Arm1_End_XYZABC);
    }

    CAxisPln Spln;
    Spln.OnSetFreq(ASPP->Freq);

    FX_BOOL result = Spln.OnMovL_DualArm_FixBody(ASPP, Arm0_Pln_Path, Arm1_Pln_Path);
    if (result == FX_FALSE)
    {
        m_LastError = Spln.GetLastError();
        return FX_FALSE;
    }

    m_LastError = FX_PLANNER_SUCCESS;
    return FX_TRUE;
}
