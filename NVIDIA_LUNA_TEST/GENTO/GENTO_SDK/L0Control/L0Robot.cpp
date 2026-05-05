#include "L0Robot.h"
#include <cstdio>
#include <cstring>

static unsigned char l0_local_log_tag = 1;

/*=============================================================================
 * 日志开关
 *============================================================================*/
void FX_L0_System_LocalLogOn(void)
{
    l0_local_log_tag = 1;
    RobotCtrl::OnLocalLogOn();
}

void FX_L0_System_LocalLogOff(void)
{
    l0_local_log_tag = 0;
    RobotCtrl::OnLocalLogOff();
}

/*=============================================================================
 * 连接处理
 *============================================================================*/
int FX_L0_System_connect(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4)
{
    return RobotCtrl::Link(ip1, ip2, ip3, ip4);
}

int FX_L0_System_Testconnect(void)
{
    return RobotCtrl::TestLink();
}

/*=============================================================================
 * 组包发送机制
 *============================================================================*/
int FX_L0_Communication_Clear(unsigned int timeout)
{
    return RobotCtrl::ClearSend(timeout);
}

int FX_L0_Communication_Send(void)
{
    return RobotCtrl::SetSend();
}

int FX_L0_Communication_SendWaitResponse(unsigned int time_out)
{
    return RobotCtrl::WaitSend(time_out);
}

/*=============================================================================
 * 系统操作
 *============================================================================*/
int FX_L0_System_GetVersion(void)
{
    return RobotCtrl::System_GetVersion();
}

int FX_L0_System_Reboot(void)
{
    return RobotCtrl::System_Reboot();
}

/*=============================================================================
 * 参数读写
 *============================================================================*/
int FX_L0_Param_GetInt(char name[30], int *ret_value)
{
    FX_INT32 val;
    if (RobotCtrl::Para_GetInt(name, &val))
    {
        *ret_value = val;
        return 1;
    }
    return 0;
}

int FX_L0_Param_GetFloat(char name[30], float *ret_value)
{
    FX_FLOAT val;
    if (RobotCtrl::Para_GetFloat(name, &val))
    {
        *ret_value = val;
        return 1;
    }
    return 0;
}

int FX_L0_Param_SetInt(char name[30], int target_value)
{
    return RobotCtrl::Para_SetInt(name, (int)target_value);
}

int FX_L0_Param_SetFloat(char name[30], float target_value)
{
    return RobotCtrl::Para_SetFloat(name, (float)target_value);
}

int FX_L0_Param_Save(void)
{
    return RobotCtrl::Para_Save();
}

/*=============================================================================
 * 末端透传数据通道
 *============================================================================*/
int FX_L0_Arm0_Terminal_ClearData(void)
{
    return RobotCtrl::Arm0_Terminal_ClearData();
}

int FX_L0_Arm0_Terminal_GetData(int *channel_type_ptr, unsigned char data_ptr[64])
{
    return RobotCtrl::Arm0_Terminal_GetData(channel_type_ptr, data_ptr);
}

int FX_L0_Arm0_Terminal_SetData(int channel_type, unsigned char *data_ptr, int data_len)
{
    return RobotCtrl::Arm0_Terminal_SetData(channel_type, data_ptr, data_len);
}

int FX_L0_Arm1_Terminal_ClearData(void)
{
    return RobotCtrl::Arm1_Terminal_ClearData();
}

int FX_L0_Arm1_Terminal_GetData(int *channel_type_ptr, unsigned char data_ptr[64])
{
    return RobotCtrl::Arm1_Terminal_GetData(channel_type_ptr, data_ptr);
}

int FX_L0_Arm1_Terminal_SetData(int channel_type, unsigned char *data_ptr, int data_len)
{
    return RobotCtrl::Arm1_Terminal_SetData(channel_type, data_ptr, data_len);
}

/*=============================================================================
 * 状态接口
 *============================================================================*/
int FX_L0_Arm0_State_GetServoErrorCode(int axis_id, int *error_code)
{
    return RobotCtrl::Arm0_State_GetServoErrorCode(axis_id, error_code);
}

int FX_L0_Arm1_State_GetServoErrorCode(int axis_id, int *error_code)
{
    return RobotCtrl::Arm1_State_GetServoErrorCode(axis_id, error_code);
}

int FX_L0_Head_State_GetServoErrorCode(int axis_id, int *error_code)
{
    return RobotCtrl::Head_State_GetServoErrorCode(axis_id, error_code);
}

int FX_L0_Body_State_GetServoErrorCode(int axis_id, int *error_code)
{
    return RobotCtrl::Body_State_GetServoErrorCode(axis_id, error_code);
}

int FX_L0_Lift_State_GetServoErrorCode(int axis_id, int *error_code)
{
    return RobotCtrl::Lift_State_GetServoErrorCode(axis_id, error_code);
}

int FX_L0_Arm0_State_Reset(void)
{
    return RobotCtrl::Arm0_State_Reset();
}

int FX_L0_Arm1_State_Reset(void)
{
    return RobotCtrl::Arm1_State_Reset();
}

int FX_L0_Head_State_Reset(void)
{
    return RobotCtrl::Head_State_Reset();
}

int FX_L0_Body_State_Reset(void)
{
    return RobotCtrl::Body_State_Reset();
}

int FX_L0_Lift_State_Reset(void)
{
    return RobotCtrl::Lift_State_Reset();
}

/*=============================================================================
 * 配置接口
 *============================================================================*/
// Arm0
int FX_L0_Arm0_Config_SetBrakeLock(unsigned char axis_mask)
{
    return RobotCtrl::Arm0_Config_SetBrakeLock(axis_mask);
}

int FX_L0_Arm0_Config_SetBrakeUnlock(unsigned char axis_mask)
{
    return RobotCtrl::Arm0_Config_SetBrakeUnlock(axis_mask);
}

int FX_L0_Arm0_Config_ResetEncOffset(unsigned char axis_mask)
{
    return RobotCtrl::Arm0_Config_ResetEncOffset(axis_mask);
}

int FX_L0_Arm0_Config_ClearEncError(unsigned char axis_mask)
{
    return RobotCtrl::Arm0_Config_ClearEncError(axis_mask);
}

int FX_L0_Arm0_Config_ResetExtEncOffset(unsigned char axis_mask)
{
    return RobotCtrl::Arm0_Config_ResetExtEncOffset(axis_mask);
}

int FX_L0_Arm0_Config_DisableSoftLimit(unsigned char axis_mask)
{
    return RobotCtrl::Arm0_Config_DisableSoftLimit(axis_mask);
}

int FX_L0_Arm0_Config_SetSensorOffset(int axis_id, int offset)
{
    return RobotCtrl::Arm0_Config_SetSensorOffset(axis_id, offset);
}

// Arm1
int FX_L0_Arm1_Config_SetBrakeLock(unsigned char axis_mask)
{
    return RobotCtrl::Arm1_Config_SetBrakeLock(axis_mask);
}

int FX_L0_Arm1_Config_SetBrakeUnlock(unsigned char axis_mask)
{
    return RobotCtrl::Arm1_Config_SetBrakeUnlock(axis_mask);
}

int FX_L0_Arm1_Config_ResetEncOffset(unsigned char axis_mask)
{
    return RobotCtrl::Arm1_Config_ResetEncOffset(axis_mask);
}

int FX_L0_Arm1_Config_ClearEncError(unsigned char axis_mask)
{
    return RobotCtrl::Arm1_Config_ClearEncError(axis_mask);
}

int FX_L0_Arm1_Config_ResetExtEncOffset(unsigned char axis_mask)
{
    return RobotCtrl::Arm1_Config_ResetExtEncOffset(axis_mask);
}

int FX_L0_Arm1_Config_DisableSoftLimit(unsigned char axis_mask)
{
    return RobotCtrl::Arm1_Config_DisableSoftLimit(axis_mask);
}

int FX_L0_Arm1_Config_SetSensorOffset(int axis_id, int offset)
{
    return RobotCtrl::Arm1_Config_SetSensorOffset(axis_id, offset);
}

// Head
int FX_L0_Head_Config_SetBrakeLock(unsigned char axis_mask)
{
    return RobotCtrl::Head_Config_SetBrakeLock(axis_mask);
}

int FX_L0_Head_Config_SetBrakeUnlock(unsigned char axis_mask)
{
    return RobotCtrl::Head_Config_SetBrakeUnlock(axis_mask);
}

int FX_L0_Head_Config_ResetEncOffset(unsigned char axis_mask)
{
    return RobotCtrl::Head_Config_ResetEncOffset(axis_mask);
}

int FX_L0_Head_Config_ClearEncError(unsigned char axis_mask)
{
    return RobotCtrl::Head_Config_ClearEncError(axis_mask);
}

int FX_L0_Head_Config_ResetExtEncOffset(unsigned char axis_mask)
{
    return RobotCtrl::Head_Config_ResetExtEncOffset(axis_mask);
}

int FX_L0_Head_Config_DisableSoftLimit(unsigned char axis_mask)
{
    return RobotCtrl::Head_Config_DisableSoftLimit(axis_mask);
}

// Body
int FX_L0_Body_Config_SetBrakeLock(unsigned char axis_mask)
{
    return RobotCtrl::Body_Config_SetBrakeLock(axis_mask);
}

int FX_L0_Body_Config_SetBrakeUnlock(unsigned char axis_mask)
{
    return RobotCtrl::Body_Config_SetBrakeUnlock(axis_mask);
}

int FX_L0_Body_Config_ResetEncOffset(unsigned char axis_mask)
{
    return RobotCtrl::Body_Config_ResetEncOffset(axis_mask);
}

int FX_L0_Body_Config_ClearEncError(unsigned char axis_mask)
{
    return RobotCtrl::Body_Config_ClearEncError(axis_mask);
}

int FX_L0_Body_Config_ResetExtEncOffset(unsigned char axis_mask)
{
    return RobotCtrl::Body_Config_ResetExtEncOffset(axis_mask);
}

int FX_L0_Body_Config_DisableSoftLimit(unsigned char axis_mask)
{
    return RobotCtrl::Body_Config_DisableSoftLimit(axis_mask);
}

int FX_L0_Body_Config_SetSensorOffset(int axis_id, int offset)
{
    return RobotCtrl::Body_Config_SetSensorOffset(axis_id, offset);
}

// Lift
int FX_L0_Lift_Config_ResetEncOffset(unsigned char axis_mask)
{
    return RobotCtrl::Lift_Config_ResetEncOffset(axis_mask);
}

int FX_L0_Lift_Config_DisableSoftLimit(unsigned char axis_mask)
{
    return RobotCtrl::Lift_Config_DisableSoftLimit(axis_mask);
}

/*=============================================================================
 * 运行时接口
 *============================================================================*/
// Arm0
int FX_L0_Arm0_Runtime_EmergencyStop(void)
{
    return RobotCtrl::Arm0_Runtime_EmergencyStop();
}

int FX_L0_Arm0_Runtime_SetState(int state)
{
    return RobotCtrl::Arm0_Runtime_SetState(state);
}

int FX_L0_Arm0_Runtime_SetJointPosCmd(double joint_pos[7])
{
    return RobotCtrl::Arm0_Runtime_SetJointPosCmd(joint_pos);
}

int FX_L0_Arm0_Runtime_SetJointTorCmd(double joint_tor[7])
{
    return RobotCtrl::Arm0_Runtime_SetJointTorCmd(joint_tor);
}

int FX_L0_Arm0_Runtime_SetForceCtrl(double force_ctrl[5])
{
    return RobotCtrl::Arm0_Runtime_SetForceCtrl(force_ctrl);
}

int FX_L0_Arm0_Runtime_SetTorqueCtrl(double torque_ctrl[5])
{
    return RobotCtrl::Arm0_Runtime_SetTorqueCtrl(torque_ctrl);
}

int FX_L0_Arm0_Runtime_SetVelRatio(double vel_ratio)
{
    return RobotCtrl::Arm0_Runtime_SetVelRatio(vel_ratio);
}

int FX_L0_Arm0_Runtime_SetAccRatio(double acc_ratio)
{
    return RobotCtrl::Arm0_Runtime_SetAccRatio(acc_ratio);
}

int FX_L0_Arm0_Runtime_SetJointK(double k[7])
{
    return RobotCtrl::Arm0_Runtime_SetJointK(k);
}

int FX_L0_Arm0_Runtime_SetJointD(double d[7])
{
    return RobotCtrl::Arm0_Runtime_SetJointD(d);
}

int FX_L0_Arm0_Runtime_SetCartK(double k[7])
{
    return RobotCtrl::Arm0_Runtime_SetCartK(k);
}

int FX_L0_Arm0_Runtime_SetCartD(double d[7])
{
    return RobotCtrl::Arm0_Runtime_SetCartD(d);
}

int FX_L0_Arm0_Runtime_SetToolK(double k[6])
{
    return RobotCtrl::Arm0_Runtime_SetToolK(k);
}

int FX_L0_Arm0_Runtime_SetToolD(double d[10])
{
    return RobotCtrl::Arm0_Runtime_SetToolD(d);
}

int FX_L0_Arm0_Runtime_SetImpType(int imp_type)
{
    return RobotCtrl::Arm0_Runtime_SetImpType(imp_type);
}

int FX_L0_Arm0_Runtime_SetDragType(int drag_type)
{
    return RobotCtrl::Arm0_Runtime_SetDragType(drag_type);
}

int FX_L0_Arm0_Runtime_InitTraj(int point_num)
{
    return RobotCtrl::Arm0_Runtime_InitTraj(point_num);
}

int FX_L0_Arm0_Runtime_SetTraj(int serial, int point_num, double *point_data)
{
    return RobotCtrl::Arm0_Runtime_SetTraj(serial, point_num, point_data);
}

int FX_L0_Arm0_Runtime_RunTraj(void)
{
    return RobotCtrl::Arm0_Runtime_RunTraj();
}

int FX_L0_Arm0_Runtime_StopTraj(void)
{
    return RobotCtrl::Arm0_Runtime_StopTraj();
}

// Arm1
int FX_L0_Arm1_Runtime_EmergencyStop(void)
{
    return RobotCtrl::Arm1_Runtime_EmergencyStop();
}

int FX_L0_Arm1_Runtime_SetState(int state)
{
    return RobotCtrl::Arm1_Runtime_SetState(state);
}

int FX_L0_Arm1_Runtime_SetJointPosCmd(double joint_pos[7])
{
    return RobotCtrl::Arm1_Runtime_SetJointPosCmd(joint_pos);
}

int FX_L0_Arm1_Runtime_SetJointTorCmd(double joint_tor[7])
{
    return RobotCtrl::Arm1_Runtime_SetJointTorCmd(joint_tor);
}

int FX_L0_Arm1_Runtime_SetForceCtrl(double force_ctrl[5])
{
    return RobotCtrl::Arm1_Runtime_SetForceCtrl(force_ctrl);
}

int FX_L0_Arm1_Runtime_SetTorqueCtrl(double torque_ctrl[5])
{
    return RobotCtrl::Arm1_Runtime_SetTorqueCtrl(torque_ctrl);
}

int FX_L0_Arm1_Runtime_SetVelRatio(double vel_ratio)
{
    return RobotCtrl::Arm1_Runtime_SetVelRatio(vel_ratio);
}

int FX_L0_Arm1_Runtime_SetAccRatio(double acc_ratio)
{
    return RobotCtrl::Arm1_Runtime_SetAccRatio(acc_ratio);
}

int FX_L0_Arm1_Runtime_SetJointK(double k[7])
{
    return RobotCtrl::Arm1_Runtime_SetJointK(k);
}

int FX_L0_Arm1_Runtime_SetJointD(double d[7])
{
    return RobotCtrl::Arm1_Runtime_SetJointD(d);
}

int FX_L0_Arm1_Runtime_SetCartK(double k[7])
{
    return RobotCtrl::Arm1_Runtime_SetCartK(k);
}

int FX_L0_Arm1_Runtime_SetCartD(double d[7])
{
    return RobotCtrl::Arm1_Runtime_SetCartD(d);
}

int FX_L0_Arm1_Runtime_SetToolK(double k[6])
{
    return RobotCtrl::Arm1_Runtime_SetToolK(k);
}

int FX_L0_Arm1_Runtime_SetToolD(double d[10])
{
    return RobotCtrl::Arm1_Runtime_SetToolD(d);
}

int FX_L0_Arm1_Runtime_SetImpType(int imp_type)
{
    return RobotCtrl::Arm1_Runtime_SetImpType(imp_type);
}

int FX_L0_Arm1_Runtime_SetDragType(int drag_type)
{
    return RobotCtrl::Arm1_Runtime_SetDragType(drag_type);
}

int FX_L0_Arm1_Runtime_InitTraj(int point_num)
{
    return RobotCtrl::Arm1_Runtime_InitTraj(point_num);
}

int FX_L0_Arm1_Runtime_SetTraj(int serial, int point_num, double *point_data)
{
    return RobotCtrl::Arm1_Runtime_SetTraj(serial, point_num, point_data);
}

int FX_L0_Arm1_Runtime_RunTraj(void)
{
    return RobotCtrl::Arm1_Runtime_RunTraj();
}

int FX_L0_Arm1_Runtime_StopTraj(void)
{
    return RobotCtrl::Arm1_Runtime_StopTraj();
}

// Head
int FX_L0_Head_Runtime_EmergencyStop(void)
{
    return RobotCtrl::Head_Runtime_EmergencyStop();
}

int FX_L0_Head_Runtime_SetState(int state)
{
    return RobotCtrl::Head_Runtime_SetState(state);
}

int FX_L0_Head_Runtime_SetJointPosCmd(double joint_pos[3])
{
    return RobotCtrl::Head_Runtime_SetJointPosCmd(joint_pos);
}

int FX_L0_Head_Runtime_SetVelRatio(double vel_ratio)
{
    return RobotCtrl::Head_Runtime_SetVelRatio(vel_ratio);
}

int FX_L0_Head_Runtime_SetAccRatio(double acc_ratio)
{
    return RobotCtrl::Head_Runtime_SetAccRatio(acc_ratio);
}

// Body
int FX_L0_Body_Runtime_EmergencyStop(void)
{
    return RobotCtrl::Body_Runtime_EmergencyStop();
}

int FX_L0_Body_Runtime_SetState(int state)
{
    return RobotCtrl::Body_Runtime_SetState(state);
}

int FX_L0_Body_Runtime_SetJointPosCmd(double joint_pos[6])
{
    return RobotCtrl::Body_Runtime_SetJointPosCmd(joint_pos);
}

int FX_L0_Body_Runtime_SetVelRatio(double vel_ratio)
{
    return RobotCtrl::Body_Runtime_SetVelRatio(vel_ratio);
}

int FX_L0_Body_Runtime_SetAccRatio(double acc_ratio)
{
    return RobotCtrl::Body_Runtime_SetAccRatio(acc_ratio);
}

int FX_L0_Body_Runtime_SetPDP(double p[6])
{
    return RobotCtrl::Body_Runtime_SetPDP(p);
}

int FX_L0_Body_Runtime_SetPDD(double d[6])
{
    return RobotCtrl::Body_Runtime_SetPDD(d);
}

int FX_L0_Body_Runtime_InitTraj(int point_num)
{
    return RobotCtrl::Body_Runtime_InitTraj(point_num);
}

int FX_L0_Body_Runtime_SetTraj(int serial, int point_num, double *point_data)
{
    return RobotCtrl::Body_Runtime_SetTraj(serial, point_num, point_data);
}

int FX_L0_Body_Runtime_RunTraj(void)
{
    return RobotCtrl::Body_Runtime_RunTraj();
}

int FX_L0_Body_Runtime_StopTraj(void)
{
    return RobotCtrl::Body_Runtime_StopTraj();
}
// Lift

int FX_L0_Lift_Runtime_EmergencyStop(void)
{
    return RobotCtrl::Lift_Runtime_EmergencyStop();
}

int FX_L0_Lift_Runtime_SetState(int state)
{
    return RobotCtrl::Lift_Runtime_SetState(state);
}

int FX_L0_Lift_Runtime_SetJointPosCmd(double joint_pos[2])
{
    return RobotCtrl::Lift_Runtime_SetJointPosCmd(joint_pos);
}

int FX_L0_Lift_Runtime_SetVelRatio(double vel_ratio)
{
    return RobotCtrl::Lift_Runtime_SetVelRatio(vel_ratio);
}

int FX_L0_Lift_Runtime_SetAccRatio(double acc_ratio)
{
    return RobotCtrl::Lift_Runtime_SetAccRatio(acc_ratio);
}

int FX_L0_Lift_Runtime_InitTraj(int point_num)
{
    return RobotCtrl::Lift_Runtime_InitTraj(point_num);
}

int FX_L0_Lift_Runtime_SetTraj(int serial, int point_num, double *point_data)
{
    return RobotCtrl::Lift_Runtime_SetTraj(serial, point_num, point_data);
}

int FX_L0_Lift_Runtime_RunTraj(void)
{
    return RobotCtrl::Lift_Runtime_RunTraj();
}

int FX_L0_Lift_Runtime_StopTraj(void)
{
    return RobotCtrl::Lift_Runtime_StopTraj();
}
/*=============================================================================
 * 通讯数据结构获取
 *============================================================================*/
const ROBOT_RT *FX_L0_GetRobotRT(void)
{
    RobotCtrl *ctrl = RobotCtrl::GetIns();
    if (ctrl == NULL)
    {
        return NULL;
    }
    return &ctrl->m_RobotRT;
}

const ROBOT_SG *FX_L0_GetRobotSG(void)
{
    RobotCtrl *ctrl = RobotCtrl::GetIns();
    if (ctrl == NULL)
    {
        return NULL;
    }
    return &ctrl->m_RobotSG;
}
