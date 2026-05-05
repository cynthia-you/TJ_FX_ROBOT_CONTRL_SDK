#ifndef FX_ROBOTCTRL_H_
#define FX_ROBOTCTRL_H_
#include "UdpCommon.h"
#include "FXULT.h"

class RobotCtrl
{
public:
	static RobotCtrl *GetIns();
	virtual ~RobotCtrl();

	/*//////////////////////////////////////////////////////////////////////////////////////
	连接处理
	//////////////////////////////////////////////////////////////////////////////////////*/
	//(1)	创建UDP端口和启动周期数据接收/发送服务
	static FX_BOOL Link(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4);
	//(2)	开启连接：发送数据到控制器控制器开始周期性返回数据（返回值为收到控制器数据所经历的毫秒数，可以用来判断是否正确启动通信）
	static FX_INT32 TestLink();

	/*//////////////////////////////////////////////////////////////////////////////////////
	日志开关接口
	//////////////////////////////////////////////////////////////////////////////////////*/
	static FX_VOID OnLocalLogOn();
	static FX_VOID OnLocalLogOff();

	/*//////////////////////////////////////////////////////////////////////////////////////
	组包协议
		控制器会周期性向SDK发送状态数据，但是SDK向控制器发送数据是根据用户需求在需要的时候进行发送，
		为了达成同一个操作目的，可能需要发送多个数据。为了提升数据通信效率设计了组包通信机制：
	//////////////////////////////////////////////////////////////////////////////////////*/
	//(1)	清除当前组包所有内容
	static FX_BOOL ClearSend(FX_UINT32 timeout);
	//(2)	发送当前组包所有内容
	static FX_BOOL SetSend();
	//(3)	发送当前组包所有内容，并等待本次发送数据返回，time_out为超时毫秒数，返回为经历的毫秒数，如果为负是发生错误。
	static FX_INT32 WaitSend(FX_UINT32 timeout);

	/*//////////////////////////////////////////////////////////////////////////////////////
	系统操作
	//////////////////////////////////////////////////////////////////////////////////////*/
	// （1）获取系统版本
	static FX_INT32 System_GetVersion();
	// （2）系统重启
	static FX_BOOL System_Reboot();
	// static FX_BOOL System_Update();

	/*//////////////////////////////////////////////////////////////////////////////////////
	通道数据接口:对双臂末端板的通信接口进行透传处理的接口
		数据通道是针对双臂末端引出的CANFD和两路485接口做的数据透传功能，
		SDK周期数据接收服务接收到透传数据会放入一个循环缓冲区，用户调用接口可以从循环缓冲区中获取数据。
		注意：若用户不调用获取数据，循环缓冲区会发送阻塞，新到的数据会被抛弃。
	//////////////////////////////////////////////////////////////////////////////////////*/
	//(1)	清除缓冲区（丢弃过期数据）
	static FX_BOOL Arm0_Terminal_ClearData();
	static FX_BOOL Arm1_Terminal_ClearData();
	//(2)	获取缓冲区数据，返回值为获取数据长度，data_ptr返回数据，ret_ch为数据源（1-CANFD，2-第一路486，3-第二路485）。
	static FX_INT32 Arm0_Terminal_GetData(FX_INT32 *channel_type_ptr, FX_UCHAR data_ptr[64]);
	static FX_INT32 Arm1_Terminal_GetData(FX_INT32 *channel_type_ptr, FX_UCHAR data_ptr[64]);
	//(3)	发送透传数据set_ch为数据源（1-CANFD，2-第一路486，3-第二路485）。
	static FX_BOOL Arm0_Terminal_SetData(FX_INT32 channel_type, FX_UCHAR *data_ptr, FX_INT32 data_len);
	static FX_BOOL Arm1_Terminal_SetData(FX_INT32 channel_type, FX_UCHAR *data_ptr, FX_INT32 data_len);

	/*//////////////////////////////////////////////////////////////////////////////////////
	配置参数接口
		依据参数名获取或者设置参数,参数名命名方式参考*.ini文件
	//////////////////////////////////////////////////////////////////////////////////////*/
	static FX_BOOL Para_GetInt(FX_CHAR name[30], FX_INT32 *ret_value);
	static FX_BOOL Para_GetFloat(FX_CHAR name[30], FX_FLOAT *ret_value);
	static FX_BOOL Para_SetInt(FX_CHAR name[30], FX_INT32 target_value);
	static FX_BOOL Para_SetFloat(FX_CHAR name[30], FX_FLOAT target_value);
	static FX_BOOL Para_Save();
	// static FX_BOOL Para_Update();

	/*//////////////////////////////////////////////////////////////////////////////////////
	///伺服状态信息获取，仅支持单条指令下发
	//////////////////////////////////////////////////////////////////////////////////////*/
	static FX_BOOL Arm0_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code);
	static FX_BOOL Arm1_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code);
	static FX_BOOL Head_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code);
	static FX_BOOL Body_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code);
	static FX_BOOL Lift_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code);

	/*/////////////////////////////////////////////////////////
	///配置接口，支持多条指令同时下发
	/////////////////////////////////////////////////////////*/
	// （1）ARM0 抱闸 松闸 重置内部编码器偏移 清除内部编码器错误 重置外部编码器偏移 外部编码器清错
	static FX_BOOL Arm0_Config_SetBrakeLock(FX_UINT8 axis_mask);
	static FX_BOOL Arm0_Config_SetBrakeUnlock(FX_UINT8 axis_mask);
	static FX_BOOL Arm0_Config_ResetEncOffset(FX_UINT8 axis_mask);
	static FX_BOOL Arm0_Config_ClearEncError(FX_UINT8 axis_mask);
	static FX_BOOL Arm0_Config_ResetExtEncOffset(FX_UINT8 axis_mask);
	static FX_BOOL Arm0_Config_DisableSoftLimit(FX_UINT8 axis_mask);
	static FX_BOOL Arm0_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset);
	// （2）ARM1 抱闸 松闸 重置内部编码器偏移 清除内部编码器错误 重置外部编码器偏移 外部编码器清错
	static FX_BOOL Arm1_Config_SetBrakeLock(FX_UINT8 axis_mask);
	static FX_BOOL Arm1_Config_SetBrakeUnlock(FX_UINT8 axis_mask);
	static FX_BOOL Arm1_Config_ResetEncOffset(FX_UINT8 axis_mask);
	static FX_BOOL Arm1_Config_ClearEncError(FX_UINT8 axis_mask);
	static FX_BOOL Arm1_Config_ResetExtEncOffset(FX_UINT8 axis_mask);
	static FX_BOOL Arm1_Config_DisableSoftLimit(FX_UINT8 axis_mask);
	static FX_BOOL Arm1_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset);
	// （3）HEAD 抱闸 松闸 重置内部编码器偏移 清除内部编码器错误 重置外部编码器偏移
	static FX_BOOL Head_Config_SetBrakeLock(FX_UINT8 axis_mask);
	static FX_BOOL Head_Config_SetBrakeUnlock(FX_UINT8 axis_mask);
	static FX_BOOL Head_Config_ResetEncOffset(FX_UINT8 axis_mask);
	static FX_BOOL Head_Config_ClearEncError(FX_UINT8 axis_mask);
	static FX_BOOL Head_Config_ResetExtEncOffset(FX_UINT8 axis_mask);
	static FX_BOOL Head_Config_DisableSoftLimit(FX_UINT8 axis_mask);
	// （4）BODY 抱闸 松闸 重置内部编码器偏移 清除内部编码器错误 重置外部编码器偏移 外部编码器清错
	static FX_BOOL Body_Config_SetBrakeLock(FX_UINT8 axis_mask);
	static FX_BOOL Body_Config_SetBrakeUnlock(FX_UINT8 axis_mask);
	static FX_BOOL Body_Config_ResetEncOffset(FX_UINT8 axis_mask);
	static FX_BOOL Body_Config_ClearEncError(FX_UINT8 axis_mask);
	static FX_BOOL Body_Config_ResetExtEncOffset(FX_UINT8 axis_mask);
	static FX_BOOL Body_Config_DisableSoftLimit(FX_UINT8 axis_mask);
	static FX_BOOL Body_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset);
	// （5）LIFT 重置内部编码器偏移
	static FX_BOOL Lift_Config_ResetEncOffset(FX_UINT8 axis_mask);
	static FX_BOOL Lift_Config_DisableSoftLimit(FX_UINT8 axis_mask);

	// 各部件的清错和急停
	static FX_BOOL Arm0_State_Reset();
	static FX_BOOL Arm1_State_Reset();
	static FX_BOOL Head_State_Reset();
	static FX_BOOL Body_State_Reset();
	static FX_BOOL Lift_State_Reset();

	/*/////////////////////////////////////////////////////////
	///运行时设置接口,支持多条指令同时下发
	/////////////////////////////////////////////////////////*/
	/*(1) ARM0 设置目标关节位置  设置目标扭矩 设置速度 设置加速度
		 设置关节刚度参数  设置关节阻尼参数 设置笛卡尔刚度参数 设置笛卡尔阻尼参数
		 设置工具刚度参数  设置工具阻尼参数 设置扭矩阻抗类型 设置拖动类型
		 初始化关节空间规划初始化  设置轨迹 轨迹运行  轨迹运行中断*/
	static FX_BOOL Arm0_Runtime_EmergencyStop();
	static FX_BOOL Arm0_Runtime_SetState(FX_INT32 state);
	static FX_BOOL Arm0_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[7]);
	static FX_BOOL Arm0_Runtime_SetJointTorCmd(FX_DOUBLE joint_tor[7]);
	static FX_BOOL Arm0_Runtime_SetForceCtrl(FX_DOUBLE force_ctrl[5]);
	static FX_BOOL Arm0_Runtime_SetTorqueCtrl(FX_DOUBLE torque_ctrl[5]);
	static FX_BOOL Arm0_Runtime_SetVelRatio(FX_DOUBLE vel_ratio);
	static FX_BOOL Arm0_Runtime_SetAccRatio(FX_DOUBLE acc_ratio);
	static FX_BOOL Arm0_Runtime_SetJointK(FX_DOUBLE k[7]);
	static FX_BOOL Arm0_Runtime_SetJointD(FX_DOUBLE d[7]);
	static FX_BOOL Arm0_Runtime_SetCartK(FX_DOUBLE k[7]);
	static FX_BOOL Arm0_Runtime_SetCartD(FX_DOUBLE d[7]);
	static FX_BOOL Arm0_Runtime_SetToolK(FX_DOUBLE k[6]);
	static FX_BOOL Arm0_Runtime_SetToolD(FX_DOUBLE d[10]);
	static FX_BOOL Arm0_Runtime_SetImpType(FX_INT32 imp_type);
	static FX_BOOL Arm0_Runtime_SetDragType(FX_INT16 drag_type);
	static FX_BOOL Arm0_Runtime_InitTraj(FX_INT32 point_num);
	static FX_BOOL Arm0_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data);
	static FX_BOOL Arm0_Runtime_RunTraj();
	static FX_BOOL Arm0_Runtime_StopTraj();
	/*(2) ARM1 设置目标关节位置  设置目标扭矩 设置速度 设置加速度
		 设置关节刚度参数  设置关节阻尼参数 设置笛卡尔刚度参数 设置笛卡尔阻尼参数
		 设置工具刚度参数  设置工具阻尼参数 设置扭矩阻抗类型 设置拖动类型
		 初始化关节空间规划初始化  设置轨迹 轨迹运行  轨迹运行中断*/
	static FX_BOOL Arm1_Runtime_EmergencyStop();
	static FX_BOOL Arm1_Runtime_SetState(FX_INT32 state);
	static FX_BOOL Arm1_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[7]);
	static FX_BOOL Arm1_Runtime_SetJointTorCmd(FX_DOUBLE joint_tor[7]);
	static FX_BOOL Arm1_Runtime_SetForceCtrl(FX_DOUBLE force_ctrl[5]);
	static FX_BOOL Arm1_Runtime_SetTorqueCtrl(FX_DOUBLE torque_ctrl[5]);
	static FX_BOOL Arm1_Runtime_SetVelRatio(FX_DOUBLE vel_ratio);
	static FX_BOOL Arm1_Runtime_SetAccRatio(FX_DOUBLE acc_ratio);
	static FX_BOOL Arm1_Runtime_SetJointK(FX_DOUBLE k[7]);
	static FX_BOOL Arm1_Runtime_SetJointD(FX_DOUBLE d[7]);
	static FX_BOOL Arm1_Runtime_SetCartK(FX_DOUBLE k[7]);
	static FX_BOOL Arm1_Runtime_SetCartD(FX_DOUBLE d[7]);
	static FX_BOOL Arm1_Runtime_SetToolK(FX_DOUBLE k[6]);
	static FX_BOOL Arm1_Runtime_SetToolD(FX_DOUBLE d[10]);
	static FX_BOOL Arm1_Runtime_SetImpType(FX_INT32 imp_type);
	static FX_BOOL Arm1_Runtime_SetDragType(FX_INT16 drag_type);
	static FX_BOOL Arm1_Runtime_InitTraj(FX_INT32 point_num);
	static FX_BOOL Arm1_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data);
	static FX_BOOL Arm1_Runtime_RunTraj();
	static FX_BOOL Arm1_Runtime_StopTraj();
	/*(3) HEAD 设置目标关节位置  设置速度 设置加速度*/
	static FX_BOOL Head_Runtime_EmergencyStop();
	static FX_BOOL Head_Runtime_SetState(FX_INT32 state);
	static FX_BOOL Head_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[3]);
	static FX_BOOL Head_Runtime_SetVelRatio(FX_DOUBLE vel_ratio);
	static FX_BOOL Head_Runtime_SetAccRatio(FX_DOUBLE acc_ratio);
	/*(4) BODY 设置目标关节位置  设置速度 设置加速度 设置PD模式下的P参数 设置PD模式下的D参数*/
	static FX_BOOL Body_Runtime_EmergencyStop();
	static FX_BOOL Body_Runtime_SetState(FX_INT32 state);
	static FX_BOOL Body_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[6]);
	static FX_BOOL Body_Runtime_SetVelRatio(FX_DOUBLE vel_ratio);
	static FX_BOOL Body_Runtime_SetAccRatio(FX_DOUBLE acc_ratio);
	static FX_BOOL Body_Runtime_SetPDP(FX_DOUBLE p[6]);
	static FX_BOOL Body_Runtime_SetPDD(FX_DOUBLE d[6]);
	static FX_BOOL Body_Runtime_InitTraj(FX_INT32 point_num);
	static FX_BOOL Body_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data);
	static FX_BOOL Body_Runtime_RunTraj();
	static FX_BOOL Body_Runtime_StopTraj();
	/*(5) LIFT 设置状态 设置目标关节位置  设置速度 设置加速度*/
	static FX_BOOL Lift_Runtime_EmergencyStop();
	static FX_BOOL Lift_Runtime_SetState(FX_INT32 state);
	static FX_BOOL Lift_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[2]);
	static FX_BOOL Lift_Runtime_SetVelRatio(FX_DOUBLE vel_ratio);
	static FX_BOOL Lift_Runtime_SetAccRatio(FX_DOUBLE acc_ratio);
	static FX_BOOL Lift_Runtime_InitTraj(FX_INT32 point_num);
	static FX_BOOL Lift_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data);
	static FX_BOOL Lift_Runtime_RunTraj();
	static FX_BOOL Lift_Runtime_StopTraj();

	/*/////////////////////////////////////////////////////////
	///通讯数据结构
	/////////////////////////////////////////////////////////*/
	// （1）实时数据结构体 1k HZ
	ROBOT_RT m_RobotRT;
	// （2）普通数据结构体
	ROBOT_SG m_RobotSG;
	/*/////////////////////////////////////////////////////////
	///内部使用接口，用户不要使用
	/////////////////////////////////////////////////////////*/
	FX_BOOL IsLinked();
	FX_VOID DoCnt();
	FX_VOID DoRecv();
	FX_VOID DoSend();
	FX_BOOL WaitOpReturn(FX_INT32 serial, FX_INT32 timeout);

private:
	FX_BOOL m_LinkTag;
	FX_BOOL m_RtSendLock;
	FX_BOOL m_LocalLogTag;

	FX_INT32L m_RobotRTUpdateTag;
	FX_INT32L m_RobotSGUpdateTag;
	FX_INT32L m_RobotRTRecvTag;
	FX_INT32L m_RobotSGRecvTag;
	FX_INT32L m_ParaSerial;

	FX_UCHAR m_send_response_local_tag;
	FX_UCHAR m_send_response_recv_tag;
	FX_INT32 m_send_response_timeout_cnt;
	FX_INT32 m_last_response_timeout_cnt;
	FX_INT32 m_respones_time_cnt;
	FX_INT32 m_respones_time_tag;

	CMarvNetAgent m_RT_NA;
	CMarvNetAgent m_SG_NA;
	CMarvNetAgent m_Flange_NA0;
	CMarvNetAgent m_Flange_NA1;
	CACB m_ACB1;
	CACB m_ACB2;
	FX_CHAR m_SendBuf1[600];
	FX_CHAR m_SendBuf2[600];
	DDSS *pDDSS1;
	DDSS *pDDSS2;

#ifdef CMPL_WIN
	MMRESULT m_TimeEventID;
#endif
#ifdef CMPL_LIN
	timer_t robot_timer;
#endif

	RobotCtrl();
	static FX_BOOL SetIns(FX_INT32 ins);
	static FX_BOOL SetState(FX_INT32 ins, FX_INT32 cmd_state);
	static FX_BOOL SetFLoat(FX_INT32 ins, FX_INT32 num, FX_DOUBLE *pdata);
	static FX_BOOL SetInt(FX_INT32 ins, FX_INT32 num, FX_INT32 *pdata);
	static FX_BOOL SetShortInt(FX_INT32 ins, FX_INT32 num, FX_INT16 *pdata);
	static FX_BOOL SetRawData(FX_INT32 ins, FX_INT32 num, FX_UCHAR *pdata);
};
#endif