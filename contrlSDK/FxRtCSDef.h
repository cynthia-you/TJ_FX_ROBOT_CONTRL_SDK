#ifndef FX_RTCS_DEF_H_
#define FX_RTCS_DEF_H_

#include "FxType.h"


#define MAX_ROWS 7
#define MAX_COLS 50


typedef enum
{
	ARM_STATE_IDLE = 0,             //////// 下伺服
	ARM_STATE_POSITION = 1,			//////// 位置跟随
	ARM_STATE_PVT = 2,				//////// PVT
	ARM_STATE_TORQ = 3,				//////// 扭矩
	ARM_STATE_RELEASE = 4,			//////// 释放

	ARM_STATE_ERROR = 100, //////// 报错了，清错
	ARM_STATE_TRANS_TO_POSITION = 101, //////// 正常，切换过程,但是如果一直是这个值就是切换失败.
	ARM_STATE_TRANS_TO_PVT = 102,//////// 正常，切换过程,但是如果一直是这个值就是切换失败.
	ARM_STATE_TRANS_TO_TORQ = 103,//////// 正常，切换过程,但是如果一直是这个值就是切换失败.
	ARM_STATE_TRANS_TO_RELEASE = 104,//////// 正常，切换过程,但是如果一直是这个值就是切换失败.
	ARM_STATE_TRANS_TO_IDLE = 109, //////// 正常，切换过程,但是如果一直是这个值就是切换失败.
}ArmState;

typedef struct
{
	FX_INT32   m_CurState;	///* 当前状态 */ ArmState
	FX_INT32   m_CmdState;	///* 指令状态 */ DCSSCmdType 0
	FX_INT32   m_ERRCode;	///* 错误码   */
}StateCtr;

typedef struct
{
	FX_INT32 	m_OutFrameSerial;   	///* 输出帧序号   0 -  1000000 取模*/
	FX_FLOAT    m_FB_Joint_Pos[7];		///* 反馈关节位置 */							0-6
	FX_FLOAT    m_FB_Joint_Vel[7];		///* 反馈关节速度 */							10-16
	FX_FLOAT    m_FB_Joint_PosE[7];		///* 反馈关节位置(外编) */						20-26
	FX_FLOAT    m_FB_Joint_Cmd[7];		///* 位置关节指令 */							30-36
	FX_FLOAT    m_FB_Joint_CToq[7];		///* 反馈关节电流 */							40-46
	FX_FLOAT    m_FB_Joint_SToq[7];		///* 反馈关节扭矩 */							50-56
	FX_FLOAT    m_FB_Joint_Them[7];		///* 反馈关节温度 */
	FX_FLOAT    m_EST_Joint_Firc[7];	///* 关节摩檫力估计值 */						60-66
	FX_FLOAT    m_EST_Joint_Firc_Dot[7];	///* 关节力扰动估计值微分 */				70-76
	FX_FLOAT    m_EST_Joint_Force[7];	///* 关节力扰动估计值 */						80-86
	FX_FLOAT    m_EST_Cart_FN[6];		///* 末端扰动估计值 */							90-95
	FX_CHAR     m_TipDI;				//末端按钮是否按下
	FX_CHAR     m_LowSpdFlag;			//机器人停止运动标志， 可用于判断是否运动到位。
	FX_CHAR     m_pad[1];
	FX_CHAR		m_TrajState;			//规划状态： 0: no traj; 1: receving; 2: recevied; >=3: running traj
}RT_OUT;

typedef struct
{
	FX_INT32 m_RtInSwitch;  	 	///* 实时输入开关 用户实时数据 进行开关设置 0 -  close rt_in ;1- open rt_in*/
	FX_INT32 m_ImpType;				///* 阻抗类型:1关节阻抗 2 迪卡尔阻抗 3力控*/
	FX_INT32 m_InFrameSerial;    	///* 输入帧序号   0 -  1000000 取模*/
	FX_INT16 m_FrameMissCnt;    	///* 丢帧计数*/
	FX_INT16 m_MaxFrameMissCnt;		///* 开 启 后 最 大 丢 帧 计 数 */

	FX_INT32 m_SysCyc;    			///* 0 -  1000000 */
	FX_INT16 m_SysCycMissCnt;		///* 实 时 性  Miss 计 数*/
	FX_INT16 m_MaxSysCycMissCnt;	///* 开 启 后 最 大 实 时 性Miss 计 数 */

	FX_FLOAT m_ToolKine[6];			///* 工 具 运 动 学 参 数 */ 1
	FX_FLOAT m_ToolDyn[10];			///* 工 具 动 力 学 参 数 */ 1

	FX_FLOAT m_Joint_CMD_Pos[7];	///* 关 节 位 置 指 令 */         7     
	FX_INT16 m_Joint_Vel_Ratio;		///* 关 节 速 度 限 制 百分比*/        2
	FX_INT16 m_Joint_Acc_Ratio;		///* 关 节 加 速 度 限 制  百分比*/    2

	FX_FLOAT m_Joint_K[7]; 			///* 关节阻抗刚度K指令*///3
	FX_FLOAT m_Joint_D[7]; 			///* 关节阻抗阻尼D指令*///3

	FX_INT32 m_DragSpType; 			///* 拖动类型*///5
	FX_FLOAT m_DragSpPara[6]; 		///* 拖动参数类型*///5
	
	FX_INT32 m_Cart_KD_Type;		///* 坐标阻抗类型*/
	FX_FLOAT m_Cart_K[6]; 			///* 坐标阻抗刚度K指令*///4
	FX_FLOAT m_Cart_D[6]; 			///* 坐标阻抗阻尼D指令*///4
	FX_FLOAT m_Cart_KN;             /// 4
	FX_FLOAT m_Cart_DN;             /// 4	

	FX_INT32  m_Force_FB_Type;		///* 力控反馈源类型*/
	FX_INT32  m_Force_Type;			///* 力控类型*///6
	FX_FLOAT  m_Force_Dir[6];		///* 力控方向6维空间方向*///6
	FX_FLOAT  m_Force_PIDUL[7];		///* 力控pid*///6
	FX_FLOAT  m_Force_AdjLmt;		///* 允许调节最大范围*///6

	FX_FLOAT  m_Force_Cmd;			///* 力控指令*///8

	FX_UCHAR m_SET_Tags[16];        ///* 设置记录*///
	FX_UCHAR m_Update_Tags[16];     ///* 更新记录 *///

	FX_UCHAR m_PvtID;
	FX_UCHAR m_PvtID_Update;
	FX_UCHAR m_Pvt_RunID;    // 0: no pvt file; 1~249: user ; 250: TOOL_CALIB.txt; 251: LEFT_ARM_CALIB.txt; 252: RIGHT_ARM_CALIB.txt 
	FX_UCHAR m_Pvt_RunState; // 0: idle; 1: loading ; 2: running; 3: error

}RT_IN;

typedef struct
{
	StateCtr m_State[2];
	RT_IN    m_In[2];
	RT_OUT	 m_Out[2];

	FX_CHAR m_ParaName[30]; // section_name.param_name
	FX_UCHAR m_ParaType; // 0: FX_INT32; 1: FX_DOUBLE; 2: FX_STRING
	FX_UCHAR m_ParaIns;  // DCSSCfgOperationType
	FX_INT32 m_ParaValueI; // FX_INT32 value
	FX_FLOAT m_ParaValueF; // FX_FLOAT value
	FX_INT16 m_ParaCmdSerial; // from PC
	FX_INT16 m_ParaRetSerial; // working: 0; finish: cmd serial; error cmd_serial + 100
}DCSS;



typedef struct
{
	FX_INT32 m_CH;
	FX_INT32 m_SUB_CH;
	FX_INT32 m_Serial;
	FX_INT32 m_Size;
	FX_UCHAR m_Data[256];// original 256
}DDSS;


typedef enum
{
    DCSS_CMD_ARM0_TRANS_STATE = 101,
    DCSS_CMD_ARM0_SET_TOOL = 102,
    DCSS_CMD_ARM0_SET_JOINT_VA = 103,
    DCSS_CMD_ARM0_SET_JOINT_KD = 104,
    DCSS_CMD_ARM0_SET_CART_KD = 105,
    DCSS_CMD_ARM0_SET_ZERO_SP = 106,
    DCSS_CMD_ARM0_SET_FORCE_PIDUL = 107,
    DCSS_CMD_ARM0_SET_JOINT_CMD = 108,
    DCSS_CMD_ARM0_SET_FORCE_CMD = 109,
    DCSS_CMD_ARM0_SET_PVT_CMD = 110,
    DCSS_CMD_ARM0_SET_IMP_TYPE = 111,
	DCSS_CMD_ARM0_INIT_TRAJ = 112,
    DCSS_CMD_ARM0_SET_TRAJ = 113,
    DCSS_CMD_ARM0_RUN_TRAJ = 114,
	DCSS_CMD_ARM0_STOP_TRAJ = 115,

    DCSS_CMD_CFG_OPERATION = 150,

    DCSS_CMD_ARM1_TRANS_STATE = 201,
    DCSS_CMD_ARM1_SET_TOOL = 202,
    DCSS_CMD_ARM1_SET_JOINT_VA = 203,
    DCSS_CMD_ARM1_SET_JOINT_KD = 204,
    DCSS_CMD_ARM1_SET_CART_KD = 205,
    DCSS_CMD_ARM1_SET_ZERO_SP = 206,
    DCSS_CMD_ARM1_SET_FORCE_PIDUL = 207,
    DCSS_CMD_ARM1_SET_JOINT_CMD = 208,
    DCSS_CMD_ARM1_SET_FORCE_CMD = 209,
    DCSS_CMD_ARM1_SET_PVT_CMD = 210,
    DCSS_CMD_ARM1_SET_IMP_TYPE = 211,
	DCSS_CMD_ARM1_INIT_TRAJ = 212,
    DCSS_CMD_ARM1_SET_TRAJ = 213,
    DCSS_CMD_ARM1_RUN_TRAJ = 214,
	DCSS_CMD_ARM1_STOP_TRAJ = 215,

}DCSSCmdType;

typedef enum
{
	DCSS_CFG_OP_SET_INT32 = 101,
	DCSS_CFG_OP_SET_DOUBLE = 102,
	DCSS_CFG_OP_GET_INT32 = 103,
	DCSS_CFG_OP_GET_DOUBLE = 104,
	DCSS_CFG_OP_SAVE = 105,
}DCSSCfgOperationType;


typedef enum
{
	SLEEP_TIME=10, //sleep time after sendCmd
	TIME_OUT=20, ///internal time_out, for api:OnSetSendWaitResponse
	ARM_IMP_JOINT=1,
	ARM_IMP_CART=2,
	ARM_IMP_FORCE=3,
}InterCMD;


typedef enum
{
	 ARM_ERR_BusPhysicAbnoraml = 1, //"总线拓扑异常"
	ARM_ERR_ServoError = 2,//"伺服故障"
	ARM_ERR_InvalidPVT = 3,//"PVT异常"
	ARM_ERR_RequestPositionMode = 4,//"请求进位置失败"
	ARM_ERR_PositionModeOK = 5,//"进位置失败"
	ARM_ERR_RequestSensorMode = 6,//"请求进扭矩失败"
	ARM_ERR_SensorModeOK = 7,//"进扭矩失败"
	ARM_ERR_RequestEnableServo = 8,//"请求上伺服失败"
	ARM_ERR_EnableServoOK = 9,//"上伺服失败"
	ARM_ERR_RequestDisableServo = 10, //"请求下伺服失败
	ARM_ERR_DisableServoOK = 11, //"下伺服失败"
	ARM_ERR_InvalidSubState = 12, //"内部错"
	ARM_ERR_Emcy = 13, //"急停"
	ARM_DYNA_FLOAT_NO_GYRO = 14,//"配置文件选择了浮动基座选项，但是UMI设置在配置文件未开"
}ArmError;

#endif


