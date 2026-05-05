#ifndef FX_UDPCOMMON_H_
#define FX_UDPCOMMON_H_

#include "FxType.h"

typedef enum
{
	UDP_ARM0_RT_CmdState = 0,
	UDP_ARM0_RT_CmdJointPos = 1,	// settag[0]
	UDP_ARM0_RT_CmdJointTor = 2,	// settag[1]
	UDP_ARM0_RT_ForceDir = 3,		// settag[2]
	UDP_ARM0_RT_TorqueDir = 4,		// settag[3]
	UDP_ARM0_SG_VelRatio = 5,		// settag[4]
	UDP_ARM0_SG_AccRatio = 6,		// settag[5]
	UDP_ARM0_SG_JointK = 7,			// settag[6]
	UDP_ARM0_SG_JointD = 8,			// settag[7]
	UDP_ARM0_SG_CartK = 9,			// settag[8]
	UDP_ARM0_SG_CartD = 10,			// settag[9]
	UDP_ARM0_SG_ToolK = 11,			// settag[10]
	UDP_ARM0_SG_ToolD = 12,			// settag[11]
	UDP_ARM0_SG_ImpType = 13,		// settag[12]
	UDP_ARM0_RT_DragType = 14,		// settag[13]
	UDP_ARM0_RT_ForceCtrlType = 15, // settag[14]

	UDP_ARM1_RT_CmdState = 30,
	UDP_ARM1_RT_CmdJointPos = 31,	// settag[0]
	UDP_ARM1_RT_CmdJointTor = 32,	// settag[1]
	UDP_ARM1_RT_ForceDir = 33,		// settag[2]
	UDP_ARM1_RT_TorqueDir = 34,		// settag[3]
	UDP_ARM1_SG_VelRatio = 35,		// settag[4]
	UDP_ARM1_SG_AccRatio = 36,		// settag[5]
	UDP_ARM1_SG_JointK = 37,		// settag[6]
	UDP_ARM1_SG_JointD = 38,		// settag[7]
	UDP_ARM1_SG_CartK = 39,			// settag[8]
	UDP_ARM1_SG_CartD = 40,			// settag[9]
	UDP_ARM1_SG_ToolK = 41,			// settag[10]
	UDP_ARM1_SG_ToolD = 42,			// settag[11]
	UDP_ARM1_SG_ImpType = 43,		// settag[12]
	UDP_ARM1_RT_DragType = 44,		// settag[13]
	UDP_ARM1_RT_ForceCtrlType = 45, // settag[14]

	UDP_HEAD_RT_CmdState = 60,
	UDP_HEAD_RT_CmdPos = 61,   // settag[0]
	UDP_HEAD_SG_VelRatio = 62, // settag[1]
	UDP_HEAD_SG_AccRatio = 63, // settag[2]

	UDP_BODY_RT_CmdState = 90,
	UDP_BODY_RT_CmdPos = 91,	  // settag[0]
	UDP_BODY_RT_CmdCtrlType = 92, // settag[1]
	UDP_BODY_SG_VelRatio = 93,	  // settag[2]
	UDP_BODY_SG_AccRatio = 94,	  // settag[3]
	UDP_BODY_SG_PDK = 95,		  // settag[4]
	UDP_BODY_SG_PDD = 96,		  // settag[5]

	UDP_LIFT_RT_CmdState = 120,
	UDP_LIFT_RT_CmdPos = 121,	// settag[0]
	UDP_LIFT_SG_VelRatio = 122, // settag[1]
	UDP_LIFT_SG_AccRatio = 123, // settag[2]

        UDP_ARM0_SP_Emcy = 200,         //special cmd 
        UDP_ARM1_SP_Emcy = 201,         //special cmd 
        UDP_HEAD_SP_Emcy = 202,         //special cmd 
        UDP_BODY_SP_Emcy = 203,         //special cmd 
        UDP_LIFT_SP_Emcy = 204,         //special cmd 

	UDP_ARM0_SP_InitTraj = 210, // special cmd
	UDP_ARM0_SP_SetTraj = 211,	// special cmd
	UDP_ARM0_SP_RunTraj = 212,	// special cmd
	UDP_ARM0_SP_StopTraj = 213, // special cmd
	UDP_ARM1_SP_InitTraj = 214, // special cmd
	UDP_ARM1_SP_SetTraj = 215,	// special cmd
	UDP_ARM1_SP_RunTraj = 216,	// special cmd
	UDP_ARM1_SP_StopTraj = 217, // special cmd
	UDP_BODY_SP_InitTraj = 218, // special cmd
	UDP_BODY_SP_SetTraj = 219,	// special cmd
	UDP_BODY_SP_RunTraj = 220,	// special cmd
	UDP_BODY_SP_StopTraj = 221, // special cmd
	UDP_LIFT_SP_InitTraj = 222, // special cmd
	UDP_LIFT_SP_SetTraj = 223,	// special cmd
	UDP_LIFT_SP_RunTraj = 224,	// special cmd
	UDP_LIFT_SP_StopTraj = 225, // special cmd

	UDP_SYS_WAIT_SERIAL = 251,
	UDP_OPERATION = 254,
} UdpInsType;

typedef enum
{
	ARM_STATE_IDLE = 0,
	ARM_STATE_POSITION = 1,
	ARM_STATE_TORQUE = 2,
	ARM_STATE_RELEASE = 3,

	ARM_STATE_TRANS_TAG = 50,
	ARM_STATE_ERROR = 100,
	ARM_STATE_TRANS_TO_POSITION = 101,
	ARM_STATE_TRANS_TO_TORQUE = 102,
	ARM_STATE_TRANS_TO_RELEASE = 103,

	ARM_STATE_TRANS_TO_IDLE = 109,
} ArmState;

typedef enum
{
	HEAD_STATE_IDLE = 0,

	HEAD_STATE_POSITION = 1,

	HEAD_STATE_TRANS_TAG = 50,
	HEAD_STATE_ERROR = 100,
	HEAD_STATE_TRANS_TO_POSITION = 101,

	HEAD_STATE_TRANS_TO_IDLE = 109,
} HeadState;

typedef enum
{
	BODY_STATE_IDLE = 0,

	BODY_STATE_POSITION = 1,
	BODY_STATE_TORQUE = 2,
	BODY_STATE_RELEASE = 3,

	BODY_STATE_TRANS_TAG = 50,
	BODY_STATE_ERROR = 100,
	BODY_STATE_TRANS_TO_POSITION = 101,
	BODY_STATE_TRANS_TO_TORQUE = 102,
	BODY_STATE_TRANS_TO_RELEASE = 103,

	BODY_STATE_TRANS_TO_IDLE = 109,
} BodyState;

typedef enum
{
	LIFT_STATE_IDLE = 0,

	LIFT_STATE_POSITION = 1,

	LIFT_STATE_TRANS_TAG = 50,
	LIFT_STATE_ERROR = 100,
	LIFT_STATE_TRANS_TO_POSITION = 101,

	LIFT_STATE_TRANS_TO_IDLE = 109,
} LiftState;

typedef enum
{
	IMP_TYPE_NULL = 0,
	IMP_TYPE_JOINT = 1,
	IMP_TYPE_CART = 2,
	IMP_TYPE_FORCE = 3,
} ImpType;

typedef enum
{
	DRAG_TYPE_NULL = 0,
	DRAG_TYPE_JOINT = 1,
	DRAG_TYPE_CART_X = 2,
	DRAG_TYPE_CART_Y = 3,
	DRAG_TYPE_CART_Z = 4,
	DRAG_TYPE_CART_R = 5,
} DragType;

typedef struct
{
	FX_UINT16 m_CurState; ///* 当前状态 *////
	FX_UINT16 m_CmdState; ///* 指令状态 *////
	FX_UINT32 m_ERRCode;  ///* 错误码   *////
} StateCtr;

typedef struct
{
	FX_FLOAT m_ARM_CMD_Joint_Tor[7];   ///* 关节扭矩指令 *////
	FX_FLOAT m_ARM_CMD_Joint_Pos[7];   ///* 关节位置指令 *////
	FX_INT16 m_ARM_CMD_Ctrl_DragType;	   ///* 拖动类型*////
	FX_INT16 m_ARM_CMD_Ctrl_ForceType; ///* 力控类型*////
	FX_FLOAT m_ARM_CMD_Ctrl_ForceDir[5];   ///* 力控方向IJKFR*////
	FX_FLOAT m_ARM_CMD_Ctrl_TorqueDir[5];  ///* 扭矩方向RIJKNR*////
} ARM_IN;

typedef struct
{
	FX_FLOAT m_ARM_FBK_Joint_Pos[7];		///* 反馈关节位置 *////
	FX_FLOAT m_ARM_FBK_Joint_Vel[7];		///* 反馈关节速度 *////
	FX_FLOAT m_ARM_FBK_Joint_Cmd[7];		///* 位置关节指令 *////
	FX_FLOAT m_ARM_FBK_Joint_SensorTor[7];	///* 反馈关节扭矩 *////
	FX_FLOAT m_ARM_FBK_Joint_ExternalTorEst[7]; ///* 关节力扰动估计值 *////
	FX_FLOAT m_ARM_FBK_Base_FNEst[6];	///* 基座力和扭矩 *////
	FX_FLOAT m_ARM_FBK_Base_Gyro[6];		///* ACCX,ACCY,ACCZ,OMGX,OMGY,OMGZ*////
	FX_FLOAT m_ARM_FBK_Flange_FTSensor[6];		///* 末端六维力传感器*////
} ARM_OUT;

typedef struct
{
	FX_FLOAT m_ARM_FBK_Joint_Tor[7]; ///* 反馈关节扭矩 *////
	FX_FLOAT m_ARM_FBK_Joint_ExtPos[7]; ///* 反馈关节位置(外编) *////
	FX_CHAR m_ARM_FBK_Flange_DI;
	FX_CHAR m_ARM_FBK_LowSpdFlag;
	FX_CHAR m_ARM_FBK_TrajState; /// 0: no traj; 1: receving; 2: recevied; >=3: running traj
	FX_CHAR m_pad[1];
} ARM_GET;
typedef struct
{
	FX_INT32 m_ARM_Ctrl_ImpType;		  ///* 阻抗类型 *////
	FX_FLOAT m_ARM_Ctrl_VelRatio;	  ///* 关节速度限制百分比*////
	FX_FLOAT m_ARM_Ctrl_AccRatio;	  ///* 关节加速度限制百分比*////
	FX_FLOAT m_ARM_Ctrl_JointK[7];	  ///* 关节阻抗刚度K指令*////
	FX_FLOAT m_ARM_Ctrl_JointD[7];	  ///* 关节阻抗阻尼D指令*////
	FX_FLOAT m_ARM_Ctrl_CartK[7];	  ///* 坐标阻抗刚度K指令*////
	FX_FLOAT m_ARM_Ctrl_CartD[7];	  ///* 坐标阻抗阻尼D指令*////
	FX_FLOAT m_ARM_Ctrl_ToolKine[6];  ///* 工具运动学参数*////
	FX_FLOAT m_ARM_Ctrl_ToolDyna[10]; ///* 工具动力学参数*////
	FX_BOOL m_ARM_SET_SetTags[16];
	FX_BOOL m_ARM_SET_UpDateTag[16];
} ARM_SET;

typedef struct
{
	FX_FLOAT m_HEAD_CMD_Joint_Pos[3];
} HEAD_IN;
typedef struct
{
	FX_FLOAT m_HEAD_FBK_Joint_Pos[3];
} HEAD_OUT;

typedef struct
{
	FX_FLOAT m_HEAD_FBK_Joint_Tor[3]; ///* 电流 *////
	FX_FLOAT m_HEAD_FBK_Joint_ExtPos[3]; ///* 反馈关节位置(外编) *////
} HEAD_GET;

typedef struct
{
	FX_FLOAT m_HEAD_Ctrl_VelRatio;
	FX_FLOAT m_HEAD_Ctrl_AccRatio;
	FX_BOOL m_HEAD_SET_SetTags[4];
	FX_BOOL m_HEAD_SET_UpDateTag[4];
} HEAD_SET;

typedef struct
{
	FX_FLOAT m_LIFT_CMD_Joint_Pos[2];
} LIFT_IN;
typedef struct
{
	FX_FLOAT m_LIFT_FBK_Joint_Pos[2];
} LIFT_OUT;

typedef struct
{
	FX_FLOAT m_LIFT_FBK_Joint_Tor[2];  ///* 电流 *////
	FX_CHAR m_LIFT_FBK_TrajState; /// 0: no traj; 1: receving; 2: recevied; >=3: running traj
	FX_CHAR m_pad[3];
} LIFT_GET;

typedef struct
{
	FX_FLOAT m_LIFT_Ctrl_VelRatio;
	FX_FLOAT m_LIFT_Ctrl_AccRatio;
	FX_BOOL m_LIFT_SET_SetTags[4];
	FX_BOOL m_LIFT_SET_UpDateTag[4];
} LIFT_SET;

typedef struct
{
	FX_INT32 m_BODY_CMD_Ctrl_Type;
	FX_FLOAT m_BODY_CMD_Joint_Pos[6];
} BODY_IN;
typedef struct
{
	FX_FLOAT m_BODY_FBK_Joint_Pos[6];
	FX_FLOAT m_BODY_FBK_Joint_Vel[6];
	FX_FLOAT m_BODY_FBK_Joint_SensorTor[6];
	FX_FLOAT m_BODY_FBK_Base_Gyro[6];
} BODY_OUT;

typedef struct
{
	FX_FLOAT m_BODY_FBK_Joint_Tor[6];  ///* 电流 *////
	FX_FLOAT m_BODY_FBK_Joint_ExtPos[6];  ///* 反馈关节位置(外编) *////
	FX_CHAR m_BODY_FBK_TrajState; /// 0: no traj; 1: receving; 2: recevied; >=3: running traj
	FX_CHAR m_pad[3];
} BODY_GET;

typedef struct
{
	FX_FLOAT m_BODY_Ctrl_VelRatio;
	FX_FLOAT m_BODY_Ctrl_AccRatio;
	FX_FLOAT m_BODY_Ctrl_PDK[6];
	FX_FLOAT m_BODY_Ctrl_PDD[6];
	FX_BOOL m_BODY_SET_SetTags[6];
	FX_BOOL m_BODY_SET_UpDateTag[6];
} BODY_SET;

typedef struct
{
	FX_INT16 m_HAND_CMD_Joint_Pos[24];
	FX_INT16 m_HAND_CMD_Joint_Tor[24];
} HAND_IN;

typedef struct
{
	FX_INT16 m_HAND_FBK_Joint_Pos[24];
	FX_INT16 m_HAND_FBK_Joint_Tor[24];
	FX_INT16 m_HAND_FBK_F[24];
} HAND_OUT;

typedef struct
{
	FX_INT16 m_HAND_Ctrl_KP[24];
	FX_INT16 m_HAND_Ctrl_KD[24];
	FX_INT16 m_HAND_Ctrl_Vel[24];
} HAND_SET;

typedef struct
{
	StateCtr m_ARM_State; ///* 手臂状态 *////
	ARM_IN m_ARM_IN;	  ///* 输入数据 *////
	ARM_OUT m_ARM_OUT;	  ///* 输出数据 *////
} ARM_RT;

typedef struct
{
	StateCtr m_HEAD_State; ///* 头状态 *////
	HEAD_IN m_HEAD_IN;
	HEAD_OUT m_HEAD_OUT;
} HEAD_RT;

typedef struct
{
	StateCtr m_BODY_State; ///* 身体状态 *////
	BODY_IN m_BODY_IN;
	BODY_OUT m_BODY_OUT;
} BODY_RT;

typedef struct
{
	StateCtr m_HAND_State; ///* 灵巧手状态 *////
	HAND_IN m_HAND_IN;
	HAND_OUT m_HAND_OUT;
} HAND_RT;

typedef struct
{
	StateCtr m_LIFT_State; ///* 头状态 *////
	LIFT_IN m_LIFT_IN;
	LIFT_OUT m_LIFT_OUT;
} LIFT_RT;

typedef struct
{
	ARM_SET m_ARM_SET; ///* 输入数据 *////
	ARM_GET m_ARM_GET; ///* 输出数据 *////
} ARM_SG;

typedef struct
{
	HEAD_SET m_HEAD_SET;
	HEAD_GET m_HEAD_GET;
} HEAD_SG;

typedef struct
{
	BODY_SET m_BODY_SET;
	BODY_GET m_BODY_GET;
} BODY_SG;

typedef struct
{
	LIFT_SET m_LIFT_SET;
	LIFT_GET m_LIFT_GET;
} LIFT_SG;

typedef struct
{
	HAND_SET m_HAND_SET;
} HAND_SG;

typedef enum
{
	// Param
	OPINS_PARAM_SET_INT32 = 1, // Input: m_OpValueS + m_OpValueI; Output: null
	OPINS_PARAM_SET_FLOAT = 2, // Input: m_OpValueS + m_OpValueF; Output: null
	OPINS_PARAM_GET_INT32 = 3, // Input: m_OpValueS; Output: m_OpValueI
	OPINS_PARAM_GET_FLOAT = 4, // Input: m_OpValueS; Output: m_OpValueF
	OPINS_PARAM_SAVE = 5,	   // Input: m_OpValueS; Output: null
	OPINS_PARAM_UPDATE = 6,	   // Input: m_OpValueS, ini file name; Output: null

	// System
	OPINS_SYSTEM_UPDATE = 10,	   // Input: null; Output: null
	OPINS_SYSTEM_REBOOT = 11,	   // Input: null; Output: null
	OPINS_SYSTEM_GET_VERSION = 12, // Input: null; Output: m_OpValueI

	// Arm0
	OPINS_ARM0_RESET = 100,				  // Input: null; Output: null
	OPINS_ARM0_DISABLE_SOFTLIMIT = 101,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM0_BRAKE_LOCK = 102,		  // Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_ARM0_BRAKE_UNLOCK = 103,		  // Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_ARM0_ENC_RESET_OFFSET = 104,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM0_ENC_CLEAR_ERROR = 105,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM0_EXTENC_RESET_OFFSET = 106, // Input: m_OpValueI, bit0~7 for each extenc; Output: null
	OPINS_ARM0_SENSOR0_SET_OFFSET = 107,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR1_SET_OFFSET = 108,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR2_SET_OFFSET = 109,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR3_SET_OFFSET = 110,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR4_SET_OFFSET = 111,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR5_SET_OFFSET = 112,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR6_SET_OFFSET = 113,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR7_SET_OFFSET = 114,  // Input: m_OpValueI, sensor offset; Output: null

	OPINS_ARM0_SERVO_GET_ERROR_CODE = 150, // Input: m_OpValueI, servo id; Output: m_OpValueI, servo error code

	// Arm1
	OPINS_ARM1_RESET = 200,				  // Input: null; Output: null
	OPINS_ARM1_DISABLE_SOFTLIMIT = 201,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM1_BRAKE_LOCK = 202,		  // Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_ARM1_BRAKE_UNLOCK = 203,		  // Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_ARM1_ENC_RESET_OFFSET = 204,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM1_ENC_CLEAR_ERROR = 205,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM1_EXTENC_RESET_OFFSET = 206, // Input: m_OpValueI, bit0~7 for each extenc; Output: null
	OPINS_ARM1_SENSOR0_SET_OFFSET = 207,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR1_SET_OFFSET = 208,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR2_SET_OFFSET = 209,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR3_SET_OFFSET = 210,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR4_SET_OFFSET = 211,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR5_SET_OFFSET = 212,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR6_SET_OFFSET = 213,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR7_SET_OFFSET = 214,  // Input: m_OpValueI, sensor offset; Output: null

	OPINS_ARM1_SERVO_GET_ERROR_CODE = 250, // Input: m_OpValueI, servo id; Output: m_OpValueI, servo error code

	// Head
	OPINS_HEAD_RESET = 300,				  // Input: null; Output: null
	OPINS_HEAD_DISABLE_SOFTLIMIT = 301,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_HEAD_BRAKE_LOCK = 302,		  // Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_HEAD_BRAKE_UNLOCK = 303,		  // Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_HEAD_ENC_RESET_OFFSET = 304,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_HEAD_ENC_CLEAR_ERROR = 305,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_HEAD_EXTENC_RESET_OFFSET = 306, // Input: m_OpValueI, bit0~7 for each extenc; Output: null

	OPINS_HEAD_SERVO_GET_ERROR_CODE = 350, // Input: m_OpValueI, servo id; Output: m_OpValueI, servo error code

	// Body
	OPINS_BODY_RESET = 400,				  // Input: null; Output: null
	OPINS_BODY_DISABLE_SOFTLIMIT = 401,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_BODY_BRAKE_LOCK = 402,		  // Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_BODY_BRAKE_UNLOCK = 403,		  // Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_BODY_ENC_RESET_OFFSET = 404,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_BODY_ENC_CLEAR_ERROR = 405,	  // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_BODY_EXTENC_RESET_OFFSET = 406, // Input: m_OpValueI, bit0~7 for each extenc; Output: null
	OPINS_BODY_SENSOR0_SET_OFFSET = 407,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR1_SET_OFFSET = 408,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR2_SET_OFFSET = 409,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR3_SET_OFFSET = 410,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR4_SET_OFFSET = 411,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR5_SET_OFFSET = 412,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR6_SET_OFFSET = 413,  // Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR7_SET_OFFSET = 414,  // Input: m_OpValueI, sensor offset; Output: null

	OPINS_BODY_SERVO_GET_ERROR_CODE = 450, // Input: m_OpValueI, servo id; Output: m_OpValueI, servo error code

	// Lift
	OPINS_LIFT_RESET = 500,				// Input: null; Output: null
	OPINS_LIFT_DISABLE_SOFTLIMIT = 501, // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_LIFT_ENC_RESET_OFFSET = 504,	// Input: m_OpValueI, bit0 used; Output: null
	OPINS_LIFT_ENC_CLEAR_ERROR = 505,	// Input: m_OpValueI, bit0 used; Output: null

	OPINS_LIFT_SERVO_GET_ERROR_CODE = 550, // Input: m_OpValueI, servo id; Output: m_OpValueI, servo error code

} OpInsType;

typedef struct
{
	FX_INT16 m_OpIns;		// OpInsType
	FX_CHAR m_OpValueS[30]; // FX_CHAR[] value
	FX_INT32 m_OpValueI;	// FX_INT32 value
	FX_FLOAT m_OpValueF;	// FX_FLOAT value
	FX_INT16 m_OpCmdSerial; // from PC
	FX_INT16 m_OpRetSerial; // working: 0; finish: cmd serial; error cmd_serial + 100
} OP_SET;

typedef struct
{
	FX_INT32 m_RT_FrameSerial; ///* 输出帧序号   0 -  1000000 取模*////
	HEAD_RT m_HEAD;
	ARM_RT m_ARMS[2];
	HAND_RT m_HANDS[2];
	BODY_RT m_BODY;
	LIFT_RT m_LIFT;
	FX_UCHAR wait_serial;
	FX_UCHAR pad[3];
} ROBOT_RT; // 1000HZ

typedef struct
{
	FX_INT32 m_RT_FrameSerial; ///* 输出帧序号   0 -  1000000 取模*////
	HEAD_SG m_HEAD;
	ARM_SG m_ARMS[2];
	HAND_SG m_HANDS[2];
	BODY_SG m_BODY;
	LIFT_SG m_LIFT;
	OP_SET m_OP_SET;
} ROBOT_SG; // 500HZ

typedef struct
{
	FX_INT32 m_CH;
	FX_INT32 m_SUB_CH;
	FX_INT32 m_Serial;
	FX_INT32 m_Size;
	FX_UCHAR m_Data[256];
} DDSS;

#endif
