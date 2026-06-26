#ifndef FX_UDPCOMMON_H_
#define FX_UDPCOMMON_H_

#include "FXType.h"


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
	UDP_ARM0_RT_CmdPDSerial = 16,	// settag[15]
	UDP_ARM0_RT_CmdTag = 17,

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
	UDP_ARM1_RT_CmdPDSerial = 46,	// settag[15]
	UDP_ARM1_RT_CmdTag = 47,

	UDP_HEAD_RT_CmdState = 60,
	UDP_HEAD_RT_CmdPos = 61,   // settag[0]
	UDP_HEAD_SG_VelRatio = 62, // settag[1]
	UDP_HEAD_SG_AccRatio = 63, // settag[2]
	UDP_HEAD_RT_CmdTag = 64,

	UDP_BODY_RT_CmdState = 90,
	UDP_BODY_RT_CmdPos = 91,	  // settag[0]
	UDP_BODY_RT_CmdCtrlType = 92, // settag[1]
	UDP_BODY_SG_VelRatio = 93,	  // settag[2]
	UDP_BODY_SG_AccRatio = 94,	  // settag[3]
	UDP_BODY_SG_PDK = 95,		  // settag[4]
	UDP_BODY_SG_PDD = 96,		  // settag[5]
	UDP_BODY_RT_CmdPDSerial = 97, // settag[6]
	UDP_BODY_RT_CmdTag = 98,

	UDP_LIFT_RT_CmdState = 120,
	UDP_LIFT_RT_CmdPos = 121,	// settag[0]
	UDP_LIFT_SG_VelRatio = 122, // settag[1]
	UDP_LIFT_SG_AccRatio = 123, // settag[2]
	UDP_LIFT_RT_CmdTag = 124,

	UDP_HAND0_RT_CmdAction = 130, // settag[0]
	UDP_HAND0_RT_CmdPos = 131,	  // settag[1]
	UDP_HAND0_SG_CmdP = 132,	  // settag[2]
	UDP_HAND0_SG_CmdD = 133,	  // settag[3]
	UDP_HAND0_SG_CmdMaxTor = 134, // settag[4]

	UDP_HAND1_RT_CmdAction = 160, // settag[0]
	UDP_HAND1_RT_CmdPos = 161,	  // settag[1]
	UDP_HAND1_SG_CmdP = 162,	  // settag[2]
	UDP_HAND1_SG_CmdD = 163,	  // settag[3]
	UDP_HAND1_SG_CmdMaxTor = 164, // settag[4]

	UDP_ARM0_SP_Emcy = 200, // special cmd
	UDP_ARM1_SP_Emcy = 201, // special cmd
	UDP_HEAD_SP_Emcy = 202, // special cmd
	UDP_BODY_SP_Emcy = 203, // special cmd
	UDP_LIFT_SP_Emcy = 204, // special cmd

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
	// Param
	OPINS_PARAM_SET_INT32 = 1,	// Input: m_OpValueS + m_OpValueI; Output: null
	OPINS_PARAM_SET_FLOAT = 2,	// Input: m_OpValueS + m_OpValueF; Output: null
	OPINS_PARAM_GET_INT32 = 3,	// Input: m_OpValueS; Output: m_OpValueI
	OPINS_PARAM_GET_FLOAT = 4,	// Input: m_OpValueS; Output: m_OpValueF
	OPINS_PARAM_GET_STRING = 5, // Input: m_OpValueS; Output: m_OpValueS
	OPINS_PARAM_SAVE = 6,		// Input: m_OpValueS; Output: null

	// System
	OPINS_SYSTEM_UPDATE = 10,				 // Input: null; Output: null
	OPINS_SYSTEM_REBOOT = 11,				 // Input: null; Output: null
	OPINS_SYSTEM_GET_VERSION = 12,			 // Input: null; Output: m_OpValueI
	OPINS_SYSTEM_CHECK_VERSION = 13,		 // Input: m_OpValueI, sdk version; Output: m_OpValueI, 0: failed; 1: pass
	OPINS_SYSTEM_SET_PD_CMD_CYCLE_TIME = 14, // Input: m_OpValueI, command cycle time in PD mode, ms; Output: null
	OPINS_SYSTEM_GET_PD_CMD_CYCLE_TIME = 15, // Input: null; Output: m_OpValueI, command cycle time in PD mode, ms

	// Arm0
	OPINS_ARM0_RESET = 100,					// Input: null; Output: null
	OPINS_ARM0_DISABLE_SOFTLIMIT = 101,		// Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM0_BRAKE_LOCK = 102,			// Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_ARM0_BRAKE_UNLOCK = 103,			// Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_ARM0_ENC_RESET_SINGLE_TURN = 104, // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM0_ENC_CLEAR_ERROR = 105,		// Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM0_ENC_RESET_MULTI_TURN = 106,	// Input: m_OpValueI, bit0~7 for each extenc; Output: null
	OPINS_ARM0_SENSOR0_SET_OFFSET = 107,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR1_SET_OFFSET = 108,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR2_SET_OFFSET = 109,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR3_SET_OFFSET = 110,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR4_SET_OFFSET = 111,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR5_SET_OFFSET = 112,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR6_SET_OFFSET = 113,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM0_SENSOR7_SET_OFFSET = 114,	// Input: m_OpValueI, sensor offset; Output: null

	OPINS_ARM0_SERVO_GET_ERROR_CODE = 150, // Input: m_OpValueI, servo id; Output: m_OpValueI, servo error code
	OPINS_ARM0_SERVO_GET_VERSION = 151,	   // Input: m_OpValueI, servo id; Output: m_OpValueS, servo version
	OPINS_ARM0_SENSOR_GET_VERSION = 152,   // Input: m_OpValueI, sensor id; Output: m_OpValueI, sensor version
	OPINS_ARM0_SENSOR_GET_SERIAL = 153,	   // Input: m_OpValueI, sensor id; Output: m_OpValueI, sensor serial
	OPINS_ARM0_GET_PHYSIC_STATE = 154,	   // Input: null; Output: m_OpValueI, physical state

	// Arm1
	OPINS_ARM1_RESET = 200,					// Input: null; Output: null
	OPINS_ARM1_DISABLE_SOFTLIMIT = 201,		// Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM1_BRAKE_LOCK = 202,			// Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_ARM1_BRAKE_UNLOCK = 203,			// Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_ARM1_ENC_RESET_SINGLE_TURN = 204, // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM1_ENC_CLEAR_ERROR = 205,		// Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_ARM1_ENC_RESET_MULTI_TURN = 206,	// Input: m_OpValueI, bit0~7 for each extenc; Output: null
	OPINS_ARM1_SENSOR0_SET_OFFSET = 207,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR1_SET_OFFSET = 208,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR2_SET_OFFSET = 209,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR3_SET_OFFSET = 210,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR4_SET_OFFSET = 211,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR5_SET_OFFSET = 212,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR6_SET_OFFSET = 213,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_ARM1_SENSOR7_SET_OFFSET = 214,	// Input: m_OpValueI, sensor offset; Output: null

	OPINS_ARM1_SERVO_GET_ERROR_CODE = 250, // Input: m_OpValueI, servo id; Output: m_OpValueI, servo error code
	OPINS_ARM1_SERVO_GET_VERSION = 251,	   // Input: m_OpValueI, servo id; Output: m_OpValueS, servo version
	OPINS_ARM1_SENSOR_GET_VERSION = 252,   // Input: m_OpValueI, sensor id; Output: m_OpValueI, sensor version
	OPINS_ARM1_SENSOR_GET_SERIAL = 253,	   // Input: m_OpValueI, sensor id; Output: m_OpValueI, sensor serial
	OPINS_ARM1_GET_PHYSIC_STATE = 254,	   // Input: null; Output: m_OpValueI, physical state

	// Head
	OPINS_HEAD_RESET = 300,					// Input: null; Output: null
	OPINS_HEAD_DISABLE_SOFTLIMIT = 301,		// Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_HEAD_BRAKE_LOCK = 302,			// Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_HEAD_BRAKE_UNLOCK = 303,			// Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_HEAD_ENC_RESET_SINGLE_TURN = 304, // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_HEAD_ENC_CLEAR_ERROR = 305,		// Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_HEAD_ENC_RESET_MULTI_TURN = 306,	// Input: m_OpValueI, bit0~7 for each extenc; Output: null

	OPINS_HEAD_SERVO_GET_ERROR_CODE = 350, // Input: m_OpValueI, servo id; Output: m_OpValueI, servo error code
	OPINS_HEAD_SERVO_GET_VERSION = 351,	   // Input: m_OpValueI, servo id; Output: m_OpValueS, servo version
	OPINS_HEAD_GET_PHYSIC_STATE = 354,	   // Input: null; Output: m_OpValueI, physical state

	// Body
	OPINS_BODY_RESET = 400,					// Input: null; Output: null
	OPINS_BODY_DISABLE_SOFTLIMIT = 401,		// Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_BODY_BRAKE_LOCK = 402,			// Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_BODY_BRAKE_UNLOCK = 403,			// Input: m_OpValueI, bit0~7 for each axis; Output: null
	OPINS_BODY_ENC_RESET_SINGLE_TURN = 404, // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_BODY_ENC_CLEAR_ERROR = 405,		// Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_BODY_ENC_RESET_MULTI_TURN = 406,	// Input: m_OpValueI, bit0~7 for each extenc; Output: null
	OPINS_BODY_SENSOR0_SET_OFFSET = 407,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR1_SET_OFFSET = 408,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR2_SET_OFFSET = 409,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR3_SET_OFFSET = 410,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR4_SET_OFFSET = 411,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR5_SET_OFFSET = 412,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR6_SET_OFFSET = 413,	// Input: m_OpValueI, sensor offset; Output: null
	OPINS_BODY_SENSOR7_SET_OFFSET = 414,	// Input: m_OpValueI, sensor offset; Output: null

	OPINS_BODY_SERVO_GET_ERROR_CODE = 450, // Input: m_OpValueI, servo id; Output: m_OpValueI, servo error code
	OPINS_BODY_SERVO_GET_VERSION = 451,	   // Input: m_OpValueI, servo id; Output: m_OpValueS, servo version
	OPINS_BODY_SENSOR_GET_VERSION = 452,   // Input: m_OpValueI, sensor id; Output: m_OpValueI, sensor version
	OPINS_BODY_SENSOR_GET_SERIAL = 453,	   // Input: m_OpValueI, sensor id; Output: m_OpValueI, sensor serial
	OPINS_BODY_GET_PHYSIC_STATE = 454,	   // Input: null; Output: m_OpValueI, physical state

	// Lift
	OPINS_LIFT_RESET = 500,				// Input: null; Output: null
	OPINS_LIFT_DISABLE_SOFTLIMIT = 501, // Input: m_OpValueI, bit0~7 for each enc; Output: null
	OPINS_LIFT_ENC_RESET_OFFSET = 504,	// Input: m_OpValueI, bit0 used; Output: null
	OPINS_LIFT_ENC_CLEAR_ERROR = 505,	// Input: m_OpValueI, bit0 used; Output: null

	OPINS_LIFT_SERVO_GET_ERROR_CODE = 550, // Input: m_OpValueI, servo id; Output: m_OpValueI, servo error code
	OPINS_LIFT_GET_PHYSIC_STATE = 554,	   // Input: null; Output: m_OpValueI, physical state

} OpInsType;

#endif
