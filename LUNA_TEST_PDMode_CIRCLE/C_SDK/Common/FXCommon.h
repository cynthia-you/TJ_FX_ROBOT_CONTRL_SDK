/**
 * @file FXCommon.h
 * @brief Common definitions, enumerations, data structures, and utilities
 *        for the FX robot system SDK.
 *
 * This header serves as the foundational include for both low-level (L0)
 * and high-level (L1) APIs. It defines core types, versioning macros,
 * object identifiers, state machines, feedback structures, and control
 * parameters shared across the FX software stack.
 */

#ifndef FX_FXCOMMON_H_
#define FX_FXCOMMON_H_

#include "FXType.h"

/** @defgroup fx_sdk_version FX SDK Version
 *  SDK version definitions and helper macros.
 *  @{
 */

#define FX_SDK_MAJOR_VERSION 4UL /**< Major version number */
#define FX_SDK_MINOR_VERSION 4UL /**< Minor version number */
#define FX_SDK_PATCH_VERSION 2UL /**< Patch version number */

/**
 * @brief Construct a packed 32-bit SDK version value.
 *
 * The resulting version encoding follows the layout:
 * - Bits [15:8] : Major version
 * - Bits [7:0]  : Minor version
 * - Bits [23:16]: Patch version
 *
 * @param major Major version
 * @param minor Minor version
 * @param sub   Patch version
 * @return Packed 32-bit version value
 */
#define MAKE_VERSION(major, minor, sub) \
    (((major) << 16) | ((minor) << 8) | (sub))

/** @} */

/** @defgroup fx_log_levels Logging Levels
 *  Bitmask definitions for the FX logging system.
 *  @{
 */

#define FX_LOG_NULL_FLAG (0)       /**< No log output */
#define FX_LOG_DEBG_FLAG (1 << 0)  /**< Debug log messages */
#define FX_LOG_INFO_FLAG (1 << 1)  /**< Informational log messages */
#define FX_LOG_WARN_FLAG (1 << 2)  /**< Warning log messages */
#define FX_LOG_ERROR_FLAG (1 << 3) /**< Error log messages */
#define FX_LOG_ALL_FLAG (FX_LOG_DEBG_FLAG | FX_LOG_INFO_FLAG | \
                         FX_LOG_WARN_FLAG | FX_LOG_ERROR_FLAG)

/** @} */

/** @defgroup fx_object_types Object Types
 *  Identifiers for controllable robot entities.
 *  @{
 */

/**
 * @brief Object types in the FX robot system.
 */
typedef enum
{
    FX_OBJ_ARM0 = 0, ///< Left arm
    FX_OBJ_ARM1 = 1, ///< Right arm
    FX_OBJ_HEAD = 2, ///< Head module
    FX_OBJ_BODY = 3, ///< Body module
    FX_OBJ_LIFT = 4, ///< Lift module
    FX_OBJ_NUM = 5   ///< Number of object types
} FXObjType;

#define FX_OBJ_ARM0_FLAG (1 << 0) ///< Bit flag for ARM0
#define FX_OBJ_ARM1_FLAG (1 << 1) ///< Bit flag for ARM1
#define FX_OBJ_HEAD_FLAG (1 << 2) ///< Bit flag for HEAD
#define FX_OBJ_BODY_FLAG (1 << 3) ///< Bit flag for BODY
#define FX_OBJ_LIFT_FLAG (1 << 4) ///< Bit flag for LIFT
#define FX_OBJ_ALL_FLAG (FX_OBJ_ARM0_FLAG | FX_OBJ_ARM1_FLAG | \
                         FX_OBJ_HEAD_FLAG | FX_OBJ_BODY_FLAG | \
                         FX_OBJ_LIFT_FLAG)

#define FX_DEFAULT_THREAD_ID 0 /**< Default internal used thread ID */

/** @} */

/**
 * @brief Robot type of the linked robot.
 */
typedef enum
{
    FX_ROBOT_NULL = 0,          ///< Not a valid robot type
    FX_ROBOT_MARVIN_PRO_M3 = 1, ///< MarvinProM3
    FX_ROBOT_MARVIN_PRO_M6 = 2, ///< MarvinProM6
    FX_ROBOT_GENTO_SKYE = 3,    ///< GentoSkye
    FX_ROBOT_GENTO_LUNA = 4,    ///< GentoLuna
} FXRobotType;

/**
 * @brief Physical usage state of an object.
 */
typedef enum
{
    FX_OBJ_NOT_USED = 0, ///< Object is not used
    FX_OBJ_VIRTUAL = 1,  ///< Object exists as a virtual entity
    FX_OBJ_REAL = 2      ///< Object exists as a real (physical) entity
} FXObjPhysicalState;

/**
 * @brief Parameter data types supported by the FX system.
 */
typedef enum
{
    FX_PARAM_TYPE_INT,  ///< Integer parameter
    FX_PARAM_TYPE_FLOAT ///< Floating-point parameter
} FXParamType;

/**
 * @brief Sampling data types supported by the FX system.
 */
typedef enum
{
    FX_char  = 0,
    FX_short = 1,
    FX_int = 2,
    FX_long = 3,
    FX_long_long = 5,  
    FX_unsigned_char = 6,
    FX_unsigned_short = 7,
    FX_unsigned_int = 8,
    FX_unsigned_long = 9,
    FX_unsigned_long_long = 10,
    FX_float = 11,
    FX_double = 12,    
}FXUserDataType;

/**
 * @brief Terminal type identifiers.
 */
typedef enum
{
    FX_TERMINAL_ARM0 = 0, ///< Terminal for ARM0
    FX_TERMINAL_ARM1 = 1  ///< Terminal for ARM1
} FXTerminalType;

/**
 * @brief Communication channel types.
 */
typedef enum
{
    FX_CHN_CANFD = 1, ///< CAN FD channel
    FX_CHN_485A = 2,  ///< RS485 channel A
    FX_CHN_485B = 3   ///< RS485 channel B
} FXChnType;

/** @defgroup fx_state_machines State Machines
 *  Motion and system state enumerations.
 *  @{
 */

/**
 * @brief Arm motion state machine.
 */
typedef enum
{
    ARM_STATE_IDLE = 0,                ///< Idle state
    ARM_STATE_POSITION = 1,            ///< Position control mode
    ARM_STATE_TORQUE = 2,              ///< Torque control mode
    ARM_STATE_RELEASE = 3,             ///< Collaborative release mode
    ARM_STATE_TRANS_TAG = 50,          ///< Transition tag
    ARM_STATE_ERROR = 100,             ///< Error state
    ARM_STATE_TRANS_TO_POSITION = 101, ///< Transition to position mode
    ARM_STATE_TRANS_TO_TORQUE = 102,   ///< Transition to torque mode
    ARM_STATE_TRANS_TO_RELEASE = 103,  ///< Transition to release mode
    ARM_STATE_TRANS_TO_IDLE = 109      ///< Transition to idle
} ArmState;

/**
 * @brief Head motion state machine.
 */
typedef enum
{
    HEAD_STATE_IDLE = 0,
    HEAD_STATE_POSITION = 1,
    HEAD_STATE_TRANS_TAG = 50,
    HEAD_STATE_ERROR = 100,
    HEAD_STATE_TRANS_TO_POSITION = 101,
    HEAD_STATE_TRANS_TO_IDLE = 109
} HeadState;

/**
 * @brief Body motion state machine.
 */
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
    BODY_STATE_TRANS_TO_IDLE = 109
} BodyState;

/**
 * @brief Lift motion state machine.
 */
typedef enum
{
    LIFT_STATE_IDLE = 0,
    LIFT_STATE_POSITION = 1,
    LIFT_STATE_TRANS_TAG = 50,
    LIFT_STATE_ERROR = 100,
    LIFT_STATE_TRANS_TO_POSITION = 101,
    LIFT_STATE_TRANS_TO_IDLE = 109
} LiftState;

/**
 * @brief Global FX system states.
 */
typedef enum
{
    FX_STATE_IDLE = 0,           ///< Idle
    FX_STATE_POSITION = 1,       ///< Position control
    FX_STATE_IMP_JOINT = 2,      ///< Joint impedance
    FX_STATE_IMP_CART = 3,       ///< Cartesian impedance
    FX_STATE_IMP_FORCE = 4,      ///< Force-based impedance
    FX_STATE_DRAG_JOINT = 5,     ///< Joint drag teaching
    FX_STATE_DRAG_CART_X = 6,    ///< Cartesian X drag
    FX_STATE_DRAG_CART_Y = 7,    ///< Cartesian Y drag
    FX_STATE_DRAG_CART_Z = 8,    ///< Cartesian Z drag
    FX_STATE_DRAG_CART_R = 9,    ///< Rotation drag
    FX_STATE_RELEASE = 10,       ///< Release mode
    FX_STATE_PD = 11,            ///< PD control mode
    FX_STATE_ERROR = 100,        ///< Error state
    FX_STATE_TRANSFERRING = 101, ///< State transferring
    FX_STATE_UNKNOWN = 200       ///< Unknown state
} FXStateType;

/**
 * @brief Hand type identifier.
 *
 * Used to distinguish between left and right hand modules.
 */
typedef enum
{
    FX_HAND_LEFT = 0,  ///< Left hand module.
    FX_HAND_RIGHT = 1, ///< Right hand module.
} FXHandType;

/**
 * @brief Runtime action commands for the hand module.
 *
 * These actions are used to change the operating state of the hand,
 * such as enabling, disabling, or resetting the device.
 */
typedef enum
{
    FX_HAND_ACTION_DISABLE = 0, ///< Disable the hand module.
    FX_HAND_ACTION_ENABLE = 1,  ///< Enable the hand module.
    FX_HAND_ACTION_RESET = 2,   ///< Reset the hand module to its initial state.
} FXHandAction;

/**
 * @brief Current state of the hand module.
 *
 * The state is reported by the hand module to indicate whether it is
 * disabled, enabled and operational, or in an error condition.
 */
typedef enum
{
    FX_HAND_STATE_DISABLED = 0, ///< Hand module is disabled.
    FX_HAND_STATE_ENABLED = 1,  ///< Hand module is enabled and operational.
    FX_HAND_STATE_ERROR = 100,  ///< Hand module is in an error state.
} FXHandState;

/** @} */

/** @defgroup fx_force_torque_definitions Force and Torque Definitions
 *  Indices and constants for force and torque control.
 *  @{
 */

/**
 * @brief Force definition indices.
 */
typedef enum
{
    FX_FORCE_DIR_X = 0,    ///< X direction force
    FX_FORCE_DIR_Y = 1,    ///< Y direction force
    FX_FORCE_DIR_Z = 2,    ///< Z direction force
    FX_FORCE_VALUE = 3,    ///< Force magnitude
    FX_FORCE_DISTANCE = 4, ///< Distance constraint
    FX_FORCE_DEF_NUM = 5
} FXForceDef;

/**
 * @brief Torque definition indices.
 */
typedef enum
{
    FX_TORQUE_DIR_A = 0, ///< First torque direction component
    FX_TORQUE_DIR_B = 1, ///< Second torque direction component
    FX_TORQUE_DIR_C = 2, ///< Third torque direction component
    FX_TORQUE_VALUE = 3, ///< Torque magnitude value
    FX_TORQUE_ANGLE = 4, ///< Torque application angle
    FX_TORQUE_DEF_NUM = 5
} FXTorqueDef;

/** @} */

/**
 * @brief Impedance control types.
 */
typedef enum
{
    FX_IMP_TYPE_NULL = 0,  ///< No impedance control
    FX_IMP_TYPE_JOINT = 1, ///< Joint-space impedance
    FX_IMP_TYPE_CART = 2,  ///< Cartesian-space impedance
    FX_IMP_TYPE_FORCE = 3, ///< Force-based impedance
    FX_IMP_TYPE_PD = 4     ///< PD control
} FXImpType;

/**
 * @brief Drag teaching types.
 */
typedef enum
{
    FX_DRAG_TYPE_NULL = 0,   ///< Drag teaching disabled
    FX_DRAG_TYPE_JOINT = 1,  ///< Joint-space drag
    FX_DRAG_TYPE_CART_X = 2, ///< Cartesian X-axis drag
    FX_DRAG_TYPE_CART_Y = 3, ///< Cartesian Y-axis drag
    FX_DRAG_TYPE_CART_Z = 4, ///< Cartesian Z-axis drag
    FX_DRAG_TYPE_CART_R = 5  ///< Rotational drag
} DragType;

/**
 * @brief Generic state control structure.
 */
typedef struct
{
    FX_UINT16 m_CurState; ///< Current state machine state
    FX_UINT16 m_CmdState; ///< Commanded target state
    FX_UINT32 m_ERRCode;  ///< Error code if any
} StateCtr;

/* ---------------------------------------------------------
 * Arm Structures
 * --------------------------------------------------------- */

/**
 * @brief Arm input command structure.
 */
typedef struct
{
    FX_FLOAT m_ARM_CMD_Joint_Tor[7];      ///< Joint torque command
    FX_FLOAT m_ARM_CMD_Joint_Pos[7];      ///< Joint position command
    FX_INT16 m_ARM_CMD_Ctrl_DragType;     ///< Drag control type
    FX_INT16 m_ARM_CMD_Ctrl_ForceType;    ///< Force control type
    FX_FLOAT m_ARM_CMD_Ctrl_ForceDir[5];  ///< Force direction vector
    FX_FLOAT m_ARM_CMD_Ctrl_TorqueDir[5]; ///< Torque direction vector
    FX_INT32 m_ARM_CMD_Tag;               ///< User defined tag（data tag）
    FX_INT16 m_ARM_CMD_PD_Serial;         ///< command serial in PD mode
    FX_INT8 pad[2];
} ARM_IN;

/**
 * @brief Arm feedback output structure.
 */
typedef struct
{
    FX_FLOAT m_ARM_FBK_Joint_Pos[7];            ///< Joint position feedback
    FX_FLOAT m_ARM_FBK_Joint_Vel[7];            ///< Joint velocity feedback
    FX_FLOAT m_ARM_FBK_Joint_Cmd[7];            ///< Last commanded position
    FX_FLOAT m_ARM_FBK_Joint_SensorTor[7];      ///< Torque sensor feedback
    FX_FLOAT m_ARM_FBK_Joint_ExternalTorEst[7]; ///< External torque estimate
    FX_FLOAT m_ARM_FBK_Base_FNEst[6];           ///< Base force/torque estimate
    FX_FLOAT m_ARM_FBK_Base_Gyro[6];            ///< Base IMU data
    FX_FLOAT m_ARM_FBK_Flange_FTSensor[6];      ///< Flange F/T sensor
    FX_INT32 m_ARM_FBK_Tag;                     ///< Feedback of user defined tag
} ARM_OUT;

/**
 * @brief Arm extended feedback structure.
 */
typedef struct
{
    FX_FLOAT m_ARM_FBK_Joint_Tor[7];    ///< Joint torque feedback
    FX_FLOAT m_ARM_FBK_Joint_ExtPos[7]; ///< External encoder position
    FX_INT8 m_ARM_FBK_Flange_DI;        ///< Flange DI signal
    FX_INT8 m_ARM_FBK_LowSpdFlag;       ///< Low speed indicator
    FX_INT8 m_ARM_FBK_TrajState;        ///< Trajectory execution state
    FX_INT8 m_ARM_FBK_PD_CmdQuality;    ///< Real-time performance metrics of command transmission in PD mode[0,100]
} ARM_GET;

/**
 * @brief Arm configuration and control parameters.
 */
typedef struct
{
    FX_INT32 m_ARM_Ctrl_ImpType;      ///< Impedance control type
    FX_FLOAT m_ARM_Ctrl_VelRatio;     ///< Velocity scaling ratio
    FX_FLOAT m_ARM_Ctrl_AccRatio;     ///< Acceleration scaling ratio
    FX_FLOAT m_ARM_Ctrl_JointK[7];    ///< Joint stiffness
    FX_FLOAT m_ARM_Ctrl_JointD[7];    ///< Joint damping
    FX_FLOAT m_ARM_Ctrl_CartK[7];     ///< Cartesian stiffness
    FX_FLOAT m_ARM_Ctrl_CartD[7];     ///< Cartesian damping
    FX_FLOAT m_ARM_Ctrl_ToolKine[6];  ///< Tool kinematics
    FX_FLOAT m_ARM_Ctrl_ToolDyna[10]; ///< Tool dynamics
    FX_BOOL m_ARM_SET_SetTag[16];     ///< Set flags
    FX_BOOL m_ARM_SET_UpdateTag[16];  ///< Update flags
} ARM_SET;

/* ---------------------------------------------------------
 * Head Structures
 * --------------------------------------------------------- */

typedef struct
{
    FX_FLOAT m_HEAD_CMD_Joint_Pos[3];
    FX_INT32 m_HEAD_CMD_Tag;
} HEAD_IN;
typedef struct
{
    FX_FLOAT m_HEAD_FBK_Joint_Pos[3];
    FX_INT32 m_HEAD_FBK_Tag;
} HEAD_OUT;
typedef struct
{
    FX_FLOAT m_HEAD_FBK_Joint_Tor[3];
    FX_FLOAT m_HEAD_FBK_Joint_ExtPos[3];
} HEAD_GET;
typedef struct
{
    FX_FLOAT m_HEAD_Ctrl_VelRatio;
    FX_FLOAT m_HEAD_Ctrl_AccRatio;
    FX_BOOL m_HEAD_SET_SetTag[4];
    FX_BOOL m_HEAD_SET_UpdateTag[4];
} HEAD_SET;

/* ---------------------------------------------------------
 * Lift Structures
 * --------------------------------------------------------- */

typedef struct
{
    FX_FLOAT m_LIFT_CMD_Joint_Pos[2];
    FX_INT32 m_LIFT_CMD_Tag;
} LIFT_IN;
typedef struct
{
    FX_FLOAT m_LIFT_FBK_Joint_Pos[2];
    FX_INT32 m_LIFT_FBK_Tag;
} LIFT_OUT;
typedef struct
{
    FX_FLOAT m_LIFT_FBK_Joint_Tor[2];
    FX_INT8 m_LIFT_FBK_TrajState;
    FX_INT8 m_pad[3];
} LIFT_GET;
typedef struct
{
    FX_FLOAT m_LIFT_Ctrl_VelRatio;
    FX_FLOAT m_LIFT_Ctrl_AccRatio;
    FX_BOOL m_LIFT_SET_SetTag[4];
    FX_BOOL m_LIFT_SET_UpdateTag[4];
} LIFT_SET;

/* ---------------------------------------------------------
 * Body Structures
 * --------------------------------------------------------- */

typedef struct
{
    FX_INT32 m_BODY_CMD_Ctrl_Type;
    FX_FLOAT m_BODY_CMD_Joint_Pos[6];
    FX_INT32 m_BODY_CMD_Tag;
    FX_INT16 m_BODY_CMD_PD_Serial; ///< command serial in PD mode
    FX_INT8 pad[2];
} BODY_IN;

typedef struct
{
    FX_FLOAT m_BODY_FBK_Joint_Pos[6];
    FX_FLOAT m_BODY_FBK_Joint_Vel[6];
    FX_FLOAT m_BODY_FBK_Joint_SensorTor[6];
    FX_FLOAT m_BODY_FBK_Base_Gyro[6];
    FX_INT32 m_BODY_FBK_Tag;
} BODY_OUT;

typedef struct
{
    FX_FLOAT m_BODY_FBK_Joint_Tor[6];
    FX_FLOAT m_BODY_FBK_Joint_ExtPos[6];
    FX_INT8 m_BODY_FBK_TrajState;
    FX_INT8 m_BODY_FBK_PD_CmdQuality; ///< Real-time performance metrics of command transmission in PD mode
    FX_INT8 m_pad[2];
} BODY_GET;

typedef struct
{
    FX_FLOAT m_BODY_Ctrl_VelRatio;
    FX_FLOAT m_BODY_Ctrl_AccRatio;
    FX_FLOAT m_BODY_Ctrl_PDK[6];
    FX_FLOAT m_BODY_Ctrl_PDD[6];
    FX_BOOL m_BODY_SET_SetTag[6];
    FX_BOOL m_BODY_SET_UpdateTag[6];
} BODY_SET;

/* ---------------------------------------------------------
 * Hand Structures
 * --------------------------------------------------------- */

typedef struct
{
    FX_INT8 m_HAND_CMD_action;
    FX_INT8 pad[3];
    FX_INT8 m_HAND_CMD_Joint_Pos[24];
    FX_INT8 m_HAND_CMD_Joint_Tor[24];
} HAND_IN;

typedef struct
{
    FX_INT8 m_HAND_FBK_State;
    FX_INT8 pad[3];
    FX_INT16 m_HAND_FBK_Joint_ErrorCode[24];
    FX_INT8 m_HAND_FBK_Joint_Pos[24];
    FX_INT8 m_HAND_FBK_Joint_Vel[24];
    FX_INT8 m_HAND_FBK_Joint_Cmd[24];
} HAND_OUT;

typedef struct
{
    FX_INT8 m_HAND_Ctrl_KP[24];
    FX_INT8 m_HAND_Ctrl_KD[24];
    FX_INT8 m_HAND_Ctrl_MaxTor[24];
    FX_BOOL m_HAND_SET_SetTag[8];
    FX_BOOL m_HAND_SET_UpdateTag[8];
} HAND_SET;

typedef struct
{
    FX_INT8 m_HAND_FBK_Joint_Tor[24];
    FX_INT8 m_HAND_FBK_Joint_Temp[24];
} HAND_GET;

/* ---------------------------------------------------------
 * Operation and Real-Time Structures
 * --------------------------------------------------------- */

/**
 * @brief Operation instruction set structure.
 */
typedef struct
{
    FX_INT16 m_OpIns;       ///< Operation instruction type
    FX_CHAR m_OpValueS[30]; ///< String-type parameter
    FX_INT32 m_OpValueI;    ///< Integer parameter
    FX_FLOAT m_OpValueF;    ///< Float parameter
    FX_INT16 m_OpCmdSerial; ///< Command serial number
    FX_INT16 m_OpRetSerial; ///< Return serial or error code
} OP_SET;

/**
 * @brief Real-time data structure for a single arm.
 */
typedef struct
{
    StateCtr m_ARM_State;
    ARM_IN m_ARM_IN;
    ARM_OUT m_ARM_OUT;
} ARM_RT;

typedef struct
{
    StateCtr m_HEAD_State;
    HEAD_IN m_HEAD_IN;
    HEAD_OUT m_HEAD_OUT;
} HEAD_RT;
typedef struct
{
    StateCtr m_BODY_State;
    BODY_IN m_BODY_IN;
    BODY_OUT m_BODY_OUT;
} BODY_RT;
typedef struct
{
    StateCtr m_LIFT_State;
    LIFT_IN m_LIFT_IN;
    LIFT_OUT m_LIFT_OUT;
} LIFT_RT;
typedef struct
{
    HAND_IN m_HAND_IN;
    HAND_OUT m_HAND_OUT;
} HAND_RT;

/**
 * @brief Slow-group configuration and feedback structure for an arm.
 */
typedef struct
{
    ARM_SET m_ARM_SET;
    ARM_GET m_ARM_GET;
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
    HAND_GET m_HAND_GET;
} HAND_SG;

/**
 * @brief Real-time robot data structure (1000 Hz).
 */
typedef struct
{
    FX_INT32 m_RT_FrameSerial;
    HEAD_RT m_HEAD;
    ARM_RT m_ARMS[2];
    HAND_RT m_HANDS[2];
    BODY_RT m_BODY;
    LIFT_RT m_LIFT;
    FX_UCHAR wait_serial;
    FX_UCHAR pad[3];
} ROBOT_RT;

/**
 * @brief Slow-group robot data structure (500 Hz).
 */
typedef struct
{
    FX_INT32 m_SG_FrameSerial;
    HEAD_SG m_HEAD;
    ARM_SG m_ARMS[2];
    HAND_SG m_HANDS[2];
    BODY_SG m_BODY;
    LIFT_SG m_LIFT;
    OP_SET m_OP_SET;
} ROBOT_SG;

/**
 * @brief Terminal communication packet structure.
 */
typedef struct
{
    FX_INT32 m_CH;
    FX_INT32 m_SUB_CH;
    FX_INT32 m_Serial;
    FX_INT32 m_Size;
    FX_UCHAR m_Data[256];
} DDSS;

#endif

