/**
 * @file FXErrorCode.h
 * @brief Error code definitions and function return status types for the FX system.
 *
 * This header defines standardized error identifiers used across FX SDK components.
 * It includes two primary categories:
 * - System-level and servo-related error codes (@ref FXErrorCode)
 * - Function return status values (@ref FXFuncReturn)
 *
 * These enumerations are referenced by both low-level (L0) and high-level (L1)
 * APIs to indicate operation results, fault conditions, and communication failures.
 */

#ifndef FX_ERRORTYPE_H_
#define FX_ERRORTYPE_H_

/** @defgroup fx_system_errors FX System Error Codes
 *  Error codes representing internal system faults, configuration issues,
 *  and runtime failures.
 *  @{
 */

/**
 * @brief System and servo error code definitions.
 */
typedef enum
{
    ERR_LoadIni = 1,               ///< Failed to load INI configuration file
    ERR_IniConfig = 2,             ///< Invalid or malformed INI configuration
    ERR_MasterConfig = 3,          ///< Master node configuration error
    ERR_SlaveConfig = 4,           ///< Slave node configuration error
    ERR_ActiveMaster = 5,          ///< Failed to activate master node
    ERR_RtTask = 6,                ///< Real-time task execution error
    ERR_KTask = 7,                 ///< Kernel task execution error
    ERR_Internal = 100,            ///< Internal system error
    ERR_Emcy = 101,                ///< Emergency stop condition triggered
    ERR_Servo = 102,               ///< General servo error
    ERR_PvtStreamBroken = 103,     ///< PVT data stream interruption
    ERR_RequestPositionMode = 104, ///< Failed to request position control mode
    ERR_ResponsePositionMode = 105,///< Invalid response when entering position mode
    ERR_RequestTorqueMode = 106,   ///< Failed to request torque control mode
    ERR_ResponseTorqueMode = 107,  ///< Invalid response when entering torque mode
    ERR_RequestEnableServo = 108,  ///< Failed to enable servo
    ERR_ResponseEnableServo = 109, ///< Invalid response when enabling servo
    ERR_ResponseDisableServo = 110,///< Invalid response when disabling servo
    ERR_ServoStateAbnormal = 111,  ///< Abnormal servo operational state
    ERR_SlavePdoAbnormal = 112,    ///< Abnormal slave PDO mapping or data
    ERR_SlaveStateAbnormal = 113,  ///< Abnormal slave node state
    ERR_BusLinkDown = 114          ///< Communication bus link disconnected
} FXErrorCode;

/** @} */

/** @defgroup fx_function_return_codes Function Return Status Codes
 *  Standardized return values for SDK API calls.
 *  @{
 */

/**
 * @brief Return status codes for FX SDK functions.
 *
 * This enumeration defines common return values used by most API functions
 * to indicate success, generic failures, or specific error conditions.
 */
typedef enum
{
    FUNC_RET_SUCCESS = 0,                 ///< Operation completed successfully
    FUNC_RET_OPERATION_FAILED = -1,       ///< Generic operation failure
    FUNC_RET_LINK_FAILED = -2,             ///< Communication link establishment failed
    FUNC_RET_LINK_NO_RESPONSE = -3,        ///< No response received from remote endpoint
    FUNC_RET_VERSION_INCOMPATIABLE = -4,   ///< Version mismatch detected
    FUNC_RET_INVALID_INPUT_ARG = -5,       ///< Invalid input argument provided
    FUNC_RET_INVALID_CONDITION = -6,       ///< Preconditions for operation not satisfied
    FUNC_RET_SEND_FILE_FAILED = -7,        ///< File transmission failed
    FUNC_RET_RECV_FILE_FAILED = -8,        ///< File reception failed
    FUNC_RET_INVALID_OBJ = -9,             ///< Invalid object type specified
    FUNC_RET_OPERATION_TIMEOUT = -10,      ///< Operation exceeded allowed time limit
    FUNC_RET_SET_PARAM_FAILED = -11,       ///< Failed to set parameter
    FUNC_RET_GET_PARAM_FAILED = -12,       ///< Failed to retrieve parameter
    FUNC_RET_SAVE_PARAM_FAILED = -13,      ///< Failed to save parameter to storage
    FUNC_RET_INVALID_TERMINAL = -14,       ///< Invalid terminal identifier
    FUNC_RET_INVALID_PARAM_SETTING = -15,  ///< Invalid parameter configuration
    FUNC_RET_COMM_WAIT_READY_TIMEOUT = -16,///< Timeout waiting for communication ready state
    FUNC_RET_COMM_SEND_FAILED = -17,       ///< Data transmission failed
    FUNC_RET_COMM_WAIT_REPLY_TIMEOUT = -18,///< Timeout waiting for command reply
    FUNC_RET_FORMAT_CMD_FAILED = -19,      ///< Command formatting or construction failed
    FUNC_RET_INVALID_ROBOT_TYPE = -20,     ///< Invalid robot type to support
    FUNC_RET_INVALID_HAND_TYPE = -21,      ///< Invalid hand type to support
    FUNC_RET_LINK_REJECTED = -22,          ///< Controller has already linked by other client
    FUNC_RET_INVALID_THREAD_ID = -23,      ///< Invalid thread ID for runtime APIs
    FUNC_RET_TOO_MANY_USER_DATA_ITEM = -24,///< Too many sample data item registed
    FUNC_RET_INVALID_USER_DATA_ITEM = -25, ///< Invalid sample data item 
    FUNC_RET_INVALID_USER_DATA_LEN = -26,  ///< Invalid sample data length

    FUNC_RET_KINE_NOT_INITIALIZED = -1000,         ///< Kinematics context or arm environment is not initialized
    FUNC_RET_KINE_INIT_FAILED = -1001,             ///< Kinematics environment initialization failed
    FUNC_RET_KINE_TOOL_FAILED = -1002,             ///< Tool transform update failed
    FUNC_RET_KINE_IK_UNREACHABLE = -1010,          ///< Inverse kinematics target is unreachable or singular
    FUNC_RET_KINE_IK_JOINT_LIMIT_EXCEEDED = -1011, ///< Inverse kinematics result exceeds joint limits
    FUNC_RET_KINE_PLAN_FAILED = -1020,             ///< Motion planning failed for an unspecified kinematics reason
    FUNC_RET_KINE_PLAN_JOINT_LIMIT = -1021,        ///< Planned motion exceeds joint limits
    FUNC_RET_KINE_PLAN_UNREACHABLE = -1022,        ///< Planned Cartesian target is unreachable
    FUNC_RET_KINE_TRAJ_POINT_OVERFLOW = -1023,     ///< Planned trajectory exceeds the maximum point count
    FUNC_RET_KINE_SYNC_POINT_MISMATCH = -1024,     ///< Dual-arm synchronized plan produced mismatched point counts
    FUNC_RET_KINE_DYNAMICS_IDENT_FAILED = -1030,   ///< Dynamics identification failed
    FUNC_RET_KINE_INTERNAL_ERROR = -1099,          ///< Internal kinematics or planning error    
} FXFuncReturn;

/** @} */

#endif


