/**
 * @file L1Robot.h
 * @brief FX L1 Robot SDK API definitions.
 *
 * This header defines the Layer 1 (L1) application programming interface for
 * interacting with FX robot systems. L1 APIs provide high-level abstractions
 * over lower-level communication, state management, motion control, and
 * configuration services.
 *
 * The interface covers system-level operations such as linking to a robot
 * controller, managing logging, querying version information, updating system
 * firmware, and handling file transfers. It also includes modules for
 * real-time feedback, state machine transitions, parameter management,
 * terminal communication, hardware configuration, and runtime motion control.
 *
 * Most L1 functions operate on well-defined object types represented by
 * FXObjType, and return standardized status codes consistent with those
 * described in FXErrorCode.h and FXCommon.h.
 */
#ifndef FX_L1ROBOT_H_
#define FX_L1ROBOT_H_

#include "FXCommon.h"
#include "FXErrorCode.h"
#include "L0Robot.h"
#include "L0KineMotion.h"

/* Platform-specific export/import macros */
#if defined(_WIN32) || defined(_WIN64)
#ifdef L1_SDK_EXPORTS
#define FX_L1_SDK_API __declspec(dllexport)
#else
#define FX_L1_SDK_API __declspec(dllimport)
#endif
#elif defined(__linux__)
#ifdef L1_SDK_EXPORTS
#define FX_L1_SDK_API __attribute__((visibility("default")))
#else
#define FX_L1_SDK_API
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif
    /* =========================================================
     * System-level APIs
     * ========================================================= */

    /**
     * @brief Establish a UDP link to a remote robot controller.
     *
     * This function creates a network connection to a robot controller
     * identified by its IPv4 address, performs version negotiation,
     * and initializes logging for the session.
     *
     * @param ip1 First octet of the controller IPv4 address.
     * @param ip2 Second octet of the controller IPv4 address.
     * @param ip3 Third octet of the controller IPv4 address.
     * @param ip4 Fourth octet of the controller IPv4 address.
     * @param log_level Logging verbosity level for this link.
     * @return Network latency indication on success, or a negative error code
     *         defined in FXFuncReturn.
     *
     * @see FX_L1_System_SetLogLevel
     */
    FX_L1_SDK_API int FX_L1_System_Link(unsigned char ip1,
                                        unsigned char ip2,
                                        unsigned char ip3,
                                        unsigned char ip4,
                                        unsigned int log_level);
    /**
     * @brief Unlinks the UDP link.
     *
     * This function performs the necessary operations to detach or unlink the L1
     * system component, ensuring that all associated resources are released and
     * no further interactions are routed through this system.
     *
     * @return void
     */
    FX_L1_SDK_API void FX_L1_System_Unlink();
    /**
     * @brief Retrieve the current communication link state of the system.
     *
     * This function queries the status of the communication link between the
     * local system and the connected target device or network entity.
     *
     * @return int Link state indicator.
     *             0 typically indicates a disconnected or inactive link;
     *             1 values indicate an active or established connection.
     *             -1 values indicate an active connection but can't receive 
     *                data from controller 
     *
     * @note The returned value represents the most recently known link status
     *       and may not reflect real-time link fluctuations.
     */
    FX_L1_SDK_API int FX_L1_System_GetLinkState();
    /**
     * @brief Set the global log level for L1 APIs.
     *
     * Configures the minimum severity threshold for log messages generated
     * by the L1 SDK. Messages below this level are suppressed.
     *
     * @param log_level Desired log level mask or enumeration value.
     * @return void
     *
     * @see FX_L1_System_GetLogLevel
     */
    FX_L1_SDK_API void FX_L1_System_SetLogLevel(unsigned int log_level);

    /**
     * @brief Retrieve the current global log level.
     *
     * @return Currently configured log level.
     */
    FX_L1_SDK_API unsigned int FX_L1_System_GetLogLevel();

    /**
     * @brief Query the controller firmware version.
     *
     * Returns an encoded version number representing the controller firmware.
     *
     * @return Encoded version value, or FUNC_RET_OPERATION_FAILED if unavailable.
     *
     * @see FX_L1_System_GetSDKVersion
     */
    FX_L1_SDK_API int FX_L1_System_GetControllerVersion();

    /**
     * @brief Query the SDK version.
     *
     * Returns an encoded version number identifying the L1 SDK build.
     *
     * @return Encoded SDK version value, or FUNC_RET_OPERATION_FAILED if unavailable.
     */
    FX_L1_SDK_API int FX_L1_System_GetSDKVersion();

    /**
     * @brief Reboot the robot system.
     *
     * Initiates a controlled reboot of the robot controller or system.
     *
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_System_Reboot();

    /**
     * @brief Perform a system update using specified files.
     *
     * Applies an update package and configuration to the robot system.
     *
     * @param update_file_path Path to the update binary or image.
     * @param ini_file_path Path to the update configuration file.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_System_Update(char *update_file_path,
                                          char *ini_file_path);

    /**
     * @brief Send a file to the remote system.
     *
     * Transfers a local file to a specified path on the robot controller.
     *
     * @param local_file_path Local file path.
     * @param remote_file_path Destination path on the controller.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_System_SendFile(char *local_file_path,
                                            char *remote_file_path);

    /**
     * @brief Receive a file from the remote system.
     *
     * Retrieves a file from the controller and stores it locally.
     *
     * @param local_file_path Local destination path.
     * @param remote_file_path Source path on the controller.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_System_RecvFile(char *local_file_path,
                                            char *remote_file_path);

    /* =========================================================
     * Feedback APIs
     * ========================================================= */

    /**
     * @brief Get the degree of freedom count for a control object.
     *
     * @param obj_type Target object type.
     * @return Number of degrees of freedom, or -1 on error.
     */
    FX_L1_SDK_API int FX_L1_Fbk_GetCtrlObjDof(FXObjType obj_type);

    /**
     * @brief Retrieve servo firmware versions for a control object.
     *
     * @param obj_type Target object type.
     * @param version Output buffer for version strings.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Fbk_GetCtrlObjServoVersion(FXObjType obj_type,
                                                        char version[7][30]);

    /**
     * @brief Retrieve sensor firmware versions and serial numbers.
     *
     * @param obj_type Target object type.
     * @param version Output array of version integers.
     * @param serial Output array of serial numbers.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Fbk_GetCtrlObjSensorVersionAndSerial(
        FXObjType obj_type, int version[7], int serial[7]);

    /**
     * @brief Query the physical usage state of a control object.
     *
     * @param obj_type Target object type.
     * @param physical_state Output pointer for physical state.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     * @see FXObjPhysicalState
     */
    FX_L1_SDK_API int FX_L1_Fbk_GetCtrlObjPhysicalState(FXObjType obj_type,
                                                        int *physical_state);
    /**
     * @brief Get type of the linked robot.
     *
     * @return Robot type defined by FXRobotType.
     *
     * @see FXRobotType
     */
    FX_L1_SDK_API FXRobotType FX_L1_Fbk_GetRobotType();

    /**
     * @brief Query the current high-level state of a control object.
     *
     * @param obj_type Target object type.
     * @return Current FXStateType value.
     *
     * @see FXStateType
     */
    FX_L1_SDK_API FXStateType FX_L1_Fbk_CurrentState(FXObjType obj_type);

    /**
     * @brief Access real-time feedback data.
     *
     * Provides read-only access to the latest ROBOT_RT structure.
     *
     * @return Pointer to ROBOT_RT data.
     */
    FX_L1_SDK_API const ROBOT_RT *FX_L1_Fbk_GetRT();

    /**
     * @brief Access slow-group feedback data.
     *
     * Provides read-only access to the latest ROBOT_SG structure.
     *
     * @return Pointer to ROBOT_SG data.
     */
    FX_L1_SDK_API const ROBOT_SG *FX_L1_Fbk_GetSG();

    /**
     * @brief Sample the user data from feedback.
     *
     * @param data_ptr Pointer to store the retrieved user data.
     */
    FX_L1_SDK_API void FX_L1_Fbk_GetUserData(void* data_ptr);

     /**
     * @brief Reset all registered user data sets.
     *
     * @return Defined in FXFuncReturn.
     *
     * @note This function clears all previously registered user data sets.
     */
    FX_L1_SDK_API void FX_L1_Fbk_ResetUserDataSet();
    
    /**
     * @brief Register a user data set for feedback.
     *
     * @param name      Name identifier for the user data set.
     * @param data_type Data type (@ref FXUserDataType).
     * @param sub       Sub-index for the data set.
     * @param data_num  Number of data elements.
     *
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn, FXUserDataType
     */
    FX_L1_SDK_API int FX_L1_Fbk_RegisterUserDataSet(char* name, FXUserDataType data_type, int sub, int data_num);
    
    /**
     * @brief Check the validity of user data set length.
     *
     * @param user_data_len Total length of user data to verify.
     *
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Fbk_CheckUserDataSet(int user_data_len);

    /**
     * @brief Retrieve servo error codes for a control object.
     *
     * @param obj_type Target object type.
     * @param error_code Output array for error codes.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     * @see FXErrorCode
     */
    FX_L1_SDK_API int FX_L1_State_GetServoErrorCode(FXObjType obj_type,
                                                    unsigned int error_code[7]);

    /* =========================================================
     * State Machine APIs
     * ========================================================= */

    /**
     * @brief Reset error state for a control object.
     *
     * Clears error conditions and restores the object to a safe state.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for reset, in milliseconds.
     * @param system_errorcode Output system error code.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_ResetError(FXObjType obj_type,
                                             unsigned int timeout,
                                             unsigned int *system_errorcode);

    /**
     * @brief Switch a control object to idle state.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for state transition.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToIdle(FXObjType obj_type,
                                               unsigned int timeout);

    /**
     * @brief Switch a control object to position control mode.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @param vel_ratio Velocity scaling ratio.
     * @param acc_ratio Acceleration scaling ratio.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToPositionMode(FXObjType obj_type,
                                                       unsigned int timeout,
                                                       double vel_ratio,
                                                       double acc_ratio);

    /**
     * @brief Switch to joint-space impedance control mode.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @param vel_ratio Velocity scaling ratio.
     * @param acc_ratio Acceleration scaling ratio.
     * @param k Stiffness coefficients.
     * @param d Damping coefficients.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToImpJointMode(FXObjType obj_type,
                                                       unsigned int timeout,
                                                       double vel_ratio,
                                                       double acc_ratio,
                                                       double k[7],
                                                       double d[7]);

    /**
     * @brief Switch to Cartesian-space impedance control mode.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @param vel_ratio Velocity scaling ratio.
     * @param acc_ratio Acceleration scaling ratio.
     * @param k Cartesian stiffness coefficients.
     * @param d Cartesian damping coefficients.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToImpCartMode(FXObjType obj_type,
                                                      unsigned int timeout,
                                                      double vel_ratio,
                                                      double acc_ratio,
                                                      double k[7],
                                                      double d[7]);

    /**
     * @brief Switch to force-based impedance control mode.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @param force_ctrl Force control parameters.
     * @param torque_ctrl Torque control parameters.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToImpForceMode(
        FXObjType obj_type,
        unsigned int timeout,
        double force_ctrl[FX_FORCE_DEF_NUM],
        double torque_ctrl[FX_TORQUE_DEF_NUM]);

    /**
     * @brief Switch to PD control mode.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @param vel_ratio Velocity scaling ratio.
     * @param acc_ratio Acceleration scaling ratio.
     * @param k Stiffness coefficients.
     * @param d Damping coefficients.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToPDMode(FXObjType obj_type,
                                                 unsigned int timeout,
                                                 double vel_ratio,
                                                 double acc_ratio,
                                                 double k[7],
                                                 double d[7]);

    /**
     * @brief Switch to joint-space drag teaching mode.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @param k Stiffness coefficients.
     * @param d Damping coefficients.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToDragJoint(FXObjType obj_type,
                                                    unsigned int timeout,
                                                    double k[7],
                                                    double d[7]);

    /**
     * @brief Switch to Cartesian drag teaching along the X axis.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @param k Stiffness coefficients.
     * @param d Damping coefficients.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToDragCartX(FXObjType obj_type,
                                                    unsigned int timeout,
                                                    double k[7],
                                                    double d[7]);

    /**
     * @brief Switch to Cartesian drag teaching along the Y axis.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @param k Stiffness coefficients.
     * @param d Damping coefficients.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToDragCartY(FXObjType obj_type,
                                                    unsigned int timeout,
                                                    double k[7],
                                                    double d[7]);

    /**
     * @brief Switch to Cartesian drag teaching along the Z axis.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @param k Stiffness coefficients.
     * @param d Damping coefficients.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToDragCartZ(FXObjType obj_type,
                                                    unsigned int timeout,
                                                    double k[7],
                                                    double d[7]);

    /**
     * @brief Switch to rotational Cartesian drag teaching.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @param k Stiffness coefficients.
     * @param d Damping coefficients.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToDragCartR(FXObjType obj_type,
                                                    unsigned int timeout,
                                                    double k[7],
                                                    double d[7]);

    /**
     * @brief Switch to collaborative release mode.
     *
     * @param obj_type Target object type.
     * @param timeout Maximum time allowed for transition.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToCollaborativeRelease(
        FXObjType obj_type, unsigned int timeout);

    /* =========================================================
     * Parameter Management APIs
     * ========================================================= */

    /**
     * @brief Set an integer parameter by name.
     *
     * @param name Parameter name string.
     * @param value Integer value to assign.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Param_SetInt32(char *name, int value);

    /**
     * @brief Set a floating-point parameter by name.
     *
     * @param name Parameter name string.
     * @param value Float value to assign.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Param_SetFloat(char *name, float value);

    /**
     * @brief Get an integer parameter by name.
     *
     * @param name Parameter name string.
     * @param value Output pointer for retrieved value.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Param_GetInt32(char *name, int *value);

    /**
     * @brief Get a floating-point parameter by name.
     *
     * @param name Parameter name string.
     * @param value Output pointer for retrieved value.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Param_GetFloat(char *name, float *value);

    /**
     * @brief Get a string parameter by name.
     *
     * @param name Parameter name string.
     * @param value Output buffer for retrieved string.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Param_GetString(char *name, char *value);

    /* =========================================================
     * Terminal Communication APIs
     * ========================================================= */

    /**
     * @brief Clear terminal communication data.
     *
     * @param terminal_type Target terminal type.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Terminal_ClearData(FXTerminalType terminal_type);

    /**
     * @brief Receive data from a terminal.
     *
     * @param terminal_type Target terminal type.
     * @param timeout Maximum time allowed for receive data.
     * @param chn_type Output channel type.
     * @param data Output data buffer.
     * @param receiving_time Receiving time in millisecond.
     * @return >=0 Represents for the data bytes received
     *         <0 Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Terminal_GetData(FXTerminalType terminal_type,
                                             unsigned int timeout,
                                             FXChnType *chn_type,
                                             unsigned char data[64],
                                             unsigned int *receving_time);

    /**
     * @brief Send data to a terminal.
     *
     * @param terminal_type Target terminal type.
     * @param chn_type Channel type.
     * @param timeout Maximum time allowed for send data.
     * @param data Input data buffer.
     * @param data_len Length of data in bytes.
     * @param sending_time Sending time in millisecond.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Terminal_SetData(FXTerminalType terminal_type,
                                             FXChnType chn_type,
                                             unsigned int timeout,
                                             unsigned char data[64],
                                             unsigned int data_len,
                                             unsigned int *sending_time);

    /* =========================================================
     * Hardware Configuration APIs
     * ========================================================= */

    /**
     * @brief Lock brakes for specified axes.
     *
     * @param obj_type Target object type.
     * @param axis_mask Bitmask of axes to lock.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Config_SetBrakeLock(FXObjType obj_type,
                                                unsigned char axis_mask);

    /**
     * @brief Unlock brakes for specified axes.
     *
     * @param obj_type Target object type.
     * @param axis_mask Bitmask of axes to unlock.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Config_SetBrakeUnlock(FXObjType obj_type,
                                                  unsigned char axis_mask);

    /**
     * @brief Reset encoder offsets.
     *
     * @param obj_type Target object type.
     * @param axis_mask Bitmask of axes to reset.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Config_ResetEncOffset(FXObjType obj_type,
                                                  unsigned char axis_mask);

    /**
     * @brief Clear encoder errors.
     *
     * @param obj_type Target object type.
     * @param axis_mask Bitmask of axes to clear.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Config_ClearEncError(FXObjType obj_type,
                                                 unsigned char axis_mask);

    /**
     * @brief Reset axis sensor offset.
     *
     * @param obj_type Target object type.
     * @param axis_id Axis identifier.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Config_ResetAxisSensorOffset(FXObjType obj_type,
                                                         unsigned int axis_id);

    /**
     * @brief Reset all sensor offsets.
     *
     * @param obj_type Target object type.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Config_ResetSensorOffset(FXObjType obj_type);

    /**
     * @brief Disable software joint limits.
     *
     * @param obj_type Target object type.
     * @param axis_mask Bitmask of axes to disable limits.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Config_DisableSoftLimit(FXObjType obj_type,
                                                    unsigned char axis_mask);

    /**
     * @brief Set the command sending cycle time in PD mode.
     *
     * This function configures how frequently commands are sent to the system
     * while operating in PD (Position Demand) mode.
     *
     * @param[in] cycle_time  Command cycle time, in millsecond.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Config_SetPDCmdCycleTime(int cycle_time);

    /**
     * @brief Retrieve the current command sending cycle time in PD mode.
     *
     * This function obtains the currently configured command transmission
     * frequency used by the system in PD (Position Demand) mode.
     *
     * @param[out] cycle_time Pointer to an integer where the command cycle time
     *                      (in millsecond) will be stored.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Config_GetPDCmdCycleTime(int *cycle_time);

    /**
     * @brief Configure a trajectory for execution.
     *
     * @param obj_type Target object type.
     * @param point_num Number of trajectory points.
     * @param point_data Array of trajectory point data.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Config_SetTraj(FXObjType obj_type,
                                           unsigned int point_num,
                                           double *point_data);

    /* =========================================================
     * Runtime Motion Control APIs
     * ========================================================= */

    /**
     * @brief Trigger emergency stop for selected objects.
     *
     * @param obj_mask Bitmask of affected objects.
     * @return Bitmask of objects successfully stopped.
     */
    FX_L1_SDK_API unsigned int FX_L1_Runtime_EmergencyStop(unsigned int thread_id, unsigned int obj_mask);

    /**
     * @brief Send user defined tag command.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param tag User defined tag.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetTag(unsigned int thread_id, FXObjType obj_type, int tag);

    /**
     * @brief Send joint position command.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param pos_cmd Joint position command array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetJointPosCmd(unsigned int thread_id, FXObjType obj_type, double pos_cmd[7]);

    /**
     * @brief Send joint position command in PD mode.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param pos_cmd Joint position command array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetJointPosPDCmd(unsigned int thread_id, FXObjType obj_type, double pos_cmd[7]);

    /**
     * @brief Apply force control parameters.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param force_ctrl Force control definition array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetForceCtrl(unsigned int thread_id, FXObjType obj_type, double force_ctrl[FX_FORCE_DEF_NUM]);

    /**
     * @brief Apply torque control parameters.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param torque_ctrl Torque control definition array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetTorqueCtrl(unsigned int thread_id, FXObjType obj_type, double torque_ctrl[FX_TORQUE_DEF_NUM]);

    /**
     * @brief Set velocity scaling ratio.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param vel_ratio Velocity scaling factor.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetVelRatio(unsigned int thread_id, FXObjType obj_type, double vel_ratio);

    /**
     * @brief Set acceleration scaling ratio.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param acc_ratio Acceleration scaling factor.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetAccRatio(unsigned int thread_id, FXObjType obj_type, double acc_ratio);

    /**
     * @brief Set velocity and acceleration scaling ratios.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param vel_ratio Velocity scaling factor.
     * @param acc_ratio Acceleration scaling factor.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetSpeedRatio(unsigned int thread_id, FXObjType obj_type, double vel_ratio, double acc_ratio);

    /**
     * @brief Set joint stiffness coefficients.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param k Joint stiffness array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetJointK(unsigned int thread_id, FXObjType obj_type, double k[7]);

    /**
     * @brief Set joint damping coefficients.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param d Joint damping array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetJointD(unsigned int thread_id, FXObjType obj_type, double d[7]);

    /**
     * @brief Set joint stiffness and damping coefficients.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param k Joint stiffness array.
     * @param d Joint damping array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetJointKD(unsigned int thread_id, FXObjType obj_type, double k[7], double d[7]);

    /**
     * @brief Set Cartesian stiffness coefficients.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param k Cartesian stiffness array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetCartK(unsigned int thread_id, FXObjType obj_type, double k[7]);

    /**
     * @brief Set Cartesian damping coefficients.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param d Cartesian damping array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetCartD(unsigned int thread_id, FXObjType obj_type, double d[7]);

    /**
     * @brief Set Cartesian stiffness and damping coefficients.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param k Cartesian stiffness array.
     * @param d Cartesian damping array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetCartKD(unsigned int thread_id, FXObjType obj_type, double k[7], double d[7]);

    /**
     * @brief Set tool kinematics parameters.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param k Tool kinematic parameters.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetToolK(unsigned int thread_id, FXObjType obj_type, double k[6]);

    /**
     * @brief Set tool dynamics parameters.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param d Tool dynamic parameters.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetToolD(unsigned int thread_id, FXObjType obj_type, double d[10]);

    /**
     * @brief Set tool kinematics and dynamics parameters.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_type Target object type.
     * @param k Tool kinematic parameters.
     * @param d Tool dynamic parameters.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetToolKD(unsigned int thread_id, FXObjType obj_type, double k[6], double d[10]);

    /**
     * @brief Set body PD control proportional gains.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param p Proportional gain array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetBodyPDP(unsigned int thread_id, double p[6]);

    /**
     * @brief Set body PD control derivative gains.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param d Derivative gain array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetBodyPDD(unsigned int thread_id, double d[6]);

    /**
     * @brief Set body PD control gains.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param p Proportional gain array.
     * @param d Derivative gain array.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetBodyPD(unsigned int thread_id, double p[6], double d[6]);

    /**
     * @brief Start trajectory execution.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_mask Bitmask of objects executing the trajectory.
     * @return Bitmask of objects successfully started.
     */
    FX_L1_SDK_API unsigned int FX_L1_Runtime_RunTraj(unsigned int thread_id, unsigned int obj_mask);

    /**
     * @brief Stop trajectory execution.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param obj_mask Bitmask of objects to stop.
     * @return Bitmask of objects successfully stopped.
     */
    FX_L1_SDK_API unsigned int FX_L1_Runtime_StopTraj(unsigned int thread_id, unsigned int obj_mask);

    /**
     * @brief Set the runtime action command for the specified hand.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param hand_type  Hand identifier (@ref FXHandType).
     * @param hand_action Action to be applied (@ref FXHandAction).
     *
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetHandAction(unsigned int thread_id, FXHandType hand_type, FXHandAction hand_action);

    /**
     * @brief Set the target position command for the specified hand.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param hand_type Hand identifier (@ref FXHandType).
     * @param pos       Target position array for each joint (length = 24).
     *
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetHandPos(unsigned int thread_id, FXHandType hand_type, int pos[24]);

    /**
     * @brief Set the proportional gain (P) for the specified hand.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param hand_type Hand identifier (@ref FXHandType).
     * @param p         Proportional gain array for each joint (length = 24).
     *
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetHandP(unsigned int thread_id, FXHandType hand_type, int p[24]);

    /**
     * @brief Set the derivative gain (D) for the specified hand.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param hand_type Hand identifier (@ref FXHandType).
     * @param d         Derivative gain array for each joint (length = 24).
     *
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetHandD(unsigned int thread_id, FXHandType hand_type, int d[24]);

    /**
     * @brief Set the maximum torque limit for the specified hand.
     *
     * @param thread_id Thread ID for this function call, range 1~7.
     * @param hand_type Hand identifier (@ref FXHandType).
     * @param max_tor   Maximum torque array for each joint (length = 24).
     *
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetHandMaxTor(unsigned int thread_id, FXHandType hand_type, int max_tor[24]);

    /* =========================================================
     * Kinematics and motion planning APIs
     * ========================================================= */

    /**
     * @brief Opaque handle for the kinematics context.
     *
     * The handle owns the single-arm kinematics, MAX body kinematics,
     * and motion planning related objects.
     */
    typedef struct FX_MotionContext *FX_MotionHandle;
    /**
     * @brief Create a kinematics context.
     *
     * @return A valid handle on success, or `null` on failure.
     */
    FX_L1_SDK_API FX_MotionHandle FX_L1_Kinematics_Create(void);
    /**
     * @brief Destroy a kinematics context.
     *
     * @param[in] handle Context handle created by `FX_L0_Kinematics_create`.
     */
    FX_L1_SDK_API void FX_L1_Kinematics_Destroy(FX_MotionHandle handle);
    /**
     * @brief Control the logging of kinematics-related information.
     *
     * @param[in] log_level Log level. For details, please refer to Kinematics/FXLog.h.
     */
    FX_L1_SDK_API void FX_L1_Kinematics_SetLogLevel(unsigned int log_level = FX_LOG_INFO_FLAG);
    /**
     * @brief Initialize the kinematics environment for a single arm using input parameters.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] RobotSerial Arm index, `0` for left arm and `1` for right arm.
     * @param[in] type Pointer to the robot joint type array.
     * @param[in] DH Robot DH parameter table, size `[8][4]`.
     * @param[in] PNVA Robot PNVA parameter table, size `[8][4]`.
     * @param[in] BOUND Joint boundary limits, size `[4][3]`.
     * @param[in] GRV Gravity vector, size `[3]`.
     * @param[in] MASS Link mass array, size `[7]`.
     * @param[in] MCP Link center of mass positions, size `[7][3]`.
     * @param[in] I Link inertia parameter table, size `[7][6]`.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_InitSingleArm_ByInputParams(FX_MotionHandle handle, int RobotSerial, int *type, double DH[8][4], double PNVA[8][4], double BOUND[4][3],
                                                                   double GRV[3], double MASS[7], double MCP[7][3], double I[7][6]);
    /**
     * @brief Initialize the kinematics environment for a single arm using default ini configuration. Robot must be connected.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] RobotSerial Arm index, `0` for left arm and `1` for right arm.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_InitSingleArm_ByIniConfig(FX_MotionHandle handle, int RobotSerial);

    /* Kinematics Calculation */
    /**
     * @brief Set the tool transformation matrix for the specified robot arm.
     *
     * This function sets the homogeneous transformation matrix of the tool
     * relative to the robot flange coordinate system.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Robot arm index, `0` for left arm and `1` for right arm.
     * @param[in] tool 4x4 homogeneous transformation matrix in row-major order.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_SetTool(FX_MotionHandle handle, int robot_serial, double tool[4][4]);

    /**
     * @brief Remove the tool transformation for the specified robot arm.
     *
     * This function clears the currently configured tool transformation and
     * restores the default tool configuration.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Robot arm index, `0` for left arm and `1` for right arm.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_RemoveTool(FX_MotionHandle handle, int robot_serial);

    /**
     * @brief Solve forward kinematics for one arm.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index, `0` for left arm and `1` for right arm.
     * @param[in] joints Input joint values, length 7.
     * @param[out] pose_matrix Output TCP pose matrix, length 16 in 4x4 row-major order.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_ForwardKinematics(FX_MotionHandle handle, int robot_serial,
                                                         double joints[7], double pose_matrix[4][4]);
    /**
     * @brief Solve the Jacobian matrix for one arm.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index, `0` for left arm and `1` for right arm.
     * @param[in] joints Input joint values, length 7.
     * @param[out] jacobian Output Jacobian, length 42 in 6x7 row-major order.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_Jacobian(FX_MotionHandle handle, int robot_serial,
                                                double joints[7], double jacobian[6][7]);
    /**
     * @brief Solve inverse kinematics for one arm.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index, `0` for left arm and `1` for right arm.
     * @param[in,out] params IK input/output parameters including target pose,
     *                      reference joints, and solution buffer.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_InverseKinematics(FX_MotionHandle handle, int robot_serial,
                                                         FX_InvKineSolvePara *params);

    /* Skye Body Kinematics */
    /**
     * @brief Set MAX body kinematics parameters.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] std_body Standard body position parameters, length 3.
     * @param[in] k_body Body stiffness parameters, length 3.
     * @param[in] std_arm0_len Standard length of the left arm.
     * @param[in] k_arm1 Stiffness coefficient of the left arm.
     * @param[in] std_arm1_len Standard length of the right arm.
     * @param[in] k_arm1 Stiffness coefficient of the right arm.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_SetSkyeBodyCondition(FX_MotionHandle handle,
                                                            double std_body[3], double k_body[3],
                                                            double std_arm0_len, double k_arm0,
                                                            double std_arm1_len, double k_arm1);
    /**
     * @brief Solve forward kinematics for the MAX body.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] jv Input body joint values, length 3.
     * @param[out] arm0_shoulder_matrix Output left shoulder matrix, length 16 in
     *                                  4x4 row-major order.
     * @param[out] arm1_shoulder_matrix Output right shoulder matrix, length 16 in
     *                                   4x4 row-major order.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_SkyeBodyForwardKinematics(FX_MotionHandle handle, double jv[3],
                                                                 double arm0_shoulder_matrix[4][4], double arm1_shoulder_matrix[4][4]);
    /**
     * @brief Compute body joint values from dual-arm TCP positions.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] arm0_tcp Left TCP position, length 3.
     * @param[in] arm1_tcp Right TCP position, length 3.
     * @param[out] out_body_joints Output body joints, length 3.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_SkyeBodyInverseKinematics(FX_MotionHandle handle,
                                                                 double arm0_tcp[3], double arm1_tcp[3],
                                                                 double out_body_joints[3]);
    /**
     * @brief Compute body joint values with a reference body pose.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] ref_body_joints Reference body joints, length 3.
     * @param[in] arm0_tcp Left TCP position, length 3.
     * @param[in] arm1_tcp Right TCP position, length 3.
     * @param[out] out_body_joints Output body joints, length 3.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_SkyeBodyInverseKinematicsWithRef(FX_MotionHandle handle,
                                                                        double ref_body_joints[3],
                                                                        double arm0_tcp[3], double arm1_tcp[3],
                                                                        double out_body_joints[3]);
    /**
     * @brief Plan a joint-space MoveJ path.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index. The parameter is currently reserved by this API.
     * @param[in] start_joints Start joint values, length 7.
     * @param[in] end_joints Target joint values, length 7.
     * @param[in] vel_ratio Velocity ratio. Range `(0, 1]`.
     * @param[in] acc_ratio Acceleration ratio. Range `(0, 1]`
     * @param[in] freq Path sampling frequency.
     * @param[in,out] point_set_handle Path point set handle created by
     *                                 `FX_L0_CPointSet_Create`.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_PlanJointMove(FX_MotionHandle handle, int robot_serial,
                                                     double start_joints[7], double end_joints[7],
                                                     double vel_ratio, double acc_ratio, int freq,
                                                     double *point_set_handle, int *point_num);
    /**
     * @brief Plan a Cartesian linear MoveL path.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index. The parameter is currently reserved by this API.
     * @param[in] start_xyzabc Start pose in `XYZABC` format, length 6, angles in degrees.
     * @param[in] end_xyzabc Target pose in `XYZABC` format, length 6, angles in degrees.
     * @param[in] ref_joints Reference joint values, length 7.
     * @param[in] vel Path velocity.
     * @param[in] acc Path acceleration.
     * @param[in] freq Path sampling frequency.
     * @param[in,out] point_set_handle Path point set handle created by
     *                                 `FX_L0_CPointSet_Create`.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_PlanLinearMove(FX_MotionHandle handle, int robot_serial,
                                                      double start_xyzabc[6], double end_xyzabc[6],
                                                      double ref_joints[7],
                                                      double vel, double acc, int freq,
                                                      double *point_set_handle, int *point_num);
    /**
     * @brief Plan a linear path while keeping joint posture.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index. The parameter is currently reserved by this API.
     * @param[in] start_joints Start joint values, length 7.
     * @param[in] end_joints Target joint values, length 7.
     * @param[in] vel Path velocity.
     * @param[in] acc Path acceleration.
     * @param[in] freq Path sampling frequency.
     * @param[in,out] point_set_handle Path point set handle created by
     *                                 `FX_L0_CPointSet_Create`.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_PlanLinearMoveKeepJoints(FX_MotionHandle handle, int robot_serial,
                                                                double start_joints[7], double end_joints[7],
                                                                double vel, double acc, int freq,
                                                                double *point_set_handle, int *point_num);
    /**
     * @brief Start a multi-segment Cartesian MoveL planning sequence.
     *
     * This call initializes a continuous multi-point linear MoveL path and
     * submits the first segment of the path.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index. The parameter is currently reserved by this API.
     * @param[in] ref_joints Reference joint values, length 7.
     * @param[in] start_xyzabc Start pose of the first segment, length 6, in `XYZABC`
     *                         format with angles in degrees.
     * @param[in] end_xyzabc End pose of the first segment, length 6, in `XYZABC`
     *                       format with angles in degrees.
     * @param[in] allow_range Allowed blending or transition range.
     * @param[in] zsp_type Zero-space planning type.
     * @param[in] zsp_para Zero-space planning parameters, length 6.
     * @param[in] vel Segment velocity.
     * @param[in] acc Segment acceleration.
     * @param[in] freq Path sampling frequency.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetStart(FX_MotionHandle handle, int robot_serial,
                                                                           double ref_joints[7],
                                                                           double start_xyzabc[6], double end_xyzabc[6],
                                                                           double allow_range, int zsp_type,
                                                                           double zsp_para[6],
                                                                           double vel, double acc, int freq);
    /**
     * @brief Append the next segment to a multi-segment Cartesian MoveL path.
     *
     * Call this after `FX_L0_Kinematics_multi_points_set_movl_start` to keep
     * adding the following Cartesian target points into the same planning sequence.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index. The parameter is currently reserved by this API.
     * @param[in] next_xyzabc End pose of the next segment, length 6, in `XYZABC`
     *                        format with angles in degrees.
     * @param[in] allow_range Allowed blending or transition range.
     * @param[in] zsp_type Zero-space planning type.
     * @param[in] zsp_para Zero-space planning parameters, length 6.
     * @param[in] vel Segment velocity.
     * @param[in] acc Segment acceleration.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints(FX_MotionHandle handle, int robot_serial,
                                                                                double next_xyzabc[6],
                                                                                double allow_range, int zsp_type,
                                                                                double zsp_para[6],
                                                                                double vel, double acc);
    /**
     * @brief Export the planned path of a multi-segment Cartesian MoveL sequence.
     *
     * This call collects the path generated by the previously submitted
     * multi-point MoveL segments and writes it into the output point buffer.
     *
     * @param[in] handle Kinematics context handle.
     * @param[out] point_set_handle Output point buffer.
     * @param[out] point_num Output number of planned points.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_PlanLinearMove_MultiPoints_GetPoints(FX_MotionHandle handle,
                                                                            double *point_set_handle, int *point_num);

    /* ArmsSynchronousPlanning(body fixed) */
    /**
     * @brief Plan a synchronized dual-arm linear path with fixed body state.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] params Dual-arm fixed-body planning parameters.
     * @param[in,out] arm0_point_set Left arm path point set handle created by
     *                               `FX_L0_CPointSet_Create`.
     * @param[in,out] arm1_point_set Right arm path point set handle created by
     *                                `FX_L0_CPointSet_Create`.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_ArmsSynchronousPlanning(FX_MotionHandle handle,
                                                               ArmsSynchronousPlanningParams *params,
                                                               double *arm0_point_set, double *arm1_point_set, int *point_num);

    /* Dynamics Identification*/
    /**
     * @brief Identify the dynamic parameters of the robot tool/load.
     *
     * @param[in] robot_type Robot model type identifier.
     * @param[in] file_path Path to the identification data file.
     * @param[out] mass Identified load mass.
     * @param[out] mr Identified first-order moment vector, size `[3]`.
     * @param[out] inertia Identified inertia parameter vector, size `[6]`.
     * @return Defined in FXFuncReturn.
     *
     * @see FXFuncReturn
     */
    FX_L1_SDK_API int FX_L1_Kinematics_DynamicsIdentification(int robot_type, char *file_path, double *mass, double mr[3], double inertia[6]);

    /**
     * @brief Convert an `XYZABC` pose to a 4x4 transform matrix.
     *
     * @param[in] xyzabc Input pose, length 6, angles in degrees.
     * @param[out] matrix Output matrix, length 16 in 4x4 row-major order.
     */
    FX_L1_SDK_API void FX_L1_XYZABC2Matrix(double xyzabc[6], double matrix[4][4]);
    /**
     * @brief Convert a 4x4 transform matrix to an `XYZABC` pose.
     *
     * @param[in] matrix Input matrix, length 16 in 4x4 row-major order.
     * @param[out] xyzabc Output pose, length 6, angles in degrees.
     */
    FX_L1_SDK_API void FX_L1_Matrix2XYZABC(double matrix[4][4], double xyzabc[6]);

#ifdef __cplusplus
}
#endif

#endif
