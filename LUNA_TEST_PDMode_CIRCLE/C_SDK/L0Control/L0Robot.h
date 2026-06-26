/**
 * @file L0Robot.h
 * @brief FX L0 Robot SDK API definitions.
 *
 * This header defines the Layer 0 (L0) application programming interface for
 * interacting with FX robot systems at a low level. L0 APIs provide direct
 * control over individual robot subsystems, including system management,
 * communication, parameter handling, terminal I/O, state management,
 * hardware configuration, and runtime motion control.
 */

#ifndef FX_L0ROBOT_H_
#define FX_L0ROBOT_H_

#include "RobotCtrl.h"

#if defined(_WIN32) || defined(_WIN64)
#define CONTROL_SDK_API __declspec(dllexport)
#elif defined(__linux__)
#define CONTROL_SDK_API
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  /* =========================================================
   * System Management APIs
   * ========================================================= */

  /**
   * @brief Request control of the target system.
   *
   * This function attempts to obtain control authority over the specified
   * system identified by its IPv4 address. Control may be granted or denied
   * based on system policy, current ownership state, and access permissions.
   *
   * @param[in] ip1 First octet of the target IPv4 address.
   * @param[in] ip2 Second octet of the target IPv4 address.
   * @param[in] ip3 Third octet of the target IPv4 address.
   * @param[in] ip4 Fourth octet of the target IPv4 address.
   * @return 0 values indicate success.
   *        -1 values indicate link is not established.
   *        -2 values indicate the request is rejected by the controller.
   */
  CONTROL_SDK_API int FX_L0_System_RequestControl(unsigned char ip1,
                                          unsigned char ip2,
                                          unsigned char ip3,
                                          unsigned char ip4);

  /**
   * @brief Establish a UDP connection to the robot controller.
   *
   * @param ip1 First octet of the controller IPv4 address.
   * @param ip2 Second octet of the controller IPv4 address.
   * @param ip3 Third octet of the controller IPv4 address.
   * @param ip4 Fourth octet of the controller IPv4 address.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_System_Link(unsigned char ip1,
                                        unsigned char ip2,
                                        unsigned char ip3,
                                        unsigned char ip4);

  /**
   * @brief Unlinks the UDP connection to the robot controller.
   *
   * This function detaches the L0 system component from its active configuration
   * or runtime environment. It performs necessary cleanup and ensures that no
   * further operations are routed through the L0 system until it is linked again.
   *
   * @return void
   */
  CONTROL_SDK_API void FX_L0_System_Unlink();
  /**
   * @brief Retrieve the current communication link state of the system.
   *
   * This function queries the status of the communication link between the
   * local system and the connected target device or network entity.
   *
   * @return int Link state indicator.
   *             0 typically indicates a disconnected or inactive link;
   *             1 values indicate an active or established connection;
   *             -1 values indicate an active connection but can't receive 
   *                data from the system in 100ms.
   */
  CONTROL_SDK_API int FX_L0_System_GetLinkState();

  /**
   * @brief Test the current network connection to the controller.
   *
   * @return >0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_System_Testconnect(void);

  /**
   * @brief Check version compatibility between SDK and controller.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_System_CheckVersion(void);

  /**
   * @brief Retrieve the controller firmware version.
   *
   * @return Controller firmware version value, or -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_System_GetControllerVersion(void);

  /**
   * @brief Retrieve the SDK version.
   *
   * @return SDK version value.
   */
  CONTROL_SDK_API int FX_L0_System_GetSdkVersion(void);

  /**
   * @brief Reboot the robot system.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_System_Reboot(void);

  /**
   * @brief Perform a system update.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_System_Update(void);

  /**
   * @brief Set command cycle time in PD mode, in ms.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_System_SetPDCmdCycleTime(int cycle_time);

  /**
   * @brief Get command cycle time in PD mode, in ms.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_System_GetPDCmdCycleTime(int *cycle_time);

  /**
   * @brief Enable local logging.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API void FX_L0_System_LocalLogOn(void);

  /**
   * @brief Disable local logging.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API void FX_L0_System_LocalLogOff(void);

  /* =========================================================
   * Parameter Management APIs
   * ========================================================= */

  /**
   * @brief Retrieve an integer parameter by name.
   *
   * @param name Parameter name string.
   * @param ret_value Output pointer for retrieved value.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Param_GetInt(char name[30], int *ret_value);

  /**
   * @brief Retrieve a floating-point parameter by name.
   *
   * @param name Parameter name string.
   * @param ret_value Output pointer for retrieved value.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Param_GetFloat(char name[30], float *ret_value);

  /**
   * @brief Retrieve a string parameter by name.
   *
   * @param name Parameter name string.
   * @param ret_value Output buffer for retrieved string.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Param_GetString(char name[30], char ret_value[30]);

  /**
   * @brief Set an integer parameter by name.
   *
   * @param name Parameter name string.
   * @param target_value Value to set.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Param_SetInt(char name[30], int target_value);

  /**
   * @brief Set a floating-point parameter by name.
   *
   * @param name Parameter name string.
   * @param target_value Value to set.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Param_SetFloat(char name[30], float target_value);

  /**
   * @brief Save parameters to persistent storage.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Param_Save(void);

  /* =========================================================
   * Terminal Communication APIs
   * ========================================================= */

  /**
   * @brief Clear Arm0 terminal communication data.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Terminal_ClearData(void);

  /**
   * @brief Retrieve data from Arm0 terminal.
   *
   * @param channel_type_ptr Output pointer for channel type.
   * @param data_ptr Output buffer for received data.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Terminal_GetData(int *channel_type_ptr,
                                                  unsigned char data_ptr[64]);

  /**
   * @brief Send data to Arm0 terminal.
   *
   * @param channel_type Communication channel type.
   * @param data_ptr Pointer to data buffer.
   * @param data_len Length of data in bytes.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Terminal_SetData(int channel_type,
                                                  unsigned char *data_ptr,
                                                  int data_len);

  /**
   * @brief Clear Arm1 terminal communication data.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Terminal_ClearData(void);

  /**
   * @brief Retrieve data from Arm1 terminal.
   *
   * @param channel_type_ptr Output pointer for channel type.
   * @param data_ptr Output buffer for received data.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Terminal_GetData(int *channel_type_ptr,
                                                  unsigned char data_ptr[64]);

  /**
   * @brief Send data to Arm1 terminal.
   *
   * @param channel_type Communication channel type.
   * @param data_ptr Pointer to data buffer.
   * @param data_len Length of data in bytes.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Terminal_SetData(int channel_type,
                                                  unsigned char *data_ptr,
                                                  int data_len);

  /* =========================================================
   * State Feedback APIs
   * ========================================================= */

  /**
   * @brief Retrieve servo error code for a specific axis of Arm0.
   *
   * @param axis_id Axis identifier.
   * @param error_code Output pointer for error code.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_State_GetServoErrorCode(int axis_id,
                                                         unsigned int *error_code);

  /**
   * @brief Retrieve servo error code for a specific axis of Arm1.
   *
   * @param axis_id Axis identifier.
   * @param error_code Output pointer for error code.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_State_GetServoErrorCode(int axis_id,
                                                         unsigned int *error_code);

  /**
   * @brief Retrieve servo error code for a specific axis of Head.
   *
   * @param axis_id Axis identifier.
   * @param error_code Output pointer for error code.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_State_GetServoErrorCode(int axis_id,
                                                         unsigned int *error_code);

  /**
   * @brief Retrieve servo error code for a specific axis of Body.
   *
   * @param axis_id Axis identifier.
   * @param error_code Output pointer for error code.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_State_GetServoErrorCode(int axis_id,
                                                         unsigned int *error_code);

  /**
   * @brief Retrieve servo error code for a specific axis of Lift.
   *
   * @param axis_id Axis identifier.
   * @param error_code Output pointer for error code.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_State_GetServoErrorCode(int axis_id,
                                                         unsigned int *error_code);

  /**
   * @brief Retrieve servo firmware version for a specific axis of Arm0.
   *
   * @param axis_id Axis identifier.
   * @param version Output buffer for version string.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_State_GetServoVersion(int axis_id,
                                                       char version[30]);

  /**
   * @brief Retrieve servo firmware version for a specific axis of Arm1.
   *
   * @param axis_id Axis identifier.
   * @param version Output buffer for version string.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_State_GetServoVersion(int axis_id,
                                                       char version[30]);

  /**
   * @brief Retrieve servo firmware version for a specific axis of Head.
   *
   * @param axis_id Axis identifier.
   * @param version Output buffer for version string.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_State_GetServoVersion(int axis_id,
                                                       char version[30]);

  /**
   * @brief Retrieve servo firmware version for a specific axis of Body.
   *
   * @param axis_id Axis identifier.
   * @param version Output buffer for version string.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_State_GetServoVersion(int axis_id,
                                                       char version[30]);

  /**
   * @brief Retrieve sensor firmware version for a specific axis of Arm0.
   *
   * @param axis_id Axis identifier.
   * @param version Output pointer for version value.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_State_GetSensorVersion(int axis_id,
                                                        int *version);

  /**
   * @brief Retrieve sensor firmware version for a specific axis of Arm1.
   *
   * @param axis_id Axis identifier.
   * @param version Output pointer for version value.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_State_GetSensorVersion(int axis_id,
                                                        int *version);

  /**
   * @brief Retrieve sensor firmware version for a specific axis of Body.
   *
   * @param axis_id Axis identifier.
   * @param version Output pointer for version value.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_State_GetSensorVersion(int axis_id,
                                                        int *version);

  /**
   * @brief Retrieve sensor serial number for a specific axis of Arm0.
   *
   * @param axis_id Axis identifier.
   * @param serial Output pointer for serial number.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_State_GetSensorSerial(int axis_id,
                                                       int *serial);

  /**
   * @brief Retrieve sensor serial number for a specific axis of Arm1.
   *
   * @param axis_id Axis identifier.
   * @param serial Output pointer for serial number.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_State_GetSensorSerial(int axis_id,
                                                       int *serial);

  /**
   * @brief Retrieve sensor serial number for a specific axis of Body.
   *
   * @param axis_id Axis identifier.
   * @param serial Output pointer for serial number.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_State_GetSensorSerial(int axis_id,
                                                       int *serial);

  /**
   * @brief Retrieve physical usage state of Arm0.
   *
   * @param physical_state Output pointer for physical state.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_State_GetPhysicalState(int *physical_state);

  /**
   * @brief Retrieve physical usage state of Arm1.
   *
   * @param physical_state Output pointer for physical state.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_State_GetPhysicalState(int *physical_state);

  /**
   * @brief Retrieve physical usage state of Head.
   *
   * @param physical_state Output pointer for physical state.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_State_GetPhysicalState(int *physical_state);

  /**
   * @brief Retrieve physical usage state of Body.
   *
   * @param physical_state Output pointer for physical state.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_State_GetPhysicalState(int *physical_state);

  /**
   * @brief Retrieve physical usage state of Lift.
   *
   * @param physical_state Output pointer for physical state.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_State_GetPhysicalState(int *physical_state);

  /**
   * @brief Reset error state for Arm0.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_State_Reset(void);

  /**
   * @brief Reset error state for Arm1.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_State_Reset(void);

  /**
   * @brief Reset error state for Head.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_State_Reset(void);

  /**
   * @brief Reset error state for Body.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_State_Reset(void);

  /**
   * @brief Reset error state for Lift.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_State_Reset(void);

  /* =========================================================
   * Hardware Configuration APIs
   * ========================================================= */

  /**
   * @brief Lock brakes for specified axes of Arm0.
   *
   * @param axis_mask Bitmask of axes to lock.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Config_SetBrakeLock(unsigned char axis_mask);

  /**
   * @brief Unlock brakes for specified axes of Arm0.
   *
   * @param axis_mask Bitmask of axes to unlock.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Config_SetBrakeUnlock(unsigned char axis_mask);

  /**
   * @brief Reset single-turn encoder offset for specified axes of Arm0.
   *
   * @param axis_mask Bitmask of axes to reset.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Config_ResetEncSingleTurn(unsigned char axis_mask);

  /**
   * @brief Clear encoder errors for specified axes of Arm0.
   *
   * @param axis_mask Bitmask of axes to clear.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Config_ClearEncError(unsigned char axis_mask);

  /**
   * @brief Reset multi-turn encoder offset for specified axes of Arm0.
   *
   * @param axis_mask Bitmask of axes to reset.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Config_ResetEncMultiTurn(unsigned char axis_mask);

  /**
   * @brief Disable software joint limits for specified axes of Arm0.
   *
   * @param axis_mask Bitmask of axes to disable limits.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Config_DisableSoftLimit(unsigned char axis_mask);

  /**
   * @brief Set sensor offset for a specific axis of Arm0.
   *
   * @param axis_id Axis identifier.
   * @param offset Sensor offset value.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Config_SetSensorOffset(int axis_id,
                                                        int offset);

  /**
   * @brief Lock brakes for specified axes of Arm1.
   *
   * @param axis_mask Bitmask of axes to lock.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Config_SetBrakeLock(unsigned char axis_mask);

  /**
   * @brief Unlock brakes for specified axes of Arm1.
   *
   * @param axis_mask Bitmask of axes to unlock.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Config_SetBrakeUnlock(unsigned char axis_mask);

  /**
   * @brief Reset single-turn encoder offset for specified axes of Arm1.
   *
   * @param axis_mask Bitmask of axes to reset.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Config_ResetEncSingleTurn(unsigned char axis_mask);

  /**
   * @brief Clear encoder errors for specified axes of Arm1.
   *
   * @param axis_mask Bitmask of axes to clear.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Config_ClearEncError(unsigned char axis_mask);

  /**
   * @brief Reset multi-turn encoder offset for specified axes of Arm1.
   *
   * @param axis_mask Bitmask of axes to reset.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Config_ResetEncMultiTurn(unsigned char axis_mask);

  /**
   * @brief Disable software joint limits for specified axes of Arm1.
   *
   * @param axis_mask Bitmask of axes to disable limits.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Config_DisableSoftLimit(unsigned char axis_mask);

  /**
   * @brief Set sensor offset for a specific axis of Arm1.
   *
   * @param axis_id Axis identifier.
   * @param offset Sensor offset value.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Config_SetSensorOffset(int axis_id,
                                                        int offset);

  /**
   * @brief Lock brakes for specified axes of Head.
   *
   * @param axis_mask Bitmask of axes to lock.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Config_SetBrakeLock(unsigned char axis_mask);

  /**
   * @brief Unlock brakes for specified axes of Head.
   *
   * @param axis_mask Bitmask of axes to unlock.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Config_SetBrakeUnlock(unsigned char axis_mask);

  /**
   * @brief Reset single-turn encoder offset for specified axes of Head.
   *
   * @param axis_mask Bitmask of axes to reset.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Config_ResetEncSingleTurn(unsigned char axis_mask);

  /**
   * @brief Clear encoder errors for specified axes of Head.
   *
   * @param axis_mask Bitmask of axes to clear.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Config_ClearEncError(unsigned char axis_mask);

  /**
   * @brief Reset multi-turn encoder offset for specified axes of Head.
   *
   * @param axis_mask Bitmask of axes to reset.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Config_ResetEncMultiTurn(unsigned char axis_mask);

  /**
   * @brief Disable software joint limits for specified axes of Head.
   *
   * @param axis_mask Bitmask of axes to disable limits.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Config_DisableSoftLimit(unsigned char axis_mask);

  /**
   * @brief Lock brakes for specified axes of Body.
   *
   * @param axis_mask Bitmask of axes to lock.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Config_SetBrakeLock(unsigned char axis_mask);

  /**
   * @brief Unlock brakes for specified axes of Body.
   *
   * @param axis_mask Bitmask of axes to unlock.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Config_SetBrakeUnlock(unsigned char axis_mask);

  /**
   * @brief Reset single-turn encoder offset for specified axes of Body.
   *
   * @param axis_mask Bitmask of axes to reset.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Config_ResetEncSingleTurn(unsigned char axis_mask);

  /**
   * @brief Clear encoder errors for specified axes of Body.
   *
   * @param axis_mask Bitmask of axes to clear.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Config_ClearEncError(unsigned char axis_mask);

  /**
   * @brief Reset multi-turn encoder offset for specified axes of Body.
   *
   * @param axis_mask Bitmask of axes to reset.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Config_ResetEncMultiTurn(unsigned char axis_mask);

  /**
   * @brief Disable software joint limits for specified axes of Body.
   *
   * @param axis_mask Bitmask of axes to disable limits.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Config_DisableSoftLimit(unsigned char axis_mask);

  /**
   * @brief Set sensor offset for a specific axis of Body.
   *
   * @param axis_id Axis identifier.
   * @param offset Sensor offset value.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Config_SetSensorOffset(int axis_id,
                                                        int offset);

  /**
   * @brief Reset encoder offset for specified axes of Lift.
   *
   * @param axis_mask Bitmask of axes to reset.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Config_ResetEncOffset(unsigned char axis_mask);

  /**
   * @brief Disable software joint limits for specified axes of Lift.
   *
   * @param axis_mask Bitmask of axes to disable limits.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Config_DisableSoftLimit(unsigned char axis_mask);

  /* =========================================================
   * Runtime Motion Control APIs
   * ========================================================= */

  /**
   * @brief Trigger emergency stop for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_EmergencyStop(unsigned int thread_id);

  /**
   * @brief Set runtime state for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param state Target state identifier.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetState(unsigned int thread_id, int state);

  /**
   * @brief Set runtime user defined tag for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param tag User defined tag.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetTag(unsigned int thread_id, int tag);

  /**
   * @brief Send joint position command to Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param joint_pos Joint position command array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetJointPosCmd(unsigned int thread_id, double joint_pos[7]);

  /**
   * @brief Send joint torque command to Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param joint_tor Joint torque command array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetJointTorCmd(unsigned int thread_id, double joint_tor[7]);

  /**
   * @brief Apply force control parameters to Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param force_ctrl Force control definition array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetForceCtrl(unsigned int thread_id, double force_ctrl[5]);

  /**
   * @brief Apply torque control parameters to Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param torque_ctrl Torque control definition array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetTorqueCtrl(unsigned int thread_id, double torque_ctrl[5]);

  /**
   * @brief Apply command serial for PD mode to Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param cmd_serial command serial for PD mode.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetCmdPDSerial(unsigned int thread_id, int cmd_serial);

  /**
   * @brief Set velocity scaling ratio for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param vel_ratio Velocity scaling factor.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetVelRatio(unsigned int thread_id, double vel_ratio);

  /**
   * @brief Set acceleration scaling ratio for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param acc_ratio Acceleration scaling factor.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetAccRatio(unsigned int thread_id, double acc_ratio);

  /**
   * @brief Set joint stiffness coefficients for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param k Joint stiffness array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetJointK(unsigned int thread_id, double k[7]);

  /**
   * @brief Set joint damping coefficients for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param d Joint damping array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetJointD(unsigned int thread_id, double d[7]);

  /**
   * @brief Set Cartesian stiffness coefficients for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param k Cartesian stiffness array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetCartK(unsigned int thread_id, double k[7]);

  /**
   * @brief Set Cartesian damping coefficients for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param d Cartesian damping array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetCartD(unsigned int thread_id, double d[7]);

  /**
   * @brief Set tool kinematics parameters for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param k Tool kinematic parameters.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetToolK(unsigned int thread_id, double k[6]);

  /**
   * @brief Set tool dynamics parameters for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param d Tool dynamic parameters.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetToolD(unsigned int thread_id, double d[10]);

  /**
   * @brief Set impedance control type for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param imp_type Impedance control type identifier.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetImpType(unsigned int thread_id, int imp_type);

  /**
   * @brief Set drag teaching type for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param drag_type Drag teaching type identifier.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetDragType(unsigned int thread_id, int drag_type);

  /**
   * @brief Initialize trajectory buffer for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param point_num Number of trajectory points.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_InitTraj(unsigned int thread_id, int point_num);

  /**
   * @brief Set trajectory data for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param serial Trajectory segment serial number.
   * @param point_num Number of points in this segment.
   * @param point_data Pointer to trajectory point data.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetTraj(unsigned int thread_id, int serial, int point_num, double *point_data);

  /**
   * @brief Start trajectory execution for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_RunTraj(unsigned int thread_id);

  /**
   * @brief Stop trajectory execution for Arm0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm0_Runtime_StopTraj(unsigned int thread_id);

  /**
   * @brief Trigger emergency stop for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_EmergencyStop(unsigned int thread_id);

  /**
   * @brief Set runtime state for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param state Target state identifier.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetState(unsigned int thread_id, int state);

  /**
   * @brief Set runtime user defined tag for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param tag User defined tag.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetTag(unsigned int thread_id, int tag);

  /**
   * @brief Send joint position command to Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param joint_pos Joint position command array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetJointPosCmd(unsigned int thread_id, double joint_pos[7]);

  /**
   * @brief Send joint torque command to Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param joint_tor Joint torque command array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetJointTorCmd(unsigned int thread_id, double joint_tor[7]);

  /**
   * @brief Apply force control parameters to Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param force_ctrl Force control definition array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetForceCtrl(unsigned int thread_id, double force_ctrl[5]);

  /**
   * @brief Apply torque control parameters to Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param torque_ctrl Torque control definition array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetTorqueCtrl(unsigned int thread_id, double torque_ctrl[5]);

  /**
   * @brief Apply command serial for PD mode to Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param cmd_serial command serial for PD mode.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetCmdPDSerial(unsigned int thread_id, int cmd_serial);

  /**
   * @brief Set velocity scaling ratio for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param vel_ratio Velocity scaling factor.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetVelRatio(unsigned int thread_id, double vel_ratio);

  /**
   * @brief Set acceleration scaling ratio for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param acc_ratio Acceleration scaling factor.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetAccRatio(unsigned int thread_id, double acc_ratio);

  /**
   * @brief Set joint stiffness coefficients for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param k Joint stiffness array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetJointK(unsigned int thread_id, double k[7]);

  /**
   * @brief Set joint damping coefficients for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param d Joint damping array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetJointD(unsigned int thread_id, double d[7]);

  /**
   * @brief Set Cartesian stiffness coefficients for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param k Cartesian stiffness array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetCartK(unsigned int thread_id, double k[7]);

  /**
   * @brief Set Cartesian damping coefficients for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param d Cartesian damping array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetCartD(unsigned int thread_id, double d[7]);

  /**
   * @brief Set tool kinematics parameters for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param k Tool kinematic parameters.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetToolK(unsigned int thread_id, double k[6]);

  /**
   * @brief Set tool dynamics parameters for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param d Tool dynamic parameters.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetToolD(unsigned int thread_id, double d[10]);

  /**
   * @brief Set impedance control type for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param imp_type Impedance control type identifier.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetImpType(unsigned int thread_id, int imp_type);

  /**
   * @brief Set drag teaching type for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param drag_type Drag teaching type identifier.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetDragType(unsigned int thread_id, int drag_type);

  /**
   * @brief Initialize trajectory buffer for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param point_num Number of trajectory points.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_InitTraj(unsigned int thread_id, int point_num);

  /**
   * @brief Set trajectory data for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param serial Trajectory segment serial number.
   * @param point_num Number of points in this segment.
   * @param point_data Pointer to trajectory point data.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetTraj(unsigned int thread_id, int serial, int point_num, double *point_data);

  /**
   * @brief Start trajectory execution for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_RunTraj(unsigned int thread_id);

  /**
   * @brief Stop trajectory execution for Arm1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Arm1_Runtime_StopTraj(unsigned int thread_id);

  /**
   * @brief Trigger emergency stop for Head.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Runtime_EmergencyStop(unsigned int thread_id);

  /**
   * @brief Set runtime state for Head.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param state Target state identifier.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Runtime_SetState(unsigned int thread_id, int state);

  /**
   * @brief Set runtime user defined tag for Head.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param tag User defined tag.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Runtime_SetTag(unsigned int thread_id, int tag);

  /**
   * @brief Send joint position command to Head.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param joint_pos Joint position command array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Runtime_SetJointPosCmd(unsigned int thread_id, double joint_pos[3]);

  /**
   * @brief Set velocity scaling ratio for Head.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param vel_ratio Velocity scaling factor.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Runtime_SetVelRatio(unsigned int thread_id, double vel_ratio);

  /**
   * @brief Set acceleration scaling ratio for Head.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param acc_ratio Acceleration scaling factor.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Head_Runtime_SetAccRatio(unsigned int thread_id, double acc_ratio);

  /**
   * @brief Trigger emergency stop for Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_EmergencyStop(unsigned int thread_id);

  /**
   * @brief Set runtime state for Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param state Target state identifier.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_SetState(unsigned int thread_id, int state);

  /**
   * @brief Set runtime user defined tag for Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param tag User defined tag.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_SetTag(unsigned int thread_id, int tag);

  /**
   * @brief Send joint position command to Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param joint_pos Joint position command array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_SetJointPosCmd(unsigned int thread_id, double joint_pos[6]);

  /**
   * @brief Set velocity scaling ratio for Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param vel_ratio Velocity scaling factor.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_SetVelRatio(unsigned int thread_id, double vel_ratio);

  /**
   * @brief Set acceleration scaling ratio for Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param acc_ratio Acceleration scaling factor.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_SetAccRatio(unsigned int thread_id, double acc_ratio);

  /**
   * @brief Set proportional gains for Body PD control.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param p Proportional gain array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_SetPDP(unsigned int thread_id, double p[6]);

  /**
   * @brief Apply command serial for PD mode to Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param cmd_serial command serial for PD mode.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_SetCmdPDSerial(unsigned int thread_id, int cmd_serial);

  /**
   * @brief Set derivative gains for Body PD control.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param d Derivative gain array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_SetPDD(unsigned int thread_id, double d[6]);

  /**
   * @brief Initialize trajectory buffer for Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param point_num Number of trajectory points.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_InitTraj(unsigned int thread_id, int point_num);

  /**
   * @brief Set trajectory data for Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param serial Trajectory segment serial number.
   * @param point_num Number of points in this segment.
   * @param point_data Pointer to trajectory point data.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_SetTraj(unsigned int thread_id, int serial, int point_num, double *point_data);

  /**
   * @brief Start trajectory execution for Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_RunTraj(unsigned int thread_id);

  /**
   * @brief Stop trajectory execution for Body.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Body_Runtime_StopTraj(unsigned int thread_id);

  /**
   * @brief Trigger emergency stop for Lift.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Runtime_EmergencyStop(unsigned int thread_id);

  /**
   * @brief Set runtime state for Lift.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param state Target state identifier.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Runtime_SetState(unsigned int thread_id, int state);

  /**
   * @brief Set runtime user defined tag for Lift.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param tag User defined tag.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Runtime_SetTag(unsigned int thread_id, int tag);

  /**
   * @brief Send joint position command to Lift.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param joint_pos Joint position command array.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Runtime_SetJointPosCmd(unsigned int thread_id, double joint_pos[2]);

  /**
   * @brief Set velocity scaling ratio for Lift.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param vel_ratio Velocity scaling factor.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Runtime_SetVelRatio(unsigned int thread_id, double vel_ratio);

  /**
   * @brief Set acceleration scaling ratio for Lift.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param acc_ratio Acceleration scaling factor.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Runtime_SetAccRatio(unsigned int thread_id, double acc_ratio);

  /**
   * @brief Initialize trajectory buffer for Lift.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param point_num Number of trajectory points.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Runtime_InitTraj(unsigned int thread_id, int point_num);

  /**
   * @brief Set trajectory data for Lift.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param serial Trajectory segment serial number.
   * @param point_num Number of points in this segment.
   * @param point_data Pointer to trajectory point data.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Runtime_SetTraj(unsigned int thread_id, int serial, int point_num, double *point_data);

  /**
   * @brief Start trajectory execution for Lift.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Runtime_RunTraj(unsigned int thread_id);

  /**
   * @brief Stop trajectory execution for Lift.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Lift_Runtime_StopTraj(unsigned int thread_id);

  /**
   * @brief Set the runtime command action for Hand0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param[action_type Action to be applied to the hand module.
   *                      Refer to ::HandAction for valid values.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Hand0_Runtime_SetCmdAction(unsigned int thread_id, int action_type);

  /**
   * @brief Set the target position command for Hand0.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param pos Target position array for each joint.
   *                Array length must be 24.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Hand0_Runtime_SetCmdPos(unsigned int thread_id, int pos[24]);

  /**
   * @brief Set the proportional gain (P) for Hand0 runtime control.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param p Proportional gain array for each joint.
   *              Array length must be 24.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Hand0_Runtime_SetCmdP(unsigned int thread_id, int p[24]);

  /**
   * @brief Set the derivative gain (D) for Hand0 runtime control.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param d Derivative gain array for each joint.
   *              Array length must be 24.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Hand0_Runtime_SetCmdD(unsigned int thread_id, int d[24]);

  /**
   * @brief Set the maximum torque limit for Hand0 runtime control.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param max_tor Maximum torque array for each joint.
   *                   Array length must be 24.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Hand0_Runtime_SetCmdMaxTor(unsigned int thread_id, int max_tor[24]);

  /**
   * @brief Set the runtime command action for Hand1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param action_type Action to be applied to the hand module.
   *                      Refer to ::HandAction for valid values.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Hand1_Runtime_SetCmdAction(unsigned int thread_id, int action_type);

  /**
   * @brief Set the target position command for Hand1.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param pos Target position array for each joint.
   *                Array length must be 24.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Hand1_Runtime_SetCmdPos(unsigned int thread_id, int pos[24]);

  /**
   * @brief Set the proportional gain (P) for Hand1 runtime control.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param p Proportional gain array for each joint.
   *              Array length must be 24.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Hand1_Runtime_SetCmdP(unsigned int thread_id, int p[24]);

  /**
   * @brief Set the derivative gain (D) for Hand1 runtime control.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param d Derivative gain array for each joint.
   *              Array length must be 24.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Hand1_Runtime_SetCmdD(unsigned int thread_id, int d[24]);

  /**
   * @brief Set the maximum torque limit for Hand1 runtime control.
   *
   * @param thread_id Thread ID for this function call, range 0~7.
   * @param max_tor Maximum torque array for each joint.
   *                   Array length must be 24.
   *
   * @return 0 on success, -1 on failure.
   */
  CONTROL_SDK_API int FX_L0_Hand1_Runtime_SetCmdMaxTor(unsigned int thread_id, int max_tor[24]);

  /**
   * @brief Retrieve fast-group robot feedback data.
   *
   * @return Pointer to ROBOT_RT structure, or NULL on failure.
   */
  CONTROL_SDK_API const ROBOT_RT *FX_L0_GetRobotRT(void);

  /**
   * @brief Retrieve slow-group robot feedback data.
   *
   * @return Pointer to ROBOT_SG structure, or NULL on failure.
   */
  CONTROL_SDK_API const ROBOT_SG *FX_L0_GetRobotSG(void);

#ifdef __cplusplus
}
#endif

#endif /* FX_L0ROBOT_H_ */
