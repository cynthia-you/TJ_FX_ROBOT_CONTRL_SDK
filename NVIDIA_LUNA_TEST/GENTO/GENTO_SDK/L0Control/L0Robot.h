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
    /**
     * @brief Connect to a robot with the specified IP address
     *
     * This function establishes a network connection to a remote server. By passing the four octets
     * of an IPv4 address (in dotted-decimal notation), the function attempts to connect to the
     * target host. The success or failure of the connection is returned as a boolean value.
     *
     * @param ip1 First octet of the IP address (e.g., 192 in 192.168.1.2)
     * @param ip2 Second octet of the IP address (e.g., 168 in 192.168.1.2)
     * @param ip3 Third octet of the IP address (e.g., 1 in 192.168.1.2)
     * @param ip4 Fourth octet of the IP address (e.g., 2 in 192.168.1.2)
     *
     * @return int Connection result:
     *         - 1: Connection established successfully
     *         - 0: Connection failed
     */
    CONTROL_SDK_API int FX_L0_System_connect(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4);
    /**
     * @brief Test the network communication latency
     *
     * This function performs a test to verify the current network communication latency. It can be used
     * to check if the system is properly connected to the network.
     * The function returns a time delay in milliseconds indicating the test outcome.
     *
     * @return int Test communication delay:
     *         - Positive values: Test passed, indicate communication latency
     *         - Negative values: Communication is not built
     *
     * @note The test may take some time to complete depending on network conditions
     */
    CONTROL_SDK_API int FX_L0_System_Testconnect(void);
    /**
     * @brief Enable local logging functionality
     *
     * This function activates the local logging system, allowing the application to print
     * diagnostic information to stdout.
     */
    CONTROL_SDK_API void FX_L0_System_LocalLogOn(void);

    /**
     * @brief Disable local logging functionality
     *
     * This function deactivates the local logging system, not allowing the application to print
     * diagnostic information to stdout.
     */
    CONTROL_SDK_API void FX_L0_System_LocalLogOff(void);
    /**
     * @brief Clear the communication buffer and prepare for new communication
     *
     * This function clears the internal communication buffer and resets the communication state
     * to prepare for new data transmission or reception. It can be used to discard any pending
     * data and ensure a clean communication state.
     *
     * @param timeout Maximum time in milliseconds to wait for the clear operation to complete
     *                - 0: Non-blocking operation, returns immediately
     *                - >0: Block for specified timeout duration
     *
     * @return int Operation result:
     *         - 1: Communication cleared successfully
     *         - 0: Clear operation failed or timed out
     *
     * @see FX_L0_Communication_Send() Send data through the communication interface
     * @see FX_L0_Communication_SendWaitResponse() Send data through the communication interface
     * @see and wait until the data has been processed by controller.
     */
    CONTROL_SDK_API int FX_L0_Communication_Clear(unsigned int timeout);
    /**
     * @brief Send data through the communication interface
     *
     * This function transmits data that has been prepared in the send buffer through the
     * active communication interface. The actual data to be sent should be placed in the
     * appropriate buffer or structure before calling this function. The function handles
     * the transmission process and returns the result of the send operation.
     *
     * @return int Send operation result:
     *         - 1: Data sent successfully
     *         - 0: Send operation failed
     *
     * @note Data must be properly prepared in the send buffer before calling this function
     *
     * @see FX_L0_Communication_Clear() Clear the communication buffer and prepare for new communication
     * @see FX_L0_Communication_SendWaitResponse() Send data through the communication interface
     * and wait for a response.
     */
    CONTROL_SDK_API int FX_L0_Communication_Send(void);
    /**
     * @brief Send data through the communication interface and wait for a response
     *
     * This function sends data through the communication interface and then waits for a response
     * from the remote device. It combines the send operation with a synchronous wait for a reply.
     * The function returns when a response is received or when the specified timeout period elapses.
     *
     * @param time_out Maximum time in milliseconds to wait for a response after sending
     *                 - 0: No waiting, returns immediately after sending
     *                 - >0: Wait for specified timeout duration
     *
     * @return int Operation result:
     *         - Positive values: Response received successfully, indicating response time in milliseconds
     *         - 0: Response timeout
     *         - -1: Send operation failed
     *
     * @note Data must be prepared in the send buffer before calling this function
     * @note This function blocks the calling thread until response is received or timeout occurs
     * @note The timeout applies to the response wait period, not the send operation duration
     *
     * @see FX_L0_Communication_Clear() Clear the communication buffer and prepare for new communication
     * @see FX_L0_Communication_Send() Send data through the communication interface.
     */
    CONTROL_SDK_API int FX_L0_Communication_SendWaitResponse(unsigned int time_out);
    /**
     * @brief Get the version information of the system.
     *
     * This function returns the version number of the system. The version number
     * is typically composed of major, minor, and patch numbers encoded into a
     * 32-bit integer. The encoding scheme is implementation-specific.
     *
     * @return int The encoded system version number.
     *         - Positive values: represent valid version encodings.
     *         - -1: Execution error.
     *         - -2: Response is timeout.
     *
     * @note The exact format of the version encoding should be documented in the
     *       system specification. Common encodings include:
     *       - (major << 16) | (minor << 8) | patch
     */
    CONTROL_SDK_API int FX_L0_System_GetVersion(void);
    /**
     * @brief Reboots the system immediately.
     *
     * This function initiates a system reboot sequence. When called, the system
     * will begin shutdown procedures and restart. All unsaved data may be lost.
     * The function may not return if the reboot is successful.
     *
     * @return int Returns #1 if the reboot command was successfully
     *                  queued or initiated. Returns #0 if the system
     *                  cannot reboot due to hardware limitations, safety locks,
     *                  or other constraints. Note that a return of #1
     *                  does not guarantee successful reboot completion, only
     *                  that the reboot sequence was initiated.
     *
     * @warning Calling this function will cause the system to restart,
     *          potentially resulting in data loss. Ensure all critical
     *          operations are completed and data is saved before calling.
     *
     * @pre The system should be in a state where a reboot is safe.
     *
     * @post If successful, the system will restart. The function may not return
     *       control to the caller.
     */
    CONTROL_SDK_API int FX_L0_System_Reboot(void);
    /**
     * @brief Retrieves an integer parameter value by its name.
     *
     * This function looks up a system integer parameter from a
     * parameter store using the provided parameter name.
     * The retrieved value is stored in the location pointed to by ret_value.
     *
     * @param[in] name Null-terminated character array containing the parameter name.
     *                 Maximum length is 29 characters plus null terminator.
     *                 Parameter names are case-sensitive unless otherwise specified
     *                 by the system implementation.
     * @param[out] ret_value Pointer to an int variable where the parameter
     *                      value will be stored if found. The pointer must not be NULL.
     *
     * @return int Returns #1 if the parameter was successfully found
     *                  and its value retrieved. Returns #0 if the parameter
     *                  does not exist, the name is invalid.
     *
     * @warning The name buffer must be properly null-terminated. Passing a
     *          non-null-terminated string or a string longer than 29 characters
     *          (excluding null terminator) may cause undefined behavior.
     *
     * @pre The parameter with the specified name should exist in the parameter store.
     *      The ret_value pointer should point to a valid int memory location.
     *
     * @post If the function returns #1, *ret_value contains the parameter value.
     *       If the function returns #0, the content of *ret_value is unchanged.
     */
    CONTROL_SDK_API int FX_L0_Param_GetInt(char name[30], int *ret_value);
    /**
     * @brief Retrieves a floating-point parameter value by its name.
     *
     * This function looks up a system floating-point parameter from a
     * parameter store using the provided parameter name.
     * The retrieved value is stored in the location pointed to by ret_value.
     *
     * @param[in] name Null-terminated character array containing the parameter name.
     *                 Maximum length is 29 characters plus null terminator.
     *                 Parameter names are case-sensitive unless otherwise specified
     *                 by the system implementation.
     * @param[out] ret_value Pointer to an float variable where the parameter
     *                      value will be stored if found. The pointer must not be NULL.
     *
     * @return int Returns #1 if the parameter was successfully found
     *                  and its value retrieved. Returns #0 if the parameter
     *                  does not exist, the name is invalid.
     *
     * @warning The name buffer must be properly null-terminated. Passing a
     *          non-null-terminated string or a string longer than 29 characters
     *          (excluding null terminator) may cause undefined behavior.
     *
     * @pre The parameter with the specified name should exist in the parameter store.
     *      The ret_value pointer should point to a valid float memory location.
     *
     * @post If the function returns #1, *ret_value contains the parameter value.
     *       If the function returns #0, the content of *ret_value is unchanged.
     */
    CONTROL_SDK_API int FX_L0_Param_GetFloat(char name[30], float *ret_value);
    /**
     * @brief Sets an integer parameter to the specified value.
     *
     * This function sets the value of a system integer parameter
     * identified by the provided name. The new value is persisted to the parameter
     * store according to the system's implementation.
     *
     * @param[in] name Null-terminated character array containing the parameter name.
     *                 Maximum length is 29 characters plus null terminator.
     *                 Parameter names are case-sensitive unless otherwise specified
     *                 by the system implementation.
     * @param[in] target_value The new integer value to assign to the parameter.
     *
     * @return int Returns #1 if the parameter was successfully set
     *                  to the target value. Returns #0 if the operation
     *                  failed due to invalid name.
     *
     * @warning The name buffer must be properly null-terminated. Passing a
     *          non-null-terminated string or a string longer than 29 characters
     *          (excluding null terminator) may cause undefined behavior.
     *
     * @pre The calling process should have sufficient permissions to modify
     *      the parameter. Some parameters may be read-only or have restricted
     *      access.
     *
     * @post If the function returns #1, subsequent calls to
     *       FX_L0_Param_GetInt() with the same name will return target_value
     *       (or the persisted value if storage is immediate and successful).
     */
    CONTROL_SDK_API int FX_L0_Param_SetInt(char name[30], int target_value);
    /**
     * @brief Sets a floating-point parameter to the specified value.
     *
     * This function sets the value of a system floating-point parameter
     * identified by the provided name. The new value is persisted to the parameter
     * store according to the system's implementation.
     *
     * @param[in] name Null-terminated character array containing the parameter name.
     *                 Maximum length is 29 characters plus null terminator.
     *                 Parameter names are case-sensitive unless otherwise specified
     *                 by the system implementation.
     * @param[in] target_value The new floating-point value to assign to the parameter.
     *
     * @return int Returns #1 if the parameter was successfully set
     *                  to the target value. Returns #0 if the operation
     *                  failed due to invalid name, insufficient permissions,
     *                  write protection, storage failure, or other errors.
     *
     * @warning The name buffer must be properly null-terminated. Passing a
     *          non-null-terminated string or a string longer than 29 characters
     *          (excluding null terminator) may cause undefined behavior.
     *
     * @pre The calling process should have sufficient permissions to modify
     *      the parameter. Some parameters may be read-only or have restricted
     *      access.
     *
     * @post If the function returns #1, subsequent calls to
     *       FX_L0_Param_GetFloat() with the same name will return target_value
     *       (or the persisted value if storage is immediate and successful).
     */
    CONTROL_SDK_API int FX_L0_Param_SetFloat(char name[30], float target_value);
    /**
     * @brief Saves all modified parameters to persistent storage.
     *
     * This function flushes all pending parameter modifications (set via
     * FX_L0_Param_SetInt(), FX_L0_Param_SetFloat(), etc.) to the configuration file.
     * This ensures that parameter changes survive a system reset or power cycle.
     *
     * @return int Returns #1 if all parameters were successfully saved
     *                  to persistent storage. Returns #0 if the operation
     *                  failed due to storage errors, insufficient space, or
     *                  other system errors.
     *
     * @warning This function may be time-consuming depending on the storage medium
     *          and number of modified parameters. Avoid calling in time-critical
     *          sections of code.
     *
     * @pre There should be pending parameter modifications to save. Calling this
     *      function when no parameters have been modified may be a no-op.
     *
     * @post If the function returns #1, all parameter modifications are
     *       persisted and will survive system restart. If it returns #0,
     *       some or all modifications may be lost on restart.
     */
    CONTROL_SDK_API int FX_L0_Param_Save(void);
    /**
     * @brief Clears all data received from the Arm0 terminal.
     *
     * This function clears the content received from the Arm0 terminal device,
     * resetting it to a blank state.
     *
     * @return int Returns #1 if the terminal data was successfully
     *                  cleared. Returns #0 if the operation failed due
     *                  to hardware errors, invalid terminal state, or other
     *                  system errors.
     *
     * @see FX_L0_Arm0_Terminal_GetData(), FX_L0_Arm0_Terminal_SetData()
     */
    CONTROL_SDK_API int FX_L0_Arm0_Terminal_ClearData(void);
    /**
     * @brief Retrieves data from the Arm0 terminal input channel.
     *
     * This function reads available data from the Arm0 terminal input channel.
     * The function returns the number of bytes actually read and provides the channel type
     * and data through output parameters.
     *
     * @param[out] channel_type_ptr Pointer to FXChnType variable that will receive
     *                              the type of input channel from which data was
     *                              received. This parameter must not be NULL.
     *                              Valid types are defined in the FXChnType enumeration,
     *                              which typically includes:
     *                              - 1: CANFD channel
     *                              - 2: 485A channel
     *                              - 3: 485B channel
     * @param[out] data_ptr Byte array buffer where the received data will be stored.
     *                     The buffer must be at least 64 bytes in size. The data
     *                     is stored as raw bytes and may not be null-terminated.
     *                     This parameter must not be NULL.
     *
     * @return int The number of bytes actually read and stored in data_ptr.
     *                  Returns 0 if no data is currently available.
     *
     * @post If the return value is positive, channel_type_ptr contains the
     *       input channel type, and data_ptr contains the received data.
     *       The data in data_ptr is valid only up to the returned byte count.
     *
     * @see FX_L0_Arm0_Terminal_ClearData(), FX_L0_Arm0_Terminal_SetData()
     */
    CONTROL_SDK_API int FX_L0_Arm0_Terminal_GetData(int *channel_type_ptr, unsigned char data_ptr[64]);
    /**
     * @brief Sends data to the Arm0 terminal output channel.
     *
     * This function transmits data to the Arm0 terminal output channel specified
     * by channel_type. The data is sent to the appropriate output device.
     *
     * @param[in] channel_type The type of output channel to which data should be
     *                         sent. Valid types are defined in the FXChnType enumeration,
     *                         which typically includes:
     *                         - 1: CANFD channel
     *                         - 2: 485A channel
     *                         - 3: 485B channel
     * @param[in] data_ptr Pointer to the data buffer containing the bytes to send.
     *                    The data may contain text, binary data, or control
     *                    sequences. The pointer must not be NULL.
     * @param[in] data_len The number of bytes to send from data_ptr. Must be
     *                    non-negative. If zero, the function returns #1
     *                    with no data transmitted.
     *
     * @return int Returns #1 if the data was successfully queued or
     *                  transmitted to the specified channel. Returns #0
     *                  if the operation failed due to invalid parameters,
     *                  channel not available, buffer overflow, hardware error,
     *                  or other system errors.
     *
     * @warning The data_ptr must point to valid memory of at least data_len bytes.
     *          Passing an invalid pointer or negative data_len may cause undefined
     *          behavior. Some channels may have size limitations; sending data
     *          exceeding internal buffers may result in truncation or failure.
     *
     * @post If the function returns #1, the data has been accepted for
     *       transmission (though transmission may not be complete). If it returns
     *       #0, no data was transmitted.
     *
     * @see FX_L0_Arm0_Terminal_ClearData(), FX_L0_Arm0_Terminal_GetData()
     */
    CONTROL_SDK_API int FX_L0_Arm0_Terminal_SetData(int channel_type, unsigned char *data_ptr, int data_len);
    /**
     * @brief Clears all data received from the Arm1 terminal.
     *
     * This function clears the content received from the Arm1 terminal device,
     * resetting it to a blank state.
     *
     * @return int Returns #1 if the terminal data was successfully
     *                  cleared. Returns #0 if the operation failed due
     *                  to hardware errors, invalid terminal state, or other
     *                  system errors.
     *
     * @see FX_L0_Arm1_Terminal_GetData(), FX_L0_Arm1_Terminal_SetData()
     */
    CONTROL_SDK_API int FX_L0_Arm1_Terminal_ClearData(void);
    /**
     * @brief Retrieves data from the Arm1 terminal input channel.
     *
     * This function reads available data from the Arm1 terminal input channel.
     * The function returns the number of bytes actually read and provides the channel type
     * and data through output parameters.
     *
     * @param[out] channel_type_ptr Pointer to FXChnType variable that will receive
     *                              the type of input channel from which data was
     *                              received. This parameter must not be NULL.
     *                              Valid types are defined in the FXChnType enumeration,
     *                              which typically includes:
     *                              - 1: CANFD channel
     *                              - 2: 485A channel
     *                              - 3: 485B channel
     * @param[out] data_ptr Byte array buffer where the received data will be stored.
     *                     The buffer must be at least 64 bytes in size. The data
     *                     is stored as raw bytes and may not be null-terminated.
     *                     This parameter must not be NULL.
     *
     * @return int The number of bytes actually read and stored in data_ptr.
     *                  Returns 0 if no data is currently available.
     *
     * @post If the return value is positive, channel_type_ptr contains the
     *       input channel type, and data_ptr contains the received data.
     *       The data in data_ptr is valid only up to the returned byte count.
     *
     * @see FX_L0_Arm1_Terminal_ClearData(), FX_L0_Arm1_Terminal_SetData()
     */
    CONTROL_SDK_API int FX_L0_Arm1_Terminal_GetData(int *channel_type_ptr, unsigned char data_ptr[64]);
    /**
     * @brief Sends data to the Arm1 terminal output channel.
     *
     * This function transmits data to the Arm1 terminal output channel specified
     * by channel_type. The data is sent to the appropriate output device.
     *
     * @param[in] channel_type The type of output channel to which data should be
     *                         sent.Valid types are defined in the FXChnType enumeration,
     *                         which typically includes:
     *                         - 1: CANFD channel
     *                         - 2: 485A channel
     *                         - 3: 485B channel
     * @param[in] data_ptr Pointer to the data buffer containing the bytes to send.
     *                    The data may contain text, binary data, or control
     *                    sequences. The pointer must not be NULL.
     * @param[in] data_len The number of bytes to send from data_ptr. Must be
     *                    non-negative. If zero, the function returns #1
     *                    with no data transmitted.
     *
     * @return int Returns #1 if the data was successfully queued or
     *                  transmitted to the specified channel. Returns #0
     *                  if the operation failed due to invalid parameters,
     *                  channel not available, buffer overflow, hardware error,
     *                  or other system errors.
     *
     * @warning The data_ptr must point to valid memory of at least data_len bytes.
     *          Passing an invalid pointer or negative data_len may cause undefined
     *          behavior. Some channels may have size limitations; sending data
     *          exceeding internal buffers may result in truncation or failure.
     *
     * @post If the function returns #1, the data has been accepted for
     *       transmission (though transmission may not be complete). If it returns
     *       #0, no data was transmitted.
     *
     * @see FX_L0_Arm1_Terminal_ClearData(), FX_L0_Arm1_Terminal_GetData()
     */
    CONTROL_SDK_API int FX_L0_Arm1_Terminal_SetData(int channel_type, unsigned char *data_ptr, int data_len);
    /**
     * @brief Gets the error code for a specific servo axis on Arm0.
     *
     * This function retrieves the current error code from the servo motor
     * controller for the specified axis on Arm0. Servo error codes indicate
     * various fault conditions such as overcurrent, overheating, communication
     * errors, or positioning errors.
     *
     * @param[in] axis_id The identifier of the servo axis to query. Valid range
     *                   is implementation-dependent (typically 0 to 6 for
     *                   7 axes). Refer to hardware documentation for specific
     *                   axis numbering.
     * @param[out] error_code Pointer to an int variable that will receive
     *                       the error code. The pointer must not be NULL.
     *                       Error code values are system-specific; common values
     *                       include 0 (no error), positive numbers for errors.
     *
     * @return int Returns #1 if the error code was successfully
     *                  retrieved. Returns #0 if the operation failed
     *                  due to invalid axis_id, communication error.
     *
     * @post If the function returns #1, *error_code contains the current
     *       servo error code for the specified axis. If the function returns
     *       #0, the content of *error_code is unchanged.
     */
    CONTROL_SDK_API int FX_L0_Arm0_State_GetServoErrorCode(int axis_id, int *error_code);
    /**
     * @brief Gets the error code for a specific servo axis on Arm1.
     *
     * This function retrieves the current error code from the servo motor
     * controller for the specified axis on Arm1. Servo error codes indicate
     * various fault conditions such as overcurrent, overheating, communication
     * errors, or positioning errors.
     *
     * @param[in] axis_id The identifier of the servo axis to query. Valid range
     *                   is implementation-dependent (typically 0 to 6 for
     *                   7 axes). Refer to hardware documentation for specific
     *                   axis numbering.
     * @param[out] error_code Pointer to an int variable that will receive
     *                       the error code. The pointer must not be NULL.
     *                       Error code values are system-specific; common values
     *                       include 0 (no error), positive numbers for errors.
     *
     * @return int Returns #1 if the error code was successfully
     *                  retrieved. Returns #0 if the operation failed
     *                  due to invalid axis_id, communication error.
     *
     * @post If the function returns #1, *error_code contains the current
     *       servo error code for the specified axis. If the function returns
     *       #0, the content of *error_code is unchanged.
     */
    CONTROL_SDK_API int FX_L0_Arm1_State_GetServoErrorCode(int axis_id, int *error_code);
    /**
     * @brief Gets the error code for a specific servo axis on Head.
     *
     * This function retrieves the current error code from the servo motor
     * controller for the specified axis on Head. Servo error codes indicate
     * various fault conditions such as overcurrent, overheating, communication
     * errors, or positioning errors.
     *
     * @param[in] axis_id The identifier of the servo axis to query. Valid range
     *                   is implementation-dependent (typically 0 to 2 for
     *                   3 axes). Refer to hardware documentation for specific
     *                   axis numbering.
     * @param[out] error_code Pointer to an int variable that will receive
     *                       the error code. The pointer must not be NULL.
     *                       Error code values are system-specific; common values
     *                       include 0 (no error), positive numbers for errors.
     *
     * @return int Returns #1 if the error code was successfully
     *                  retrieved. Returns #0 if the operation failed
     *                  due to invalid axis_id, communication error.
     *
     * @post If the function returns #1, *error_code contains the current
     *       servo error code for the specified axis. If the function returns
     *       #0, the content of *error_code is unchanged.
     */
    CONTROL_SDK_API int FX_L0_Head_State_GetServoErrorCode(int axis_id, int *error_code);
    /**
     * @brief Gets the error code for a specific servo axis on Body.
     *
     * This function retrieves the current error code from the servo motor
     * controller for the specified axis on Body. Servo error codes indicate
     * various fault conditions such as overcurrent, overheating, communication
     * errors, or positioning errors.
     *
     * @param[in] axis_id The identifier of the servo axis to query. Valid range
     *                   is implementation-dependent (typically 0 to 5 for
     *                   6 axes). Refer to hardware documentation for specific
     *                   axis numbering.
     * @param[out] error_code Pointer to an int variable that will receive
     *                       the error code. The pointer must not be NULL.
     *                       Error code values are system-specific; common values
     *                       include 0 (no error), positive numbers for errors.
     *
     * @return int Returns #1 if the error code was successfully
     *                  retrieved. Returns #0 if the operation failed
     *                  due to invalid axis_id, communication error.
     *
     * @post If the function returns #1, *error_code contains the current
     *       servo error code for the specified axis. If the function returns
     *       #0, the content of *error_code is unchanged.
     */
    CONTROL_SDK_API int FX_L0_Body_State_GetServoErrorCode(int axis_id, int *error_code);
    /**
     * @brief Gets the error code for a specific servo axis on Lift.
     *
     * This function retrieves the current error code from the servo motor
     * controller for the specified axis on Lift. Servo error codes indicate
     * various fault conditions such as overcurrent, overheating, communication
     * errors, or positioning errors.
     *
     * @param[in] axis_id The identifier of the servo axis to query. Valid range
     *                   is implementation-dependent (typically 0 to 1 for
     *                   2 axes). Refer to hardware documentation for specific
     *                   axis numbering.
     * @param[out] error_code Pointer to an int variable that will receive
     *                       the error code. The pointer must not be NULL.
     *                       Error code values are system-specific; common values
     *                       include 0 (no error), positive numbers for errors.
     *
     * @return int Returns #1 if the error code was successfully
     *                  retrieved. Returns #0 if the operation failed
     *                  due to invalid axis_id, communication error.
     *
     * @post If the function returns #1, *error_code contains the current
     *       servo error code for the specified axis. If the function returns
     *       #0, the content of *error_code is unchanged.
     */
    CONTROL_SDK_API int FX_L0_Lift_State_GetServoErrorCode(int axis_id, int *error_code);
    /**
     * @brief Clear error of all Arm0 components and transfer Arm0 to IDLE state.
     *
     * This function performs a soft reset of the Arm0 subsystem, clearing
     * all Arm0 servos errros and transfering Arm0 to IDLE state.
     *
     * @return int Returns #1 if the Arm0 state was successfully reset.
     *                  Returns #0 if the reset operation failed due to
     *                  hardware errors, critical faults, or because Arm0 is
     *                  currently in an unrecoverable state.
     *
     * @note This is a soft reset, not a power cycle. Some components may retain
     *       state information depending on hardware design. Persistent parameters
     *       may not be affected unless explicitly cleared by the reset.
     *
     * @warning This function will interrupt any ongoing Arm0 operations.
     *          All active movements, communications, and processes on Arm0
     *          will be terminated. Ensure Arm0 is in a safe state before calling.
     *
     * @pre Arm0 should be in a state where a reset is safe. Any critical
     *      operations should be completed or suspended before calling.
     */
    CONTROL_SDK_API int FX_L0_Arm0_State_Reset(void);
    /**
     * @brief Clear error of all Arm1 components and transfer Arm1 to IDLE state.
     *
     * This function performs a soft reset of the Arm1 subsystem, clearing
     * all Arm1 servos errros and transfering Arm0 to IDLE state.
     *
     * @return int Returns #1 if the Arm0 state was successfully reset.
     *                  Returns #0 if the reset operation failed due to
     *                  hardware errors, critical faults, or because Arm0 is
     *                  currently in an unrecoverable state.
     *
     * @note This is a soft reset, not a power cycle. Some components may retain
     *       state information depending on hardware design. Persistent parameters
     *       may not be affected unless explicitly cleared by the reset.
     *
     * @warning This function will interrupt any ongoing Arm1 operations.
     *          All active movements, communications, and processes on Arm0
     *          will be terminated. Ensure Arm1 is in a safe state before calling.
     *
     * @pre Arm1 should be in a state where a reset is safe. Any critical
     *      operations should be completed or suspended before calling.
     */
    CONTROL_SDK_API int FX_L0_Arm1_State_Reset(void);
    /**
     * @brief Clear error of all Head components and transfer Head to IDLE state.
     *
     * This function performs a soft reset of the Head subsystem, clearing
     * all Head servos errros and transfering Head to IDLE state.
     *
     * @return int Returns #1 if the Head state was successfully reset.
     *                  Returns #0 if the reset operation failed due to
     *                  hardware errors, critical faults, or because Head is
     *                  currently in an unrecoverable state.
     *
     * @note This is a soft reset, not a power cycle. Some components may retain
     *       state information depending on hardware design. Persistent parameters
     *       may not be affected unless explicitly cleared by the reset.
     *
     * @warning This function will interrupt any ongoing Head operations.
     *          All active movements, communications, and processes on Head
     *          will be terminated. Ensure Head is in a safe state before calling.
     *
     * @pre Head should be in a state where a reset is safe. Any critical
     *      operations should be completed or suspended before calling.
     */
    CONTROL_SDK_API int FX_L0_Head_State_Reset(void);
    /**
     * @brief Clear error of all Body components and transfer Body to IDLE state.
     *
     * This function performs a soft reset of the Body subsystem, clearing
     * all Body servos errros and transfering Body to IDLE state.
     *
     * @return int Returns #1 if the Body state was successfully reset.
     *                  Returns #0 if the reset operation failed due to
     *                  hardware errors, critical faults, or because Body is
     *                  currently in an unrecoverable state.
     *
     * @note This is a soft reset, not a power cycle. Some components may retain
     *       state information depending on hardware design. Persistent parameters
     *       may not be affected unless explicitly cleared by the reset.
     *
     * @warning This function will interrupt any ongoing Body operations.
     *          All active movements, communications, and processes on Body
     *          will be terminated. Ensure Body is in a safe state before calling.
     *
     * @pre Body should be in a state where a reset is safe. Any critical
     *      operations should be completed or suspended before calling.
     */
    CONTROL_SDK_API int FX_L0_Body_State_Reset(void);
    /**
     * @brief Clear error of all Lift components and transfer Lift to IDLE state.
     *
     * This function performs a soft reset of the Lift subsystem, clearing
     * all Lift servos errros and transfering Lift to IDLE state.
     *
     * @return int Returns #1 if the Lift state was successfully reset.
     *                  Returns #0 if the reset operation failed due to
     *                  hardware errors, critical faults, or because Lift is
     *                  currently in an unrecoverable state.
     *
     * @note This is a soft reset, not a power cycle. Some components may retain
     *       state information depending on hardware design. Persistent parameters
     *       may not be affected unless explicitly cleared by the reset.
     *
     * @warning This function will interrupt any ongoing Lift operations.
     *          All active movements, communications, and processes on Lift
     *          will be terminated. Ensure Lift is in a safe state before calling.
     *
     * @pre Lift should be in a state where a reset is safe. Any critical
     *      operations should be completed or suspended before calling.
     */
    CONTROL_SDK_API int FX_L0_Lift_State_Reset(void);
    /**
     * @brief Locks the brakes on specified axes of Arm0.
     *
     * This function engages (locks) the brakes on one or more axes of Arm0
     * as specified by the axis mask. Brake locking prevents movement on the
     * affected axes, typically used for safety during maintenance or to
     * maintain position when power is removed.
     *
     * @param[in] axis_mask Bitmask specifying which axes' brakes to lock.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to lock
     *                      the brake on that axis, 0 to leave unchanged.
     *                      Example: 0x05 (binary 00000101) locks brakes
     *                      on axes 0 and 2.
     *
     * @return int Returns #1 if the brake lock command was successfully
     *                  sent to all specified axes. Returns #0 if the
     *                  operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, hardware faults.
     *
     * @warning Locking brakes while the axis is moving may cause abrupt stops
     *          and mechanical stress. Ensure axes are stationary before locking
     *          brakes. Some systems may prevent brake locking during motion.
     *
     * @pre Axes may need to be disabled or powered down
     *      before brakes can be locked.
     *
     * @see FX_L0_Arm0_Config_SetBrakeUnlock()
     */
    CONTROL_SDK_API int FX_L0_Arm0_Config_SetBrakeLock(unsigned char axis_mask);
    /**
     * @brief Unlocks (releases) the brakes on specified axes of Arm0.
     *
     * This function disengages (unlocks) the brakes on one or more axes of Arm0
     * as specified by the axis mask. Brake unlocking allows movement on the
     * affected axes, typically done after maintenance or before normal operation.
     *
     * @param[in] axis_mask Bitmask specifying which axes' brakes to unlock.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to unlock
     *                      the brake on that axis, 0 to leave unchanged.
     *                      Example: 0x0A (binary 00001010) unlocks brakes
     *                      on axes 1 and 3.
     *
     * @return int Returns #1 if the brake unlock command was successfully
     *                  sent to all specified axes. Returns #0 if the
     *                  operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, hardware faults.
     *
     * @note After unlocking brakes, axes may be in a "free" state where they
     *       can move freely (especially under gravity). Ensure proper support
     *       or enable servo power before unlocking brakes on vertical axes.
     *
     * @warning Unlocking brakes on unpowered or unsupported axes may cause
     *          unexpected movement, especially for vertical axes that are
     *          counteracting gravity. Take appropriate safety precautions.
     *
     * @pre The system should be in a safe state for brake release. For vertical
     *      axes, ensure servo power is enabled or mechanical support is in place
     *      before unlocking brakes to prevent uncontrolled movement.
     *
     * @see FX_L0_Arm0_Config_SetBrakeLock()
     */
    CONTROL_SDK_API int FX_L0_Arm0_Config_SetBrakeUnlock(unsigned char axis_mask);
    /**
     * @brief Resets the encoder offset for specified axes of Arm0.
     *
     * This function resets the encoder offset (sometimes called "zero position"
     * or "home offset") for one or more axes of Arm0 as specified by the axis mask.
     * This is typically used during calibration or homing procedures to establish
     * a new reference position for the encoders.
     *
     * @param[in] axis_mask Bitmask specifying which axes' encoder offsets to reset.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to reset the
     *                      encoder offset on that axis, 0 to leave unchanged.
     *                      Example: 0x07 (binary 00000111) resets offsets
     *                      on axes 0, 1, and 2.
     *
     * @return int Returns #1 if the encoder offset reset was successful
     *                  for all specified axes. Returns #0 if the operation
     *                  failed on one or more axes due to invalid axis_mask,
     *                  communication errors, hardware faults, or because the
     *                  axis is currently moving or in a fault state.
     *
     * @note Resetting encoder offsets will cause the current position to be
     *       treated as the new reference (usually zero). This affects all
     *       subsequent position commands and feedback. Use with caution.
     *
     * @warning This operation can cause significant positioning errors if
     *          performed incorrectly. Ensure the robot is in a known,
     *          calibrated position before resetting encoder offsets.
     *          Improper use may require complete recalibration.
     *
     * @pre The specified axes should be stationary and in a known reference
     *      position. The system should be in a calibration or maintenance mode.
     *
     * @post If successful, the encoder offsets for specified axes are reset,
     *       making the current position the new reference. All subsequent
     *       position readings and commands will be relative to this new zero.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Config_ResetEncOffset(unsigned char axis_mask);
    /**
     * @brief Clears encoder errors for specified axes of Arm0.
     *
     * This function clears error flags and resets error states for the encoders
     * on one or more axes of Arm0 as specified by the axis mask. This is used
     * to recover from encoder-related errors such as communication faults,
     * count errors, or hardware errors.
     *
     * @param[in] axis_mask Bitmask specifying which axes' encoder errors to clear.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to clear
     *                      encoder errors on that axis, 0 to leave unchanged.
     *                      Example: 0x0F (binary 00001111) clears errors
     *                      on axes 0, 1, 2, and 3.
     *
     * @return int Returns #1 if encoder errors were successfully
     *                  cleared for all specified axes. Returns #0 if
     *                  the operation failed on one or more axes due to
     *                  invalid axis_mask, communication errors, or persistent
     *                  hardware faults that cannot be cleared.
     *
     * @note Clearing encoder errors does not fix underlying hardware problems.
     *       If errors persist after clearing, there may be a hardware issue
     *       requiring maintenance. Some errors may reoccur immediately if
     *       the root cause is not addressed.
     *
     * @pre The axes should be in a safe state (stopped) before clearing
     *      encoder errors. Clearing errors during motion may cause
     *      unpredictable behavior.
     *
     * @post If successful, encoder error flags for the specified axes are
     *       cleared. The encoders are reset to a normal operating state
     *       if the underlying issue is resolved. Position tracking may
     *       be lost and require re-homing.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Config_ClearEncError(unsigned char axis_mask);
    /**
     * @brief Resets the external encoder offset for specified axes of the Arm0 module.
     *
     * This function resets the external encoder offset (zero position reference)
     * for one or more axes of the Arm0 module as specified by the axis mask.
     * External encoders are typically auxiliary position sensors separate from
     * the main motor encoders, used for redundant position verification or
     * secondary feedback.
     *
     * @param[in] axis_mask Bitmask specifying which axes' external encoder offsets to reset.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to reset the
     *                      external encoder offset on that axis, 0 to leave unchanged.
     *                      Example: 0x03 (binary 00000011) resets external encoder
     *                      offsets on axes 0 and 1 of the Arm0 module.
     *
     * @return int Returns #1 if the external encoder offset reset was
     *                  successful for all specified axes. Returns #0 if
     *                  the operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, the axis doesn't have
     *                  an external encoder, or because the axis is in a fault state.
     *
     * @note External encoders are often used for safety verification or as
     *       backup position sensors. Resetting their offsets should be done
     *       carefully to maintain consistency with the primary encoders.
     *
     * @warning Resetting external encoder offsets independently of primary
     *          encoders can cause disagreement between position feedback
     *          sources, potentially triggering safety faults. Ensure the
     *          Arm0 is in a known reference position before resetting.
     *
     * @pre The Arm0 module should be in a known, calibrated position. The
     *      axes should be stationary. The system should be in a calibration
     *      or maintenance mode.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Config_ResetExtEncOffset(unsigned char axis_mask);
    /**
     * @brief Disables the software position limits for specified axes of Arm0.
     *
     * This function disables the software-imposed position limits (soft limits)
     * for one or more axes of Arm0 as specified by the axis mask. Soft limits
     * are safety features that prevent the axes from moving beyond predefined
     * safe positions. Disabling them should only be done for maintenance,
     * calibration, or recovery procedures.
     *
     * @param[in] axis_mask Bitmask specifying which axes' soft limits to disable.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to disable
     *                      soft limits on that axis, 0 to enable limits.
     *                      Example: 0x1F (binary 00011111) disables soft limits
     *                      on axes 0, 1, 2, 3, and 4.
     *
     * @return int Returns #1 if soft limits were successfully disabled
     *                  for all specified axes. Returns #0 if the operation
     *                  failed on one or more axes due to invalid axis_mask,
     *                  communication errors, hardware faults, or because the
     *                  axis is in a state that prevents limit disabling
     *                  (e.g., during motion).
     *
     * @note When soft limits are disabled, the axes can move beyond their normal
     *       operational ranges, which may cause collisions or damage. This operation
     *       is only allowed in IDLE state and will be set enable automatically
     *       when Arm0 transfer to IDLE/TORQUE/RELEASE state next time.
     *
     * @warning Disabling soft limits removes an important safety feature.
     *          Only disable soft limits when absolutely necessary and with
     *          proper safety precautions. Never leave axes with soft limits
     *          disabled during normal operation.
     *
     * @pre The axes should be stationary. The operator should be aware of
     *      the risks and be ready to intervene. Consider disabling only
     *      the specific axes needed for the procedure.
     *
     * @post If successful, soft limits for the specified axes are disabled.
     *       The axes can now move beyond their normal limits until soft
     *       limits are re-enabled or the system is reset.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Config_DisableSoftLimit(unsigned char axis_mask);
    /**
     * @brief Sets the sensor offset for a specific axis of Arm0.
     *
     * This function sets a calibration offset value for a sensor on a specified
     * axis of Arm0. The offset is added to the raw sensor reading to compensate
     * for sensor misalignment, manufacturing tolerances, or to establish a
     * known zero reference.
     *
     * @param[in] axis_id The identifier of the axis whose sensor offset will be set.
     *                    Valid range is implementation-dependent (typically 0 to 6
     *                    for 7 axes). Refer to hardware documentation for specific
     *                    axis numbering.
     * @param[in] offset The offset value to apply to the sensor readings, in sensor
     *                   counts or units. Positive values shift readings upward,
     *                   negative values shift downward. The exact unit depends on
     *                   the sensor type (e.g., encoder counts, resolver bits).
     *
     * @return int Returns #1 if the sensor offset was successfully set.
     *                  Returns #0 if the operation failed due to invalid
     *                  axis_id, communication error, the sensor doesn't support
     *                  offset adjustment, or the offset value is out of range.
     *
     * @note The offset is typically stored in non-volatile memory and persists
     *       across power cycles. Some sensors may have separate offsets for
     *       different operating modes (e.g., absolute vs. incremental).
     *
     * @warning Incorrect sensor offsets can cause positioning errors, safety
     *          faults, or system damage. Set offsets only during calibration
     *          procedures by trained personnel. Always verify the new offset
     *          with known reference positions.
     *
     * @pre The axis should be stationary and in a known reference position.
     *      The sensor should be properly initialized and functional.
     *
     * @post If successful, subsequent sensor readings on the specified axis
     *       will have the offset applied. The offset is typically stored
     *       persistently and will be used after system restart.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Config_SetSensorOffset(int axis_id, int offset);
    /**
     * @brief Locks the brakes on specified axes of Arm1.
     *
     * This function engages (locks) the brakes on one or more axes of Arm1
     * as specified by the axis mask. Brake locking prevents movement on the
     * affected axes, typically used for safety during maintenance or to
     * maintain position when power is removed.
     *
     * @param[in] axis_mask Bitmask specifying which axes' brakes to lock.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to lock
     *                      the brake on that axis, 0 to leave unchanged.
     *                      Example: 0x05 (binary 00000101) locks brakes
     *                      on axes 0 and 2.
     *
     * @return int Returns #1 if the brake lock command was successfully
     *                  sent to all specified axes. Returns #0 if the
     *                  operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, hardware faults.
     *
     * @warning Locking brakes while the axis is moving may cause abrupt stops
     *          and mechanical stress. Ensure axes are stationary before locking
     *          brakes. Some systems may prevent brake locking during motion.
     *
     * @pre Axes may need to be disabled or powered down
     *      before brakes can be locked.
     *
     * @see FX_L0_Arm1_Config_SetBrakeUnlock()
     */
    CONTROL_SDK_API int FX_L0_Arm1_Config_SetBrakeLock(unsigned char axis_mask);
    /**
     * @brief Unlocks (releases) the brakes on specified axes of Arm1.
     *
     * This function disengages (unlocks) the brakes on one or more axes of Arm1
     * as specified by the axis mask. Brake unlocking allows movement on the
     * affected axes, typically done after maintenance or before normal operation.
     *
     * @param[in] axis_mask Bitmask specifying which axes' brakes to unlock.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to unlock
     *                      the brake on that axis, 0 to leave unchanged.
     *                      Example: 0x0A (binary 00001010) unlocks brakes
     *                      on axes 1 and 3.
     *
     * @return int Returns #1 if the brake unlock command was successfully
     *                  sent to all specified axes. Returns #0 if the
     *                  operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, hardware faults.
     *
     * @note After unlocking brakes, axes may be in a "free" state where they
     *       can move freely (especially under gravity). Ensure proper support
     *       or enable servo power before unlocking brakes on vertical axes.
     *
     * @warning Unlocking brakes on unpowered or unsupported axes may cause
     *          unexpected movement, especially for vertical axes that are
     *          counteracting gravity. Take appropriate safety precautions.
     *
     * @pre The system should be in a safe state for brake release. For vertical
     *      axes, ensure servo power is enabled or mechanical support is in place
     *      before unlocking brakes to prevent uncontrolled movement.
     *
     * @see FX_L0_Arm1_Config_SetBrakeLock()
     */
    CONTROL_SDK_API int FX_L0_Arm1_Config_SetBrakeUnlock(unsigned char axis_mask);
    /**
     * @brief Resets the encoder offset for specified axes of Arm1.
     *
     * This function resets the encoder offset (sometimes called "zero position"
     * or "home offset") for one or more axes of Arm1 as specified by the axis mask.
     * This is typically used during calibration or homing procedures to establish
     * a new reference position for the encoders.
     *
     * @param[in] axis_mask Bitmask specifying which axes' encoder offsets to reset.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to reset the
     *                      encoder offset on that axis, 0 to leave unchanged.
     *                      Example: 0x07 (binary 00000111) resets offsets
     *                      on axes 0, 1, and 2.
     *
     * @return int Returns #1 if the encoder offset reset was successful
     *                  for all specified axes. Returns #0 if the operation
     *                  failed on one or more axes due to invalid axis_mask,
     *                  communication errors, hardware faults, or because the
     *                  axis is currently moving or in a fault state.
     *
     * @note Resetting encoder offsets will cause the current position to be
     *       treated as the new reference (usually zero). This affects all
     *       subsequent position commands and feedback. Use with caution.
     *
     * @warning This operation can cause significant positioning errors if
     *          performed incorrectly. Ensure the robot is in a known,
     *          calibrated position before resetting encoder offsets.
     *          Improper use may require complete recalibration.
     *
     * @pre The specified axes should be stationary and in a known reference
     *      position. The system should be in a calibration or maintenance mode.
     *
     * @post If successful, the encoder offsets for specified axes are reset,
     *       making the current position the new reference. All subsequent
     *       position readings and commands will be relative to this new zero.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Config_ResetEncOffset(unsigned char axis_mask);
    /**
     * @brief Clears encoder errors for specified axes of Arm1.
     *
     * This function clears error flags and resets error states for the encoders
     * on one or more axes of Arm1 as specified by the axis mask. This is used
     * to recover from encoder-related errors such as communication faults,
     * count errors, or hardware errors.
     *
     * @param[in] axis_mask Bitmask specifying which axes' encoder errors to clear.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to clear
     *                      encoder errors on that axis, 0 to leave unchanged.
     *                      Example: 0x0F (binary 00001111) clears errors
     *                      on axes 0, 1, 2, and 3.
     *
     * @return int Returns #1 if encoder errors were successfully
     *                  cleared for all specified axes. Returns #0 if
     *                  the operation failed on one or more axes due to
     *                  invalid axis_mask, communication errors, or persistent
     *                  hardware faults that cannot be cleared.
     *
     * @note Clearing encoder errors does not fix underlying hardware problems.
     *       If errors persist after clearing, there may be a hardware issue
     *       requiring maintenance. Some errors may reoccur immediately if
     *       the root cause is not addressed.
     *
     * @pre The axes should be in a safe state (stopped) before clearing
     *      encoder errors. Clearing errors during motion may cause
     *      unpredictable behavior.
     *
     * @post If successful, encoder error flags for the specified axes are
     *       cleared. The encoders are reset to a normal operating state
     *       if the underlying issue is resolved. Position tracking may
     *       be lost and require re-homing.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Config_ClearEncError(unsigned char axis_mask);
    /**
     * @brief Resets the external encoder offset for specified axes of the Arm1 module.
     *
     * This function resets the external encoder offset (zero position reference)
     * for one or more axes of the Arm1 module as specified by the axis mask.
     * External encoders are typically auxiliary position sensors separate from
     * the main motor encoders, used for redundant position verification or
     * secondary feedback.
     *
     * @param[in] axis_mask Bitmask specifying which axes' external encoder offsets to reset.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to reset the
     *                      external encoder offset on that axis, 0 to leave unchanged.
     *                      Example: 0x03 (binary 00000011) resets external encoder
     *                      offsets on axes 0 and 1 of the Arm1 module.
     *
     * @return int Returns #1 if the external encoder offset reset was
     *                  successful for all specified axes. Returns #0 if
     *                  the operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, the axis doesn't have
     *                  an external encoder, or because the axis is in a fault state.
     *
     * @note External encoders are often used for safety verification or as
     *       backup position sensors. Resetting their offsets should be done
     *       carefully to maintain consistency with the primary encoders.
     *
     * @warning Resetting external encoder offsets independently of primary
     *          encoders can cause disagreement between position feedback
     *          sources, potentially triggering safety faults. Ensure the
     *          Arm1 is in a known reference position before resetting.
     *
     * @pre The Arm1 module should be in a known, calibrated position. The
     *      axes should be stationary. The system should be in a calibration
     *      or maintenance mode.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Config_ResetExtEncOffset(unsigned char axis_mask);
    /**
     * @brief Disables the software position limits for specified axes of Arm1.
     *
     * This function disables the software-imposed position limits (soft limits)
     * for one or more axes of Arm1 as specified by the axis mask. Soft limits
     * are safety features that prevent the axes from moving beyond predefined
     * safe positions. Disabling them should only be done for maintenance,
     * calibration, or recovery procedures.
     *
     * @param[in] axis_mask Bitmask specifying which axes' soft limits to disable.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to disable
     *                      soft limits on that axis, 0 to enable limits.
     *                      Example: 0x1F (binary 00011111) disables soft limits
     *                      on axes 0, 1, 2, 3, and 4.
     *
     * @return int Returns #1 if soft limits were successfully disabled
     *                  for all specified axes. Returns #0 if the operation
     *                  failed on one or more axes due to invalid axis_mask,
     *                  communication errors, hardware faults, or because the
     *                  axis is in a state that prevents limit disabling
     *                  (e.g., during motion).
     *
     * @note When soft limits are disabled, the axes can move beyond their normal
     *       operational ranges, which may cause collisions or damage. This operation
     *       is only allowed in IDLE state and will be set enable automatically
     *       when Arm1 transfer to IDLE/TORQUE/RELEASE state next time.
     *
     * @warning Disabling soft limits removes an important safety feature.
     *          Only disable soft limits when absolutely necessary and with
     *          proper safety precautions. Never leave axes with soft limits
     *          disabled during normal operation.
     *
     * @pre The axes should be stationary. The operator should be aware of
     *      the risks and be ready to intervene. Consider disabling only
     *      the specific axes needed for the procedure.
     *
     * @post If successful, soft limits for the specified axes are disabled.
     *       The axes can now move beyond their normal limits until soft
     *       limits are re-enabled or the system is reset.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Config_DisableSoftLimit(unsigned char axis_mask);
    /**
     * @brief Sets the sensor offset for a specific axis of Arm1.
     *
     * This function sets a calibration offset value for a sensor on a specified
     * axis of Arm1. The offset is added to the raw sensor reading to compensate
     * for sensor misalignment, manufacturing tolerances, or to establish a
     * known zero reference.
     *
     * @param[in] axis_id The identifier of the axis whose sensor offset will be set.
     *                    Valid range is implementation-dependent (typically 0 to 6
     *                    for 7 axes). Refer to hardware documentation for specific
     *                    axis numbering.
     * @param[in] offset The offset value to apply to the sensor readings, in sensor
     *                   counts or units. Positive values shift readings upward,
     *                   negative values shift downward. The exact unit depends on
     *                   the sensor type (e.g., encoder counts, resolver bits).
     *
     * @return int Returns #1 if the sensor offset was successfully set.
     *                  Returns #0 if the operation failed due to invalid
     *                  axis_id, communication error, the sensor doesn't support
     *                  offset adjustment, or the offset value is out of range.
     *
     * @note The offset is typically stored in non-volatile memory and persists
     *       across power cycles. Some sensors may have separate offsets for
     *       different operating modes (e.g., absolute vs. incremental).
     *
     * @warning Incorrect sensor offsets can cause positioning errors, safety
     *          faults, or system damage. Set offsets only during calibration
     *          procedures by trained personnel. Always verify the new offset
     *          with known reference positions.
     *
     * @pre The axis should be stationary and in a known reference position.
     *      The sensor should be properly initialized and functional.
     *
     * @post If successful, subsequent sensor readings on the specified axis
     *       will have the offset applied. The offset is typically stored
     *       persistently and will be used after system restart.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Config_SetSensorOffset(int axis_id, int offset);
    /**
     * @brief Locks the brakes on specified axes of Head.
     *
     * This function engages (locks) the brakes on one or more axes of Head
     * as specified by the axis mask. Brake locking prevents movement on the
     * affected axes, typically used for safety during maintenance or to
     * maintain position when power is removed.
     *
     * @param[in] axis_mask Bitmask specifying which axes' brakes to lock.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to lock
     *                      the brake on that axis, 0 to leave unchanged.
     *                      Example: 0x05 (binary 00000101) locks brakes
     *                      on axes 0 and 2.
     *
     * @return int Returns #1 if the brake lock command was successfully
     *                  sent to all specified axes. Returns #0 if the
     *                  operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, hardware faults.
     *
     * @warning Locking brakes while the axis is moving may cause abrupt stops
     *          and mechanical stress. Ensure axes are stationary before locking
     *          brakes. Some systems may prevent brake locking during motion.
     *
     * @pre Axes may need to be disabled or powered down
     *      before brakes can be locked.
     *
     * @see FX_L0_Head_Config_SetBrakeUnlock()
     */
    CONTROL_SDK_API int FX_L0_Head_Config_SetBrakeLock(unsigned char axis_mask);
    /**
     * @brief Unlocks (releases) the brakes on specified axes of Head.
     *
     * This function disengages (unlocks) the brakes on one or more axes of Head
     * as specified by the axis mask. Brake unlocking allows movement on the
     * affected axes, typically done after maintenance or before normal operation.
     *
     * @param[in] axis_mask Bitmask specifying which axes' brakes to unlock.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to unlock
     *                      the brake on that axis, 0 to leave unchanged.
     *                      Example: 0x0A (binary 00001010) unlocks brakes
     *                      on axes 1 and 3.
     *
     * @return int Returns #1 if the brake unlock command was successfully
     *                  sent to all specified axes. Returns #0 if the
     *                  operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, hardware faults.
     *
     * @note After unlocking brakes, axes may be in a "free" state where they
     *       can move freely (especially under gravity). Ensure proper support
     *       or enable servo power before unlocking brakes on vertical axes.
     *
     * @warning Unlocking brakes on unpowered or unsupported axes may cause
     *          unexpected movement, especially for vertical axes that are
     *          counteracting gravity. Take appropriate safety precautions.
     *
     * @pre The system should be in a safe state for brake release. For vertical
     *      axes, ensure servo power is enabled or mechanical support is in place
     *      before unlocking brakes to prevent uncontrolled movement.
     *
     * @see FX_L0_Head_Config_SetBrakeLock()
     */
    CONTROL_SDK_API int FX_L0_Head_Config_SetBrakeUnlock(unsigned char axis_mask);
    /**
     * @brief Resets the encoder offset for specified axes of Head.
     *
     * This function resets the encoder offset (sometimes called "zero position"
     * or "home offset") for one or more axes of Head as specified by the axis mask.
     * This is typically used during calibration or homing procedures to establish
     * a new reference position for the encoders.
     *
     * @param[in] axis_mask Bitmask specifying which axes' encoder offsets to reset.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to reset the
     *                      encoder offset on that axis, 0 to leave unchanged.
     *                      Example: 0x07 (binary 00000111) resets offsets
     *                      on axes 0, 1, and 2.
     *
     * @return int Returns #1 if the encoder offset reset was successful
     *                  for all specified axes. Returns #0 if the operation
     *                  failed on one or more axes due to invalid axis_mask,
     *                  communication errors, hardware faults, or because the
     *                  axis is currently moving or in a fault state.
     *
     * @note Resetting encoder offsets will cause the current position to be
     *       treated as the new reference (usually zero). This affects all
     *       subsequent position commands and feedback. Use with caution.
     *
     * @warning This operation can cause significant positioning errors if
     *          performed incorrectly. Ensure the robot is in a known,
     *          calibrated position before resetting encoder offsets.
     *          Improper use may require complete recalibration.
     *
     * @pre The specified axes should be stationary and in a known reference
     *      position. The system should be in a calibration or maintenance mode.
     *
     * @post If successful, the encoder offsets for specified axes are reset,
     *       making the current position the new reference. All subsequent
     *       position readings and commands will be relative to this new zero.
     */
    CONTROL_SDK_API int FX_L0_Head_Config_ResetEncOffset(unsigned char axis_mask);
    /**
     * @brief Clears encoder errors for specified axes of Head.
     *
     * This function clears error flags and resets error states for the encoders
     * on one or more axes of Head as specified by the axis mask. This is used
     * to recover from encoder-related errors such as communication faults,
     * count errors, or hardware errors.
     *
     * @param[in] axis_mask Bitmask specifying which axes' encoder errors to clear.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to clear
     *                      encoder errors on that axis, 0 to leave unchanged.
     *                      Example: 0x0F (binary 00001111) clears errors
     *                      on axes 0, 1, 2, and 3.
     *
     * @return int Returns #1 if encoder errors were successfully
     *                  cleared for all specified axes. Returns #0 if
     *                  the operation failed on one or more axes due to
     *                  invalid axis_mask, communication errors, or persistent
     *                  hardware faults that cannot be cleared.
     *
     * @note Clearing encoder errors does not fix underlying hardware problems.
     *       If errors persist after clearing, there may be a hardware issue
     *       requiring maintenance. Some errors may reoccur immediately if
     *       the root cause is not addressed.
     *
     * @pre The axes should be in a safe state (stopped) before clearing
     *      encoder errors. Clearing errors during motion may cause
     *      unpredictable behavior.
     *
     * @post If successful, encoder error flags for the specified axes are
     *       cleared. The encoders are reset to a normal operating state
     *       if the underlying issue is resolved. Position tracking may
     *       be lost and require re-homing.
     */
    CONTROL_SDK_API int FX_L0_Head_Config_ClearEncError(unsigned char axis_mask);
    /**
     * @brief Resets the external encoder offset for specified axes of the Head module.
     *
     * This function resets the external encoder offset (zero position reference)
     * for one or more axes of the Head module as specified by the axis mask.
     * External encoders are typically auxiliary position sensors separate from
     * the main motor encoders, used for redundant position verification or
     * secondary feedback.
     *
     * @param[in] axis_mask Bitmask specifying which axes' external encoder offsets to reset.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to reset the
     *                      external encoder offset on that axis, 0 to leave unchanged.
     *                      Example: 0x03 (binary 00000011) resets external encoder
     *                      offsets on axes 0 and 1 of the Head module.
     *
     * @return int Returns #1 if the external encoder offset reset was
     *                  successful for all specified axes. Returns #0 if
     *                  the operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, the axis doesn't have
     *                  an external encoder, or because the axis is in a fault state.
     *
     * @note External encoders are often used for safety verification or as
     *       backup position sensors. Resetting their offsets should be done
     *       carefully to maintain consistency with the primary encoders.
     *
     * @warning Resetting external encoder offsets independently of primary
     *          encoders can cause disagreement between position feedback
     *          sources, potentially triggering safety faults. Ensure the
     *          Head is in a known reference position before resetting.
     *
     * @pre The Head module should be in a known, calibrated position. The
     *      axes should be stationary. The system should be in a calibration
     *      or maintenance mode.
     */
    CONTROL_SDK_API int FX_L0_Head_Config_ResetExtEncOffset(unsigned char axis_mask);
    /**
     * @brief Disables the software position limits for specified axes of Head.
     *
     * This function disables the software-imposed position limits (soft limits)
     * for one or more axes of Head as specified by the axis mask. Soft limits
     * are safety features that prevent the axes from moving beyond predefined
     * safe positions. Disabling them should only be done for maintenance,
     * calibration, or recovery procedures.
     *
     * @param[in] axis_mask Bitmask specifying which axes' soft limits to disable.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to disable
     *                      soft limits on that axis, 0 to enable limits.
     *                      Example: 0x1F (binary 00011111) disables soft limits
     *                      on axes 0, 1, 2, 3, and 4.
     *
     * @return int Returns #1 if soft limits were successfully disabled
     *                  for all specified axes. Returns #0 if the operation
     *                  failed on one or more axes due to invalid axis_mask,
     *                  communication errors, hardware faults, or because the
     *                  axis is in a state that prevents limit disabling
     *                  (e.g., during motion).
     *
     * @note When soft limits are disabled, the axes can move beyond their normal
     *       operational ranges, which may cause collisions or damage. This operation
     *       is only allowed in IDLE state and will be set enable automatically
     *       when Head transfer to IDLE state next time.
     *
     * @warning Disabling soft limits removes an important safety feature.
     *          Only disable soft limits when absolutely necessary and with
     *          proper safety precautions. Never leave axes with soft limits
     *          disabled during normal operation.
     *
     * @pre The axes should be stationary. The operator should be aware of
     *      the risks and be ready to intervene. Consider disabling only
     *      the specific axes needed for the procedure.
     *
     * @post If successful, soft limits for the specified axes are disabled.
     *       The axes can now move beyond their normal limits until soft
     *       limits are re-enabled or the system is reset.
     */
    CONTROL_SDK_API int FX_L0_Head_Config_DisableSoftLimit(unsigned char axis_mask);
    /**
     * @brief Locks the brakes on specified axes of Body.
     *
     * This function engages (locks) the brakes on one or more axes of Body
     * as specified by the axis mask. Brake locking prevents movement on the
     * affected axes, typically used for safety during maintenance or to
     * maintain position when power is removed.
     *
     * @param[in] axis_mask Bitmask specifying which axes' brakes to lock.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to lock
     *                      the brake on that axis, 0 to leave unchanged.
     *                      Example: 0x05 (binary 00000101) locks brakes
     *                      on axes 0 and 2.
     *
     * @return int Returns #1 if the brake lock command was successfully
     *                  sent to all specified axes. Returns #0 if the
     *                  operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, hardware faults.
     *
     * @warning Locking brakes while the axis is moving may cause abrupt stops
     *          and mechanical stress. Ensure axes are stationary before locking
     *          brakes. Some systems may prevent brake locking during motion.
     *
     * @pre Axes may need to be disabled or powered down
     *      before brakes can be locked.
     *
     * @see FX_L0_Body_Config_SetBrakeUnlock()
     */
    CONTROL_SDK_API int FX_L0_Body_Config_SetBrakeLock(unsigned char axis_mask);
    /**
     * @brief Unlocks (releases) the brakes on specified axes of Body.
     *
     * This function disengages (unlocks) the brakes on one or more axes of Body
     * as specified by the axis mask. Brake unlocking allows movement on the
     * affected axes, typically done after maintenance or before normal operation.
     *
     * @param[in] axis_mask Bitmask specifying which axes' brakes to unlock.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to unlock
     *                      the brake on that axis, 0 to leave unchanged.
     *                      Example: 0x0A (binary 00001010) unlocks brakes
     *                      on axes 1 and 3.
     *
     * @return int Returns #1 if the brake unlock command was successfully
     *                  sent to all specified axes. Returns #0 if the
     *                  operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, hardware faults.
     *
     * @note After unlocking brakes, axes may be in a "free" state where they
     *       can move freely (especially under gravity). Ensure proper support
     *       or enable servo power before unlocking brakes on vertical axes.
     *
     * @warning Unlocking brakes on unpowered or unsupported axes may cause
     *          unexpected movement, especially for vertical axes that are
     *          counteracting gravity. Take appropriate safety precautions.
     *
     * @pre The system should be in a safe state for brake release. For vertical
     *      axes, ensure servo power is enabled or mechanical support is in place
     *      before unlocking brakes to prevent uncontrolled movement.
     *
     * @see FX_L0_Body_Config_SetBrakeLock()
     */

    CONTROL_SDK_API int FX_L0_Body_Config_SetBrakeUnlock(unsigned char axis_mask);
    /**
     * @brief Resets the encoder offset for specified axes of Body.
     *
     * This function resets the encoder offset (sometimes called "zero position"
     * or "home offset") for one or more axes of Body as specified by the axis mask.
     * This is typically used during calibration or homing procedures to establish
     * a new reference position for the encoders.
     *
     * @param[in] axis_mask Bitmask specifying which axes' encoder offsets to reset.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to reset the
     *                      encoder offset on that axis, 0 to leave unchanged.
     *                      Example: 0x07 (binary 00000111) resets offsets
     *                      on axes 0, 1, and 2.
     *
     * @return int Returns #1 if the encoder offset reset was successful
     *                  for all specified axes. Returns #0 if the operation
     *                  failed on one or more axes due to invalid axis_mask,
     *                  communication errors, hardware faults, or because the
     *                  axis is currently moving or in a fault state.
     *
     * @note Resetting encoder offsets will cause the current position to be
     *       treated as the new reference (usually zero). This affects all
     *       subsequent position commands and feedback. Use with caution.
     *
     * @warning This operation can cause significant positioning errors if
     *          performed incorrectly. Ensure the robot is in a known,
     *          calibrated position before resetting encoder offsets.
     *          Improper use may require complete recalibration.
     *
     * @pre The specified axes should be stationary and in a known reference
     *      position. The system should be in a calibration or maintenance mode.
     *
     * @post If successful, the encoder offsets for specified axes are reset,
     *       making the current position the new reference. All subsequent
     *       position readings and commands will be relative to this new zero.
     */

    CONTROL_SDK_API int FX_L0_Body_Config_ResetEncOffset(unsigned char axis_mask);
    /**
     * @brief Clears encoder errors for specified axes of Body.
     *
     * This function clears error flags and resets error states for the encoders
     * on one or more axes of Body as specified by the axis mask. This is used
     * to recover from encoder-related errors such as communication faults,
     * count errors, or hardware errors.
     *
     * @param[in] axis_mask Bitmask specifying which axes' encoder errors to clear.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to clear
     *                      encoder errors on that axis, 0 to leave unchanged.
     *                      Example: 0x0F (binary 00001111) clears errors
     *                      on axes 0, 1, 2, and 3.
     *
     * @return int Returns #1 if encoder errors were successfully
     *                  cleared for all specified axes. Returns #0 if
     *                  the operation failed on one or more axes due to
     *                  invalid axis_mask, communication errors, or persistent
     *                  hardware faults that cannot be cleared.
     *
     * @note Clearing encoder errors does not fix underlying hardware problems.
     *       If errors persist after clearing, there may be a hardware issue
     *       requiring maintenance. Some errors may reoccur immediately if
     *       the root cause is not addressed.
     *
     * @pre The axes should be in a safe state (stopped) before clearing
     *      encoder errors. Clearing errors during motion may cause
     *      unpredictable behavior.
     *
     * @post If successful, encoder error flags for the specified axes are
     *       cleared. The encoders are reset to a normal operating state
     *       if the underlying issue is resolved. Position tracking may
     *       be lost and require re-homing.
     */
    CONTROL_SDK_API int FX_L0_Body_Config_ClearEncError(unsigned char axis_mask);
    /**
     * @brief Resets the external encoder offset for specified axes of the Body module.
     *
     * This function resets the external encoder offset (zero position reference)
     * for one or more axes of the Body module as specified by the axis mask.
     * External encoders are typically auxiliary position sensors separate from
     * the main motor encoders, used for redundant position verification or
     * secondary feedback.
     *
     * @param[in] axis_mask Bitmask specifying which axes' external encoder offsets to reset.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to reset the
     *                      external encoder offset on that axis, 0 to leave unchanged.
     *                      Example: 0x03 (binary 00000011) resets external encoder
     *                      offsets on axes 0 and 1 of the Body module.
     *
     * @return int Returns #1 if the external encoder offset reset was
     *                  successful for all specified axes. Returns #0 if
     *                  the operation failed on one or more axes due to invalid
     *                  axis_mask, communication errors, the axis doesn't have
     *                  an external encoder, or because the axis is in a fault state.
     *
     * @note External encoders are often used for safety verification or as
     *       backup position sensors. Resetting their offsets should be done
     *       carefully to maintain consistency with the primary encoders.
     *
     * @warning Resetting external encoder offsets independently of primary
     *          encoders can cause disagreement between position feedback
     *          sources, potentially triggering safety faults. Ensure the
     *          Body is in a known reference position before resetting.
     *
     * @pre The Body module should be in a known, calibrated position. The
     *      axes should be stationary. The system should be in a calibration
     *      or maintenance mode.
     */
    CONTROL_SDK_API int FX_L0_Body_Config_ResetExtEncOffset(unsigned char axis_mask);
    /**
     * @brief Disables the software position limits for specified axes of Body.
     *
     * This function disables the software-imposed position limits (soft limits)
     * for one or more axes of Body as specified by the axis mask. Soft limits
     * are safety features that prevent the axes from moving beyond predefined
     * safe positions. Disabling them should only be done for maintenance,
     * calibration, or recovery procedures.
     *
     * @param[in] axis_mask Bitmask specifying which axes' soft limits to disable.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to disable
     *                      soft limits on that axis, 0 to enable limits.
     *                      Example: 0x1F (binary 00011111) disables soft limits
     *                      on axes 0, 1, 2, 3, and 4.
     *
     * @return int Returns #1 if soft limits were successfully disabled
     *                  for all specified axes. Returns #0 if the operation
     *                  failed on one or more axes due to invalid axis_mask,
     *                  communication errors, hardware faults, or because the
     *                  axis is in a state that prevents limit disabling
     *                  (e.g., during motion).
     *
     * @note When soft limits are disabled, the axes can move beyond their normal
     *       operational ranges, which may cause collisions or damage. This operation
     *       is only allowed in IDLE state and will be set enable automatically
     *       when Body transfer to IDLE/TORQUE state next time.
     *
     * @warning Disabling soft limits removes an important safety feature.
     *          Only disable soft limits when absolutely necessary and with
     *          proper safety precautions. Never leave axes with soft limits
     *          disabled during normal operation.
     *
     * @pre The axes should be stationary. The operator should be aware of
     *      the risks and be ready to intervene. Consider disabling only
     *      the specific axes needed for the procedure.
     *
     * @post If successful, soft limits for the specified axes are disabled.
     *       The axes can now move beyond their normal limits until soft
     *       limits are re-enabled or the system is reset.
     */
    CONTROL_SDK_API int FX_L0_Body_Config_DisableSoftLimit(unsigned char axis_mask);
    /**
     * @brief Sets the sensor offset for a specific axis of Body.
     *
     * This function sets a calibration offset value for a sensor on a specified
     * axis of Body. The offset is added to the raw sensor reading to compensate
     * for sensor misalignment, manufacturing tolerances, or to establish a
     * known zero reference.
     *
     * @param[in] axis_id The identifier of the axis whose sensor offset will be set.
     *                    Valid range is implementation-dependent (typically 0 to 5
     *                    for 6 axes). Refer to hardware documentation for specific
     *                    axis numbering.
     * @param[in] offset The offset value to apply to the sensor readings, in sensor
     *                   counts or units. Positive values shift readings upward,
     *                   negative values shift downward. The exact unit depends on
     *                   the sensor type (e.g., encoder counts, resolver bits).
     *
     * @return int Returns #1 if the sensor offset was successfully set.
     *                  Returns #0 if the operation failed due to invalid
     *                  axis_id, communication error, the sensor doesn't support
     *                  offset adjustment, or the offset value is out of range.
     *
     * @note The offset is typically stored in non-volatile memory and persists
     *       across power cycles. Some sensors may have separate offsets for
     *       different operating modes (e.g., absolute vs. incremental).
     *
     * @warning Incorrect sensor offsets can cause positioning errors, safety
     *          faults, or system damage. Set offsets only during calibration
     *          procedures by trained personnel. Always verify the new offset
     *          with known reference positions.
     *
     * @pre The axis should be stationary and in a known reference position.
     *      The sensor should be properly initialized and functional.
     *
     * @post If successful, subsequent sensor readings on the specified axis
     *       will have the offset applied. The offset is typically stored
     *       persistently and will be used after system restart.
     */
    CONTROL_SDK_API int FX_L0_Body_Config_SetSensorOffset(int axis_id, int offset);
    /**
     * @brief Resets the encoder offset for specified axes of Lift.
     *
     * This function resets the encoder offset (sometimes called "zero position"
     * or "home offset") for one or more axes of Lift as specified by the axis mask.
     * This is typically used during calibration or homing procedures to establish
     * a new reference position for the encoders.
     *
     * @param[in] axis_mask Bitmask specifying which axes' encoder offsets to reset.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to reset the
     *                      encoder offset on that axis, 0 to leave unchanged.
     *                      Example: 0x07 (binary 00000111) resets offsets
     *                      on axes 0, 1, and 2.
     *
     * @return int Returns #1 if the encoder offset reset was successful
     *                  for all specified axes. Returns #0 if the operation
     *                  failed on one or more axes due to invalid axis_mask,
     *                  communication errors, hardware faults, or because the
     *                  axis is currently moving or in a fault state.
     *
     * @note Resetting encoder offsets will cause the current position to be
     *       treated as the new reference (usually zero). This affects all
     *       subsequent position commands and feedback. Use with caution.
     *
     * @warning This operation can cause significant positioning errors if
     *          performed incorrectly. Ensure the robot is in a known,
     *          calibrated position before resetting encoder offsets.
     *          Improper use may require complete recalibration.
     *
     * @pre The specified axes should be stationary and in a known reference
     *      position. The system should be in a calibration or maintenance mode.
     *
     * @post If successful, the encoder offsets for specified axes are reset,
     *       making the current position the new reference. All subsequent
     *       position readings and commands will be relative to this new zero.
     */
    CONTROL_SDK_API int FX_L0_Lift_Config_ResetEncOffset(unsigned char axis_mask);
    /**
     * @brief Disables the software position limits for specified axes of Lift.
     *
     * This function disables the software-imposed position limits (soft limits)
     * for one or more axes of Lift as specified by the axis mask. Soft limits
     * are safety features that prevent the axes from moving beyond predefined
     * safe positions. Disabling them should only be done for maintenance,
     * calibration, or recovery procedures.
     *
     * @param[in] axis_mask Bitmask specifying which axes' soft limits to disable.
     *                      Each bit corresponds to an axis (bit 0 = axis 0,
     *                      bit 1 = axis 1, etc.). Set a bit to 1 to disable
     *                      soft limits on that axis, 0 to enable limits.
     *                      Example: 0x1F (binary 00011111) disables soft limits
     *                      on axes 0, 1, 2, 3, and 4.
     *
     * @return int Returns #1 if soft limits were successfully disabled
     *                  for all specified axes. Returns #0 if the operation
     *                  failed on one or more axes due to invalid axis_mask,
     *                  communication errors, hardware faults, or because the
     *                  axis is in a state that prevents limit disabling
     *                  (e.g., during motion).
     *
     * @note When soft limits are disabled, the axes can move beyond their normal
     *       operational ranges, which may cause collisions or damage. This operation
     *       is only allowed in IDLE state and will be set enable automatically
     *       when Lift transfer to IDLE state next time.
     *
     * @warning Disabling soft limits removes an important safety feature.
     *          Only disable soft limits when absolutely necessary and with
     *          proper safety precautions. Never leave axes with soft limits
     *          disabled during normal operation.
     *
     * @pre The axes should be stationary. The operator should be aware of
     *      the risks and be ready to intervene. Consider disabling only
     *      the specific axes needed for the procedure.
     *
     * @post If successful, soft limits for the specified axes are disabled.
     *       The axes can now move beyond their normal limits until soft
     *       limits are re-enabled or the system is reset.
     */
    CONTROL_SDK_API int FX_L0_Lift_Config_DisableSoftLimit(unsigned char axis_mask);
    /**
     * @brief Immediately stops all motion and activates emergency stop for Arm0.
     *
     * This function triggers an emergency stop (E-stop) condition for the Arm0
     * robotic system. It immediately halts all servo movements, disables motors,
     * and applies brakes (if available) to bring Arm0 to a complete stop as
     * quickly as possible. This function should be used in critical safety
     * situations to prevent damage or injury.
     *
     * @return int Returns #1 if the emergency stop was successfully
     *                  activated. Returns #0 if the emergency stop
     *                  failed to activate due to hardware faults, communication
     *                  errors, or because Arm0 is already in an emergency stop
     *                  state.
     *
     * @note Once activated, the emergency stop condition typically requires
     *       manual intervention to reset. After calling this function, the
     *       system may need to be explicitly reset or re-enabled before normal
     *       operation can resume.
     *
     * @warning This function causes abrupt stopping which may cause mechanical
     *          stress, positioning errors, or dropped payloads. Use only in
     *          genuine emergency situations. Do not use for normal stopping
     *          procedures.
     *
     * @pre None. This function can be called at any time, even during motion.
     *
     * @post Arm0 is in emergency stop state: motors are disabled, brakes are
     *       engaged (if available), and Arm0 will go to ERROR state.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_EmergencyStop(void);
    /**
     * @brief Sets the runtime state of the Arm0.
     *
     * This function transitions the Arm0 controller to a specified runtime state,
     * controlling the overall operational mode and behavior of the arm. State
     * changes may affect motion execution, error handling, and system responses.
     *
     * @param[in] state The desired runtime state for Arm0. Valid states are
     *                  defined in the ArmState enumeration, which typically includes:
     *                  - ARM_STATE_IDLE: Arm is disabled
     *                  - ARM_STATE_POSITION: Arm is in position control mode
     *                  - ARM_STATE_TORQUE: Arm is in torque control mode with all force compensations
     *                  - ARM_STATE_RELEASE: Arm is in torque control mode with only gravity compensation
     *
     * @return int Returns #1 if the state transition was successful.
     *                  Returns #0 if the state transition failed due to
     *                  invalid target state, current state prevents transition,
     *                  hardware errors, or safety constraints.
     *
     * @pre The current state must allow transition to the target state.
     *      Some transitions may require specific conditions (e.g., all
     *      axes stopped, no active faults).
     *
     * @post If successful, Arm0 enters the specified runtime state with
     *       corresponding behavior changes. State-dependent functions
     *       will behave according to the new state.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetState(int state);
    /**
     * @brief Sets the joint position command for Arm0.
     *
     * This function commands Arm0 to move to specified joint positions (angles
     * for each joint). The command specifies the target positions for all
     * 7 joints of Arm0. The arm will move to these positions using the
     * currently configured motion settings.
     *
     * @param[in] joint_pos Array of 7 joint position values (FX_DOUBLE) in
     *                     degrees.
     *
     * @return int Returns #1 if the joint position command was
     *                  successfully accepted and queued for execution.
     *                  Returns #0 if the command was rejected.
     *
     * @post If successful, Arm0 begins moving to the specified joint positions.
     *       The motion occurs asynchronously; the function returns before
     *       motion is complete.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetJointPosCmd(double joint_pos[7]);
    /**
     * @brief Sets the joint torque command for Arm0.
     *
     * This function commands torque values for each of the 7 joints of Arm0.
     * The arm will apply the specified torques to each joint motor. This is
     * typically used for force control applications.
     *
     * @param[in] joint_tor Array of 7 joint torque values (double) in
     *                     Newton-meters (Nm).
     *
     * @return int Returns #1 if the joint torque command was
     *                  successfully accepted and sent to the joint controllers.
     *                  Returns #0 if the command was rejected.
     *
     * @note This function is for direct torque control, which is an advanced
     *       feature requiring proper configuration. The arm must typically be
     *       in a specific control mode (e.g., torque control mode) to accept
     *       torque commands. Use FX_L0_Arm0_Runtime_SetImpType() to set
     *       the appropriate mode.
     *
     * @post If successful, the joint controllers apply the specified torques.
     *       The torques are applied continuously until new torque commands
     *       are sent or the control mode is changed.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetJointTorCmd(double joint_tor[7]);
    /**
     * @brief Sets the force control parameters for Arm0 to apply a certain force in some direction.
     *
     * This function configures the force control parameters for Arm0's
     * end-effector or specified contact points. Force control enables
     * the arm to apply or regulate forces in Cartesian space,
     * typically used for assembly, polishing, or contact operations.
     *
     * @param[in] force_ctrl Array of 5 force control parameters (double).
     *                      The parameters typically specify:
     *                      - force_ctrl[0]: Direction of force in X direction under the arm base coordinate
     *                      - force_ctrl[1]: Direction of force in Y direction under the arm base coordinate
     *                      - force_ctrl[2]: Direction of force in Z direction under the arm base coordinate
     *                      - force_ctrl[3]: Magnitude of force (N)
     *                      - force_ctrl[4]: Maximum move distance along the force (mm)
     *
     * @return int Returns #1 if the force control parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Force control can apply significant forces. Incorrect parameters
     *          may cause damage to the arm, tooling, or workpiece. Always
     *          start with conservative force limits and monitor force feedback
     *          during operation.
     *
     * @pre Arm0 must be configured for force control mode. A force/torque
     *      sensor should be calibrated and operational. The end-effector
     *      should be properly characterized (mass, center of gravity).
     *
     * @post If successful, Arm0 will attempt to achieve the specified
     *       forces/torques at the end-effector. The control is typically
     *       continuous until new parameters are set or control mode changes.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetForceCtrl(double force_ctrl[5]);
    /**
     * @brief Sets the force control parameters for Arm0 to apply a certain torque in some direction.
     *
     * This function configures the force control parameters for Arm0's
     * end-effector or specified contact points. Torque control enables
     * the arm to apply or regulate torques in Cartesian space,
     * typically used for assembly, polishing, or contact operations.
     *
     * @param[in] torque_ctrl Array of 5 torque control parameters (double).
     *                      The parameters typically specify:
     *                      - torque_ctrl[0]: Direction of torque in X direction under the arm base coordinate
     *                      - torque_ctrl[1]: Direction of torque in Y direction under the arm base coordinate
     *                      - torque_ctrl[2]: Direction of torque in Z direction under the arm base coordinate
     *                      - torque_ctrl[3]: Magnitude of torque (Nm)
     *                      - torque_ctrl[4]: Maximum rotate angles along the torque (rad)
     *
     * @return int Returns #1 if the torque control parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Torque control can apply significant torque. Incorrect parameters
     *          may cause damage to the arm, tooling, or workpiece. Always
     *          start with conservative force limits and monitor force feedback
     *          during operation.
     *
     * @pre Arm0 must be configured for force control mode. A force/torque
     *      sensor should be calibrated and operational. The end-effector
     *      should be properly characterized (mass, center of gravity).
     *
     * @post If successful, Arm0 will attempt to achieve the specified
     *       forces/torques at the end-effector. The control is typically
     *       continuous until new parameters are set or control mode changes.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetTorqueCtrl(double torque_ctrl[5]);
    /**
     * @brief Sets the velocity ratio (override) for Arm0.
     *
     * This function sets a scaling factor for Arm0's programmed velocities.
     * The vel_ratio parameter multiplies all commanded velocities, allowing
     * for runtime adjustment of motion speed without reprogramming paths.
     * This is commonly used for speed override during debugging, testing,
     * or operator control.
     *
     * @param[in] vel_ratio Velocity scaling factor as a multiplier.
     *                     Range is typically 1.0 to 100.0 (1% to 100% of
     *                     programmed speed).
     *
     * @return int Returns #1 if the velocity ratio was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @note The velocity ratio affects all subsequent motion commands dynamically.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetVelRatio(double vel_ratio);
    /**
     * @brief Sets the acceleration ratio (override) for Arm0.
     *
     * This function sets a scaling factor for Arm0's programmed accelerations
     * and decelerations. The acc_ratio parameter multiplies all commanded
     * acceleration values, allowing for runtime adjustment of motion
     * aggressiveness without reprogramming paths. This affects how quickly
     * the arm speeds up and slows down.
     *
     * @param[in] acc_ratio Acceleration scaling factor as a multiplier.
     *                     Range is typically 1.0 to 100.0 (1% to 100% of
     *                     programmed acceleration).
     *
     * @return int Returns #1 if the acceleration ratio was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @note The acceleration ratio affects all subsequent motion commands until
     *       changed. Lower ratios result in smoother, more gradual motions
     *       which may be desirable for delicate operations or payload stability.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetAccRatio(double acc_ratio);
    /**
     * @brief Sets the joint stiffness (K) gains for Arm0.
     *
     * This function configures the joint stiffness parameters (proportional
     * gains) for Arm0's impedance control. The stiffness values
     * determine how rigidly the joints maintain their commanded positions
     * against external forces. Higher K values make joints stiffer; lower
     * values make them more compliant.
     *
     * @param[in] k Array of 7 joint stiffness values (double) in appropriate
     *             units (typically Nm/rad).
     *
     * @return int Returns #1 if the joint stiffness parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Excessive stiffness can cause high current draw, mechanical
     *          resonance, and instability. Insufficient stiffness can cause
     *          poor position tracking and overshoot. Tune carefully, often
     *          starting with lower values and increasing gradually.
     *
     * @post If successful, the joint controllers use the new stiffness values
     *       for subsequent control cycles. The effect is immediate for
     *       impedance control.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetJointK(double k[7]);
    /**
     * @brief Sets the joint damping (D) gains for Arm0.
     *
     * This function configures the joint damping parameters (derivative
     * gains) for Arm0's impedance or position control. The damping values
     * determine how quickly velocity errors are corrected and help to
     * suppress oscillations and vibrations. Higher D values increase
     * damping; lower values reduce it.
     *
     * @param[in] d Array of 7 joint damping values (double) in appropriate
     *             units (typically Nm·s/rad).
     *
     * @return int Returns #1 if the joint damping parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Excessive damping can cause sluggish response, high current
     *          draw, and motor overheating. Insufficient damping can cause
     *          oscillations, overshoot, and instability. Damping should be
     *          tuned in conjunction with stiffness parameters.
     *
     * @post If successful, the joint controllers use the new damping values
     *       for subsequent control cycles. The effect is immediate for
     *       impedance control; for position control, it affects the
     *       derivative gain of the position loop.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetJointD(double d[7]);
    /**
     * @brief Sets the Cartesian stiffness (K) gains for Arm0.
     *
     * This function configures the Cartesian stiffness parameters (proportional
     * gains) for Arm0's Cartesian space impedance or position control. The
     * stiffness values determine how rigidly the end-effector maintains its
     * commanded position and orientation in Cartesian space (X, Y, Z, RX, RY, RZ, R0)
     * against external forces. Higher K values make the end-effector stiffer;
     * lower values make it more compliant.
     *
     * @param[in] k Array of 7 Cartesian stiffness values (double) in appropriate
     *             units (typically N/Deg for linear, Nm/Deg for rotational).
     *             The array elements typically correspond to:
     *             - k[0]: X-direction translational stiffness(N/mm)
     *             - k[1]: Y-direction translational stiffness(N/mm)
     *             - k[2]: Z-direction translational stiffness(N/mm)
     *             - k[3]: X-axis rotational stiffness(Nm/rad)
     *             - k[4]: Y-axis rotational stiffness(Nm/rad)
     *             - k[5]: Z-axis rotational stiffness(Nm/rad)
     *             - k[6]: Zero space stiffness(Nm/rad)
     *
     * @return int Returns #1 if the Cartesian stiffness parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Excessive Cartesian stiffness can cause high joint torques,
     *          especially near singularities or workspace boundaries. The
     *          effective joint stiffness varies with arm configuration.
     *          Always verify joint torque limits are not exceeded.
     *
     * @post If successful, the Cartesian controller uses the new stiffness
     *       values for subsequent control cycles. The effective joint-level
     *       stiffness will vary with arm configuration according to the
     *       Jacobian transpose transformation.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetCartK(double k[7]);
    /**
     * @brief Sets the Cartesian damping (D) gains for Arm0.
     *
     * This function configures the Cartesian damping parameters (derivative
     * gains) for Arm0's Cartesian space impedance or position control. The
     * damping values determine how quickly velocity errors are corrected in
     * Cartesian space (X, Y, Z, RX, RY, RZ, R0) and help to suppress oscillations
     * and vibrations at the end-effector. Higher D values increase damping;
     * lower values reduce it.
     *
     * @param[in] d Array of 7 Cartesian damping values (double) in appropriate
     *             units (typically N·s/m for linear, Nm·s/rad for rotational, or
     *             system-specific units). The array elements typically correspond to:
     *             - d[0]: X-direction translational damping (N·s/mm)
     *             - d[1]: Y-direction translational damping (N·s/mm)
     *             - d[2]: Z-direction translational damping (N·s/mm)
     *             - d[3]: X-axis rotational damping (Nm·s/rad)
     *             - d[4]: Y-axis rotational damping (Nm·s/rad)
     *             - d[5]: Z-axis rotational damping (Nm·s/rad)
     *             - d[6]: Zero space damping (Nm·s/rad)
     *
     * @return int Returns #1 if the Cartesian damping parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Excessive Cartesian damping can cause sluggish response and
     *          high joint torques during fast motions. Insufficient damping
     *          can cause end-effector oscillations. Damping should be tuned
     *          in coordination with Cartesian stiffness and considering the
     *          current arm configuration.
     *
     * @post If successful, the Cartesian controller uses the new damping
     *       values for subsequent control cycles. The effective joint-level
     *       damping will vary with arm configuration according to the
     *       Jacobian transpose transformation.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetCartD(double d[7]);
    /**
     * @brief Sets the tool offset in the tool frame for Arm0.
     *
     * This function configures the translational and rotational offset
     * (transformation) from the end-effector flange to the tool tip or
     * working point. The tool offset defines where the effective tool
     * frame is located relative to the robot's flange, affecting all
     * subsequent Cartesian commands and measurements.
     *
     * @param[in] k Array of 6 tool offset values (double) representing
     *             the transformation from flange frame to tool frame.
     *             The array elements are:
     *             - k[0]: X-axis translation offset (mm)
     *             - k[1]: Y-axis translation offset (mm)
     *             - k[2]: Z-axis translation offset (mm)
     *             - k[3]: Rotation about X-axis (Deg)
     *             - k[4]: Rotation about Y-axis (Deg)
     *             - k[5]: Rotation about Z-axis (Deg)
     *
     * @return int Returns #1 if the tool offset was successfully
     *                  set. Returns #0 if the operation failed.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetToolK(double k[6]);
    /**
     * @brief Sets the tool dynamics parameters for Arm0.
     *
     * This function configures the dynamic properties of the tool attached
     * to Arm0's end-effector. These parameters are used for accurate
     * dynamics compensation, gravity compensation, and proper control
     * performance. The tool dynamics are essential for applications
     * requiring precise force control or when using tools with significant mass.
     *
     * @param[in] d Array of 10 tool dynamics parameters (double) that
     *             describe the tool's mass, center of mass, and inertia tensor.
     *             The array elements are:
     *             - d[0]: Tool mass (kilograms)
     *             - d[1]: Center of mass X coordinate in tool frame (mm)
     *             - d[2]: Center of mass Y coordinate in tool frame (mm)
     *             - d[3]: Center of mass Z coordinate in tool frame (mm)
     *             - d[4]: Inertia tensor XX component (kg·m²)
     *             - d[5]: Inertia tensor XY component (kg·m²)
     *             - d[6]: Inertia tensor XZ component (kg·m²)
     *             - d[7]: Inertia tensor YY component (kg·m²)
     *             - d[8]: Inertia tensor YZ component (kg·m²)
     *             - d[9]: Inertia tensor ZZ component (kg·m²)
     *             The inertia tensor is defined relative to the center of mass.
     *
     * @return int Returns #1 if the tool dynamics parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Incorrect dynamics parameters can cause:
     *          - Poor control performance
     *          - Oscillations or instability
     *          - Excessive joint torques
     *          - Accelerated wear on components
     *          Always use measured or accurately calculated values.
     *
     * @post If successful, the robot's dynamic model incorporates the new
     *       tool parameters for all subsequent control cycles. This affects
     *       gravity compensation, model-based control, and feedforward terms.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetToolD(double d[10]);
    /**
     * @brief Sets the impedance control type for Arm0.
     *
     * This function configures the type of impedance control used for Arm0.
     * Impedance control defines the relationship between motion and force at
     * the end-effector, allowing the robot to behave as a mass-spring-damper
     * system. Different impedance types are suitable for different tasks
     * such as contact operations, assembly, or human interaction.
     *
     * @param[in] imp_type The impedance control type identifier. Valid values
     *                    are defined in the system's impedance type enumeration,
     *                    which typically includes:
     *                    - 1: Joint space impedance control
     *                    - 2: Cartesian space impedance control
     *                    - 3: Force control
     *
     * @return int Returns #1 if the impedance type was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @post If successful, Arm0 uses the specified impedance type for all
     *       subsequent control cycles. This affects the dynamic response
     *       to both commanded motions and external forces.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetImpType(int imp_type);
    /**
     * @brief Sets the drag teaching (hand guiding) mode type for Arm0.
     *
     * This function configures the type of drag teaching (hand guiding) mode
     * for Arm0. Drag teaching allows an operator to physically move the robot
     * arm to teach positions or paths, with the robot providing adjustable
     * assistance or resistance. Different drag types offer varying levels of
     * gravity compensation, damping, and control authority for different
     * teaching scenarios.
     *
     * @param[in] drag_type The drag teaching type identifier. Valid values
     *                     are defined in the system's drag type enumeration,
     *                     which typically includes:
     *                     - 1: Joint space drag teaching
     *                     - 2: Cartesian space drag teaching in X direction under arm base coordinate
     *                     - 3: Cartesian space drag teaching in Y direction under arm base coordinate
     *                     - 4: Cartesian space drag teaching in Z direction under arm base coordinate
     *                     - 5: Cartesian space drag teaching in rotation relative to TCP
     *
     * @return int Returns #1 if the drag type was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @post If successful, Arm0 enters the specified drag teaching mode.
     *       The robot becomes responsive to external forces according to
     *       the selected drag type. Motors may be powered but controlled
     *       to allow movement.
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetDragType(int drag_type);
    /**
     * @brief Initializes a trajectory for Arm0 with the specified number of points.
     *
     * This function initializes a trajectory buffer for Arm0 that can hold
     * the specified number of trajectory points. Trajectory points typically
     * include position, velocity, and acceleration information for each joint
     * or Cartesian coordinate. This function allocates or prepares the
     * necessary memory and data structures for subsequent trajectory operations.
     *
     * @param[in] point_num The number of trajectory points to allocate in the
     *                     trajectory buffer. This determines the maximum
     *                     number of points that can be added to the trajectory
     *                     before execution. Range is typically 5 to 5000.
     *
     * @return int Returns #1 if the trajectory was successfully
     *                  initialized with the specified number of points.
     *                  Returns #0 if the operation failed.
     *
     * @note Initializing a trajectory does not start execution. After
     *       initialization, points must be added using functions like
     *       FX_L0_Arm0_Runtime_SetTraj(). Finally, execution is
     *       started with FX_L0_Arm0_Runtime_RunTraj().
     *
     * @pre Arm0 should be in position control state that supports trajectory planning.
     *
     * @post If successful, a trajectory buffer is allocated and prepared
     *       to hold point_num trajectory points. The trajectory is in
     *       "initialized" state, ready to receive points but not yet
     *       executable.
     *
     * @see FX_L0_Arm0_Runtime_SetTraj(), FX_L0_Arm0_Runtime_RunTraj(),
     *      FX_L0_Arm0_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_InitTraj(int point_num);
    /**
     * @brief Sets a block of trajectory points for Arm0.
     *
     * This function sets a contiguous block of trajectory points for Arm0,
     * starting at the specified serial number. This allows efficient bulk
     * loading of trajectory data, particularly useful for pre-computed
     * trajectories or loading from external sources. Each point includes
     * 7 joints' positions and acceleration data for all joints coordinates.
     *
     * @param[in] serial The starting frame index (0-based) in the trajectory buffer
     *                  where the new points should be placed. Each frame contains
     *                  up to 50 points. If points already exist at these indices,
     *                  they will be overwritten.
     * @param[in] point_num The number of points to set. If it is not the last frame
     *                     to set, the point number must be 50.
     *                     The trajectory buffer must have been previously
     *                     initialized with FX_L0_Arm0_Runtime_InitTraj().
     * @param[in] point_data Pointer to a contiguous array of point data.
     *                      The array should contain point_num * 7 elements.
     *                      Data layout is typically:
     *                      {point0_data0, point0_data1, ..., point0_data6,
     *                       point1_data0, point1_data1, ..., point1_data6, ...}
     *
     * @return int Returns #1 if the trajectory points were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @see FX_L0_Arm0_Runtime_InitTraj(), FX_L0_Arm0_Runtime_RunTraj(),
     *      FX_L0_Arm0_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_SetTraj(int serial, int point_num, double *point_data);
    /**
     * @brief Starts execution of the loaded trajectory for Arm0.
     *
     * This function begins execution of the trajectory that has been previously
     * loaded and initialized. The arm will start moving through the trajectory
     * points according to the timing, velocity, and acceleration specifications
     * defined in the trajectory data. Execution continues until all points
     * are completed, the trajectory is stopped, or an error occurs.
     *
     * @return int Returns #1 if trajectory execution was successfully
     *                  started. Returns #0 if the operation failed.
     *
     * @note Trajectory execution is typically asynchronous; this function
     *       returns immediately after starting execution. The trajectory can be
     *       stopped using FX_L0_Arm0_Runtime_StopTraj().
     *
     * @pre A trajectory must be properly initialized via
     *      FX_L0_Arm0_Runtime_InitTraj() and populated with points via
     *      FX_L0_Arm0_Runtime_SetTraj().
     *
     * @post If successful, Arm0 begins moving through the trajectory points.
     *       The trajectory execution state becomes "running". Motors are
     *       enabled and controlled to follow the trajectory path.
     *
     * @see FX_L0_Arm0_Runtime_InitTraj(), FX_L0_Arm0_Runtime_SetTraj(),
     *      FX_L0_Arm0_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_RunTraj(void);
    /**
     * @brief Stops execution of the currently running trajectory for Arm0.
     *
     * This function immediately stops execution of the trajectory that is
     * currently running. The arm will decelerate to a stop according to
     * configured deceleration profiles or emergency stop parameters.
     * After stopping, the trajectory execution state is set to "stopped"
     * and the arm remains at its current position (or the position
     * reached after deceleration).
     *
     * @return int Returns #1 if the trajectory was successfully
     *                  stopped. Returns #0 if the operation failed.
     *
     * @post If successful, trajectory execution is terminated. The arm
     *       comes to a complete stop.
     *
     * @see FX_L0_Arm0_Runtime_InitTraj(), FX_L0_Arm0_Runtime_SetTraj(),
     *      FX_L0_Arm0_Runtime_RunTraj()
     */
    CONTROL_SDK_API int FX_L0_Arm0_Runtime_StopTraj(void);
    /**
     * @brief Immediately stops all motion and activates emergency stop for Arm1.
     *
     * This function triggers an emergency stop (E-stop) condition for the Arm1
     * robotic system. It immediately halts all servo movements, disables motors,
     * and applies brakes (if available) to bring Arm1 to a complete stop as
     * quickly as possible. This function should be used in critical safety
     * situations to prevent damage or injury.
     *
     * @return int Returns #1 if the emergency stop was successfully
     *                  activated. Returns #0 if the emergency stop
     *                  failed to activate due to hardware faults, communication
     *                  errors, or because Arm1 is already in an emergency stop
     *                  state.
     *
     * @note Once activated, the emergency stop condition typically requires
     *       manual intervention to reset. After calling this function, the
     *       system may need to be explicitly reset or re-enabled before normal
     *       operation can resume.
     *
     * @warning This function causes abrupt stopping which may cause mechanical
     *          stress, positioning errors, or dropped payloads. Use only in
     *          genuine emergency situations. Do not use for normal stopping
     *          procedures.
     *
     * @pre None. This function can be called at any time, even during motion.
     *
     * @post Arm1 is in emergency stop state: motors are disabled, brakes are
     *       engaged (if available), and Arm1 will go to ERROR state.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_EmergencyStop(void);
    /**
     * @brief Sets the runtime state of the Arm1.
     *
     * This function transitions the Arm1 controller to a specified runtime state,
     * controlling the overall operational mode and behavior of the arm. State
     * changes may affect motion execution, error handling, and system responses.
     *
     * @param[in] state The desired runtime state for Arm1. Valid states are
     *                  defined in the ArmState enumeration, which typically includes:
     *                  - ARM_STATE_IDLE: Arm is disabled
     *                  - ARM_STATE_POSITION: Arm is in position control mode
     *                  - ARM_STATE_TORQUE: Arm is in torque control mode with all force compensations
     *                  - ARM_STATE_RELEASE: Arm is in torque control mode with only gravity compensation
     *
     * @return int Returns #1 if the state transition was successful.
     *                  Returns #0 if the state transition failed due to
     *                  invalid target state, current state prevents transition,
     *                  hardware errors, or safety constraints.
     *
     * @pre The current state must allow transition to the target state.
     *      Some transitions may require specific conditions (e.g., all
     *      axes stopped, no active faults).
     *
     * @post If successful, Arm1 enters the specified runtime state with
     *       corresponding behavior changes. State-dependent functions
     *       will behave according to the new state.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetState(int state);
    /**
     * @brief Sets the joint position command for Arm1.
     *
     * This function commands Arm1 to move to specified joint positions (angles
     * for each joint). The command specifies the target positions for all
     * 7 joints of Arm1. The arm will move to these positions using the
     * currently configured motion settings.
     *
     * @param[in] joint_pos Array of 7 joint position values (double) in
     *                     degrees.
     *
     * @return int Returns #1 if the joint position command was
     *                  successfully accepted and queued for execution.
     *                  Returns #0 if the command was rejected.
     *
     * @post If successful, Arm1 begins moving to the specified joint positions.
     *       The motion occurs asynchronously; the function returns before
     *       motion is complete.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetJointPosCmd(double joint_pos[7]);
    /**
     * @brief Sets the joint torque command for Arm1.
     *
     * This function commands torque values for each of the 7 joints of Arm1.
     * The arm will apply the specified torques to each joint motor. This is
     * typically used for force control applications.
     *
     * @param[in] joint_tor Array of 7 joint torque values (double) in
     *                     Newton-meters (Nm).
     *
     * @return int Returns #1 if the joint torque command was
     *                  successfully accepted and sent to the joint controllers.
     *                  Returns #0 if the command was rejected.
     *
     * @note This function is for direct torque control, which is an advanced
     *       feature requiring proper configuration. The arm must typically be
     *       in a specific control mode (e.g., torque control mode) to accept
     *       torque commands. Use FX_L0_Arm1_Runtime_SetImpType() to set
     *       the appropriate mode.
     *
     * @post If successful, the joint controllers apply the specified torques.
     *       The torques are applied continuously until new torque commands
     *       are sent or the control mode is changed.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetJointTorCmd(double joint_tor[7]);
    /**
     * @brief Sets the force control parameters for Arm1 to apply a certain force in some direction.
     *
     * This function configures the force control parameters for Arm1's
     * end-effector or specified contact points. Force control enables
     * the arm to apply or regulate forces in Cartesian space,
     * typically used for assembly, polishing, or contact operations.
     *
     * @param[in] force_ctrl Array of 5 force control parameters (double).
     *                      The parameters typically specify:
     *                      - force_ctrl[0]: Direction of force in X direction under the arm base coordinate
     *                      - force_ctrl[1]: Direction of force in Y direction under the arm base coordinate
     *                      - force_ctrl[2]: Direction of force in Z direction under the arm base coordinate
     *                      - force_ctrl[3]: Magnitude of force (N)
     *                      - force_ctrl[4]: Maximum move distance along the force (mm)
     *
     * @return int Returns #1 if the force control parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Force control can apply significant forces. Incorrect parameters
     *          may cause damage to the arm, tooling, or workpiece. Always
     *          start with conservative force limits and monitor force feedback
     *          during operation.
     *
     * @pre Arm1 must be configured for force control mode. A force/torque
     *      sensor should be calibrated and operational. The end-effector
     *      should be properly characterized (mass, center of gravity).
     *
     * @post If successful, Arm1 will attempt to achieve the specified
     *       forces/torques at the end-effector. The control is typically
     *       continuous until new parameters are set or control mode changes.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetForceCtrl(double force_ctrl[5]);
    /**
     * @brief Sets the force control parameters for Arm1 to apply a certain torque in some direction.
     *
     * This function configures the force control parameters for Arm1's
     * end-effector or specified contact points. Torque control enables
     * the arm to apply or regulate torques in Cartesian space,
     * typically used for assembly, polishing, or contact operations.
     *
     * @param[in] torque_ctrl Array of 5 torque control parameters (double).
     *                      The parameters typically specify:
     *                      - torque_ctrl[0]: Direction of torque in X direction under the arm base coordinate
     *                      - torque_ctrl[1]: Direction of torque in Y direction under the arm base coordinate
     *                      - torque_ctrl[2]: Direction of torque in Z direction under the arm base coordinate
     *                      - torque_ctrl[3]: Magnitude of torque (N)
     *                      - torque_ctrl[4]: Maximum rotate angles along the force (rad)
     *
     * @return int Returns #1 if the torque control parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Torque control can apply significant torque. Incorrect parameters
     *          may cause damage to the arm, tooling, or workpiece. Always
     *          start with conservative force limits and monitor force feedback
     *          during operation.
     *
     * @pre Arm1 must be configured for force control mode. A force/torque
     *      sensor should be calibrated and operational. The end-effector
     *      should be properly characterized (mass, center of gravity).
     *
     * @post If successful, Arm1 will attempt to achieve the specified
     *       forces/torques at the end-effector. The control is typically
     *       continuous until new parameters are set or control mode changes.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetTorqueCtrl(double torque_ctrl[5]);
    /**
     * @brief Sets the velocity ratio (override) for Arm1.
     *
     * This function sets a scaling factor for Arm1's programmed velocities.
     * The vel_ratio parameter multiplies all commanded velocities, allowing
     * for runtime adjustment of motion speed without reprogramming paths.
     * This is commonly used for speed override during debugging, testing,
     * or operator control.
     *
     * @param[in] vel_ratio Velocity scaling factor as a multiplier.
     *                     Range is typically 1.0 to 100.0 (1% to 100% of
     *                     programmed speed).
     *
     * @return int Returns #1 if the velocity ratio was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @note The velocity ratio affects all subsequent motion commands dynamically.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetVelRatio(double vel_ratio);
    /**
     * @brief Sets the acceleration ratio (override) for Arm1.
     *
     * This function sets a scaling factor for Arm1's programmed accelerations
     * and decelerations. The acc_ratio parameter multiplies all commanded
     * acceleration values, allowing for runtime adjustment of motion
     * aggressiveness without reprogramming paths. This affects how quickly
     * the arm speeds up and slows down.
     *
     * @param[in] acc_ratio Acceleration scaling factor as a multiplier.
     *                     Range is typically 1.0 to 100.0 (1% to 100% of
     *                     programmed acceleration).
     *
     * @return int Returns #1 if the acceleration ratio was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @note The acceleration ratio affects all subsequent motion commands until
     *       changed. Lower ratios result in smoother, more gradual motions
     *       which may be desirable for delicate operations or payload stability.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetAccRatio(double acc_ratio);
    /**
     * @brief Sets the joint stiffness (K) gains for Arm1.
     *
     * This function configures the joint stiffness parameters (proportional
     * gains) for Arm1's impedance control. The stiffness values
     * determine how rigidly the joints maintain their commanded positions
     * against external forces. Higher K values make joints stiffer; lower
     * values make them more compliant.
     *
     * @param[in] k Array of 7 joint stiffness values (double) in appropriate
     *             units (typically Nm/rad).
     *
     * @return int Returns #1 if the joint stiffness parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Excessive stiffness can cause high current draw, mechanical
     *          resonance, and instability. Insufficient stiffness can cause
     *          poor position tracking and overshoot. Tune carefully, often
     *          starting with lower values and increasing gradually.
     *
     * @post If successful, the joint controllers use the new stiffness values
     *       for subsequent control cycles. The effect is immediate for
     *       impedance control.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetJointK(double k[7]);
    /**
     * @brief Sets the joint damping (D) gains for Arm1.
     *
     * This function configures the joint damping parameters (derivative
     * gains) for Arm1's impedance or position control. The damping values
     * determine how quickly velocity errors are corrected and help to
     * suppress oscillations and vibrations. Higher D values increase
     * damping; lower values reduce it.
     *
     * @param[in] d Array of 7 joint damping values (double) in appropriate
     *             units (typically Nm·s/rad).
     *
     * @return int Returns #1 if the joint damping parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Excessive damping can cause sluggish response, high current
     *          draw, and motor overheating. Insufficient damping can cause
     *          oscillations, overshoot, and instability. Damping should be
     *          tuned in conjunction with stiffness parameters.
     *
     * @post If successful, the joint controllers use the new damping values
     *       for subsequent control cycles. The effect is immediate for
     *       impedance control; for position control, it affects the
     *       derivative gain of the position loop.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetJointD(double d[7]);
    /**
     * @brief Sets the Cartesian stiffness (K) gains for Arm1.
     *
     * This function configures the Cartesian stiffness parameters (proportional
     * gains) for Arm1's Cartesian space impedance or position control. The
     * stiffness values determine how rigidly the end-effector maintains its
     * commanded position and orientation in Cartesian space (X, Y, Z, RX, RY, RZ, R0)
     * against external forces. Higher K values make the end-effector stiffer;
     * lower values make it more compliant.
     *
     * @param[in] k Array of 7 Cartesian stiffness values (double) in appropriate
     *             units (typically N/Deg for linear, Nm/Deg for rotational).
     *             The array elements typically correspond to:
     *             - k[0]: X-direction translational stiffness(N/mm)
     *             - k[1]: Y-direction translational stiffness(N/mm)
     *             - k[2]: Z-direction translational stiffness(N/mm)
     *             - k[3]: X-axis rotational stiffness(N*m/rad)
     *             - k[4]: Y-axis rotational stiffness(N*m/rad)
     *             - k[5]: Z-axis rotational stiffness(N*m/rad)
     *             - k[6]: Zero space stiffness(N*m/rad)
     *
     * @return int Returns #1 if the Cartesian stiffness parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Excessive Cartesian stiffness can cause high joint torques,
     *          especially near singularities or workspace boundaries. The
     *          effective joint stiffness varies with arm configuration.
     *          Always verify joint torque limits are not exceeded.
     *
     * @post If successful, the Cartesian controller uses the new stiffness
     *       values for subsequent control cycles. The effective joint-level
     *       stiffness will vary with arm configuration according to the
     *       Jacobian transpose transformation.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetCartK(double k[7]);
    /**
     * @brief Sets the Cartesian damping (D) gains for Arm1.
     *
     * This function configures the Cartesian damping parameters (derivative
     * gains) for Arm1's Cartesian space impedance or position control. The
     * damping values determine how quickly velocity errors are corrected in
     * Cartesian space (X, Y, Z, RX, RY, RZ, R0) and help to suppress oscillations
     * and vibrations at the end-effector. Higher D values increase damping;
     * lower values reduce it.
     *
     * @param[in] d Array of 7 Cartesian damping values (double) in appropriate
     *             units (typically N·s/m for linear, Nm·s/rad for rotational, or
     *             system-specific units). The array elements typically correspond to:
     *             - d[0]: X-direction translational damping (N·s/mm)
     *             - d[1]: Y-direction translational damping (N·s/mm)
     *             - d[2]: Z-direction translational damping (N·s/mm)
     *             - d[3]: X-axis rotational damping (Nm·s/rad)
     *             - d[4]: Y-axis rotational damping (Nm·s/rad)
     *             - d[5]: Z-axis rotational damping (Nm·s/rad)
     *             - d[6]: Zero space damping (Nm·s/rad)
     *
     * @return int Returns #1 if the Cartesian damping parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Excessive Cartesian damping can cause sluggish response and
     *          high joint torques during fast motions. Insufficient damping
     *          can cause end-effector oscillations. Damping should be tuned
     *          in coordination with Cartesian stiffness and considering the
     *          current arm configuration.
     *
     * @post If successful, the Cartesian controller uses the new damping
     *       values for subsequent control cycles. The effective joint-level
     *       damping will vary with arm configuration according to the
     *       Jacobian transpose transformation.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetCartD(double d[7]);
    /**
     * @brief Sets the tool offset in the tool frame for Arm1.
     *
     * This function configures the translational and rotational offset
     * (transformation) from the end-effector flange to the tool tip or
     * working point. The tool offset defines where the effective tool
     * frame is located relative to the robot's flange, affecting all
     * subsequent Cartesian commands and measurements.
     *
     * @param[in] k Array of 6 tool offset values (double) representing
     *             the transformation from flange frame to tool frame.
     *             The array elements are:
     *             - k[0]: X-axis translation offset (mm)
     *             - k[1]: Y-axis translation offset (mm)
     *             - k[2]: Z-axis translation offset (mm)
     *             - k[3]: Rotation about X-axis (Deg)
     *             - k[4]: Rotation about Y-axis (Deg)
     *             - k[5]: Rotation about Z-axis (Deg)
     *
     * @return int Returns #1 if the tool offset was successfully
     *                  set. Returns #0 if the operation failed.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetToolK(double k[6]);
    /**
     * @brief Sets the tool dynamics parameters for Arm1.
     *
     * This function configures the dynamic properties of the tool attached
     * to Arm1's end-effector. These parameters are used for accurate
     * dynamics compensation, gravity compensation, and proper control
     * performance. The tool dynamics are essential for applications
     * requiring precise force control or when using tools with significant mass.
     *
     * @param[in] d Array of 10 tool dynamics parameters (double) that
     *             describe the tool's mass, center of mass, and inertia tensor.
     *             The array elements are:
     *             - d[0]: Tool mass (kilograms)
     *             - d[1]: Center of mass X coordinate in tool frame (mm)
     *             - d[2]: Center of mass Y coordinate in tool frame (mm)
     *             - d[3]: Center of mass Z coordinate in tool frame (mm)
     *             - d[4]: Inertia tensor XX component (kg·m²)
     *             - d[5]: Inertia tensor XY component (kg·m²)
     *             - d[6]: Inertia tensor XZ component (kg·m²)
     *             - d[7]: Inertia tensor YY component (kg·m²)
     *             - d[8]: Inertia tensor YZ component (kg·m²)
     *             - d[9]: Inertia tensor ZZ component (kg·m²)
     *             The inertia tensor is defined relative to the center of mass.
     *
     * @return int Returns #1 if the tool dynamics parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Incorrect dynamics parameters can cause:
     *          - Poor control performance
     *          - Oscillations or instability
     *          - Excessive joint torques
     *          - Accelerated wear on components
     *          Always use measured or accurately calculated values.
     *
     * @post If successful, the robot's dynamic model incorporates the new
     *       tool parameters for all subsequent control cycles. This affects
     *       gravity compensation, model-based control, and feedforward terms.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetToolD(double d[10]);
    /**
     * @brief Sets the impedance control type for Arm1.
     *
     * This function configures the type of impedance control used for Arm1.
     * Impedance control defines the relationship between motion and force at
     * the end-effector, allowing the robot to behave as a mass-spring-damper
     * system. Different impedance types are suitable for different tasks
     * such as contact operations, assembly, or human interaction.
     *
     * @param[in] imp_type The impedance control type identifier. Valid values
     *                    are defined in the system's impedance type enumeration,
     *                    which typically includes:
     *                    - 1: Joint space impedance control
     *                    - 2: Cartesian space impedance control
     *                    - 3: Force control
     *
     * @return int Returns #1 if the impedance type was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @post If successful, Arm1 uses the specified impedance type for all
     *       subsequent control cycles. This affects the dynamic response
     *       to both commanded motions and external forces.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetImpType(int imp_type);
    /**
     * @brief Sets the drag teaching (hand guiding) mode type for Arm1.
     *
     * This function configures the type of drag teaching (hand guiding) mode
     * for Arm1. Drag teaching allows an operator to physically move the robot
     * arm to teach positions or paths, with the robot providing adjustable
     * assistance or resistance. Different drag types offer varying levels of
     * gravity compensation, damping, and control authority for different
     * teaching scenarios.
     *
     * @param[in] drag_type The drag teaching type identifier. Valid values
     *                     are defined in the system's drag type enumeration,
     *                     which typically includes:
     *                     - 1: Joint space drag teaching
     *                     - 2: Cartesian space drag teaching in X direction under arm base coordinate
     *                     - 3: Cartesian space drag teaching in Y direction under arm base coordinate
     *                     - 4: Cartesian space drag teaching in Z direction under arm base coordinate
     *                     - 5: Cartesian space drag teaching in rotation relative to TCP
     *
     * @return int Returns #1 if the drag type was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @post If successful, Arm1 enters the specified drag teaching mode.
     *       The robot becomes responsive to external forces according to
     *       the selected drag type. Motors may be powered but controlled
     *       to allow movement.
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetDragType(int drag_type);
    /**
     * @brief Initializes a trajectory for Arm1 with the specified number of points.
     *
     * This function initializes a trajectory buffer for Arm1 that can hold
     * the specified number of trajectory points. Trajectory points typically
     * include position, velocity, and acceleration information for each joint
     * or Cartesian coordinate. This function allocates or prepares the
     * necessary memory and data structures for subsequent trajectory operations.
     *
     * @param[in] point_num The number of trajectory points to allocate in the
     *                     trajectory buffer. This determines the maximum
     *                     number of points that can be added to the trajectory
     *                     before execution. Range is typically 5 to 5000.
     *
     * @return int Returns #1 if the trajectory was successfully
     *                  initialized with the specified number of points.
     *                  Returns #0 if the operation failed.
     *
     * @note Initializing a trajectory does not start execution. After
     *       initialization, points must be added using functions like
     *       FX_L0_Arm1_Runtime_SetTraj(). Finally, execution is
     *       started with FX_L0_Arm1_Runtime_RunTraj().
     *
     * @pre Arm1 should be in position control state that supports trajectory planning.
     *
     * @post If successful, a trajectory buffer is allocated and prepared
     *       to hold point_num trajectory points. The trajectory is in
     *       "initialized" state, ready to receive points but not yet
     *       executable.
     *
     * @see FX_L0_Arm1_Runtime_SetTraj(), FX_L0_Arm1_Runtime_RunTraj(),
     *      FX_L0_Arm1_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_InitTraj(int point_num);
    /**
     * @brief Sets a block of trajectory points for Arm1.
     *
     * This function sets a contiguous block of trajectory points for Arm1,
     * starting at the specified serial number. This allows efficient bulk
     * loading of trajectory data, particularly useful for pre-computed
     * trajectories or loading from external sources. Each point includes
     * 7 joints' positions and acceleration data for all joints coordinates.
     *
     * @param[in] serial The starting frame index (0-based) in the trajectory buffer
     *                  where the new points should be placed. Each frame contains
     *                  up to 50 points. If points already exist at these indices,
     *                  they will be overwritten.
     * @param[in] point_num The number of points to set. If it is not the last frame
     *                     to set, the point number must be 50.
     *                     The trajectory buffer must have been previously
     *                     initialized with FX_L0_Arm1_Runtime_InitTraj().
     * @param[in] point_data Pointer to a contiguous array of point data.
     *                      The array should contain point_num * 7 elements.
     *                      Data layout is typically:
     *                      {point0_data0, point0_data1, ..., point0_data6,
     *                       point1_data0, point1_data1, ..., point1_data6, ...}
     *
     * @return int Returns #1 if the trajectory points were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @see FX_L0_Arm1_Runtime_InitTraj(), FX_L0_Arm1_Runtime_RunTraj(),
     *      FX_L0_Arm1_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_SetTraj(int serial, int point_num, double *point_data);
    /**
     * @brief Starts execution of the loaded trajectory for Arm1.
     *
     * This function begins execution of the trajectory that has been previously
     * loaded and initialized. The arm will start moving through the trajectory
     * points according to the timing, velocity, and acceleration specifications
     * defined in the trajectory data. Execution continues until all points
     * are completed, the trajectory is stopped, or an error occurs.
     *
     * @return int Returns #1 if trajectory execution was successfully
     *                  started. Returns #0 if the operation failed.
     *
     * @note Trajectory execution is typically asynchronous; this function
     *       returns immediately after starting execution. The trajectory can be
     *       stopped using FX_L0_Arm1_Runtime_StopTraj().
     *
     * @pre A trajectory must be properly initialized via
     *      FX_L0_Arm1_Runtime_InitTraj() and populated with points via
     *      FX_L0_Arm1_Runtime_SetTraj().
     *
     * @post If successful, Arm1 begins moving through the trajectory points.
     *       The trajectory execution state becomes "running". Motors are
     *       enabled and controlled to follow the trajectory path.
     *
     * @see FX_L0_Arm1_Runtime_InitTraj(), FX_L0_Arm1_Runtime_SetTraj(),
     *      FX_L0_Arm1_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_RunTraj(void);
    /**
     * @brief Stops execution of the currently running trajectory for Arm1.
     *
     * This function immediately stops execution of the trajectory that is
     * currently running. The arm will decelerate to a stop according to
     * configured deceleration profiles or emergency stop parameters.
     * After stopping, the trajectory execution state is set to "stopped"
     * and the arm remains at its current position (or the position
     * reached after deceleration).
     *
     * @return int Returns #1 if the trajectory was successfully
     *                  stopped. Returns #0 if the operation failed.
     *
     * @post If successful, trajectory execution is terminated. The arm
     *       comes to a complete stop.
     *
     * @see FX_L0_Arm1_Runtime_InitTraj(), FX_L0_Arm1_Runtime_SetTraj(),
     *      FX_L0_Arm1_Runtime_RunTraj()
     */
    CONTROL_SDK_API int FX_L0_Arm1_Runtime_StopTraj(void);
    /**
     * @brief Immediately stops all motion and activates emergency stop for Head.
     *
     * This function triggers an emergency stop (E-stop) condition for the Head
     * robotic system. It immediately halts all servo movements, disables motors,
     * and applies brakes (if available) to bring Head to a complete stop as
     * quickly as possible. This function should be used in critical safety
     * situations to prevent damage or injury.
     *
     * @return int Returns #1 if the emergency stop was successfully
     *                  activated. Returns #0 if the emergency stop
     *                  failed to activate due to hardware faults, communication
     *                  errors, or because Head is already in an emergency stop
     *                  state.
     *
     * @note Once activated, the emergency stop condition typically requires
     *       manual intervention to reset. After calling this function, the
     *       system may need to be explicitly reset or re-enabled before normal
     *       operation can resume.
     *
     * @warning This function causes abrupt stopping which may cause mechanical
     *          stress, positioning errors, or dropped payloads. Use only in
     *          genuine emergency situations. Do not use for normal stopping
     *          procedures.
     *
     * @pre None. This function can be called at any time, even during motion.
     *
     * @post Head is in emergency stop state: motors are disabled, brakes are
     *       engaged (if available), and Head will go to ERROR state.
     */
    CONTROL_SDK_API int FX_L0_Head_Runtime_EmergencyStop(void);
    /**
     * @brief Sets the runtime state of the Head.
     *
     * This function transitions the Head controller to a specified runtime state,
     * controlling the overall operational mode and behavior of the head. State
     * changes may affect motion execution, error handling, and system responses.
     *
     * @param[in] state The desired runtime state for Head. Valid states are
     *                  defined in the HeadState enumeration, which typically includes:
     *                  - HEAD_STATE_IDLE: Head is disabled
     *                  - HEAD_STATE_POSITION: Head is in position control mode
     *
     * @return int Returns #1 if the state transition was successful.
     *                  Returns #0 if the state transition failed due to
     *                  invalid target state, current state prevents transition,
     *                  hardware errors, or safety constraints.
     *
     * @pre The current state must allow transition to the target state.
     *      Some transitions may require specific conditions (e.g., all
     *      axes stopped, no active faults).
     *
     * @post If successful, Head enters the specified runtime state with
     *       corresponding behavior changes. State-dependent functions
     *       will behave according to the new state.
     */
    CONTROL_SDK_API int FX_L0_Head_Runtime_SetState(int state);
    /**
     * @brief Sets the joint position command for Head.
     *
     * This function commands Head to move to specified joint positions (angles
     * for each joint). The command specifies the target positions for all
     * 3 joints of Head. The arm will move to these positions using the
     * currently configured motion settings.
     *
     * @param[in] joint_pos Array of 3 joint position values (double) in
     *                     degrees.
     *
     * @return int Returns #1 if the joint position command was
     *                  successfully accepted and queued for execution.
     *                  Returns #0 if the command was rejected.
     *
     * @post If successful, Head begins moving to the specified joint positions.
     *       The motion occurs asynchronously; the function returns before
     *       motion is complete.
     */
    CONTROL_SDK_API int FX_L0_Head_Runtime_SetJointPosCmd(double joint_pos[3]);
    /**
     * @brief Sets the velocity ratio (override) for Head.
     *
     * This function sets a scaling factor for Head's programmed velocities.
     * The vel_ratio parameter multiplies all commanded velocities, allowing
     * for runtime adjustment of motion speed without reprogramming paths.
     * This is commonly used for speed override during debugging, testing,
     * or operator control.
     *
     * @param[in] vel_ratio Velocity scaling factor as a multiplier.
     *                     Range is typically 1.0 to 100.0 (1% to 100% of
     *                     programmed speed).
     *
     * @return int Returns #1 if the velocity ratio was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @note The velocity ratio affects all subsequent motion commands dynamically.
     */
    CONTROL_SDK_API int FX_L0_Head_Runtime_SetVelRatio(double vel_ratio);
    /**
     * @brief Sets the acceleration ratio (override) for Head.
     *
     * This function sets a scaling factor for Head's programmed accelerations
     * and decelerations. The acc_ratio parameter multiplies all commanded
     * acceleration values, allowing for runtime adjustment of motion
     * aggressiveness without reprogramming paths. This affects how quickly
     * the arm speeds up and slows down.
     *
     * @param[in] acc_ratio Acceleration scaling factor as a multiplier.
     *                     Range is typically 1.0 to 100.0 (1% to 100% of
     *                     programmed acceleration).
     *
     * @return int Returns #1 if the acceleration ratio was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @note The acceleration ratio affects all subsequent motion commands until
     *       changed. Lower ratios result in smoother, more gradual motions
     *       which may be desirable for delicate operations or payload stability.
     */
    CONTROL_SDK_API int FX_L0_Head_Runtime_SetAccRatio(double acc_ratio);
    /**
     * @brief Immediately stops all motion and activates emergency stop for Body.
     *
     * This function triggers an emergency stop (E-stop) condition for the Body
     * robotic system. It immediately halts all servo movements, disables motors,
     * and applies brakes (if available) to bring Body to a complete stop as
     * quickly as possible. This function should be used in critical safety
     * situations to prevent damage or injury.
     *
     * @return int Returns #1 if the emergency stop was successfully
     *                  activated. Returns #0 if the emergency stop
     *                  failed to activate due to hardware faults, communication
     *                  errors, or because Body is already in an emergency stop
     *                  state.
     *
     * @note Once activated, the emergency stop condition typically requires
     *       manual intervention to reset. After calling this function, the
     *       system may need to be explicitly reset or re-enabled before normal
     *       operation can resume.
     *
     * @warning This function causes abrupt stopping which may cause mechanical
     *          stress, positioning errors, or dropped payloads. Use only in
     *          genuine emergency situations. Do not use for normal stopping
     *          procedures.
     *
     * @pre None. This function can be called at any time, even during motion.
     *
     * @post Body is in emergency stop state: motors are disabled, brakes are
     *       engaged (if available), and Body will go to ERROR state.
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_EmergencyStop(void);
    /**
     * @brief Sets the runtime state of the Body.
     *
     * This function transitions the Body controller to a specified runtime state,
     * controlling the overall operational mode and behavior of the body. State
     * changes may affect motion execution, error handling, and system responses.
     *
     * @param[in] state The desired runtime state for Body. Valid states are
     *                  defined in the BodyState enumeration, which typically includes:
     *                  - BODY_STATE_IDLE: Body is disabled
     *                  - BODY_STATE_POSITION: Body is in position control mode
     *                  - BODY_STATE_TORQUE: Body is in torque control mode with all force compensations
     *                  - BODY_STATE_RELEASE: Body is in torque control mode with only gravity compensation
     *
     * @return int Returns #1 if the state transition was successful.
     *                  Returns #0 if the state transition failed due to
     *                  invalid target state, current state prevents transition,
     *                  hardware errors, or safety constraints.
     *
     * @pre The current state must allow transition to the target state.
     *      Some transitions may require specific conditions (e.g., all
     *      axes stopped, no active faults).
     *
     * @post If successful, Body enters the specified runtime state with
     *       corresponding behavior changes. State-dependent functions
     *       will behave according to the new state.
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_SetState(int state);
    /**
     * @brief Sets the joint position command for Body.
     *
     * This function commands Body to move to specified joint positions (angles
     * for each joint). The command specifies the target positions for all
     * 6 joints of Body. The arm will move to these positions using the
     * currently configured motion settings.
     *
     * @param[in] joint_pos Array of 6 joint position values (double) in
     *                     degrees.
     *
     * @return int Returns #1 if the joint position command was
     *                  successfully accepted and queued for execution.
     *                  Returns #0 if the command was rejected.
     *
     * @post If successful, Body begins moving to the specified joint positions.
     *       The motion occurs asynchronously; the function returns before
     *       motion is complete.
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_SetJointPosCmd(double joint_pos[6]);
    /**
     * @brief Sets the velocity ratio (override) for Body.
     *
     * This function sets a scaling factor for Body's programmed velocities.
     * The vel_ratio parameter multiplies all commanded velocities, allowing
     * for runtime adjustment of motion speed without reprogramming paths.
     * This is commonly used for speed override during debugging, testing,
     * or operator control.
     *
     * @param[in] vel_ratio Velocity scaling factor as a multiplier.
     *                     Range is typically 1.0 to 100.0 (1% to 100% of
     *                     programmed speed).
     *
     * @return int Returns #1 if the velocity ratio was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @note The velocity ratio affects all subsequent motion commands dynamically.
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_SetVelRatio(double vel_ratio);
    /**
     * @brief Sets the acceleration ratio (override) for Body.
     *
     * This function sets a scaling factor for Body's programmed accelerations
     * and decelerations. The acc_ratio parameter multiplies all commanded
     * acceleration values, allowing for runtime adjustment of motion
     * aggressiveness without reprogramming paths. This affects how quickly
     * the arm speeds up and slows down.
     *
     * @param[in] acc_ratio Acceleration scaling factor as a multiplier.
     *                     Range is typically 1.0 to 100.0 (1% to 100% of
     *                     programmed acceleration).
     *
     * @return int Returns #1 if the acceleration ratio was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @note The acceleration ratio affects all subsequent motion commands until
     *       changed. Lower ratios result in smoother, more gradual motions
     *       which may be desirable for delicate operations or payload stability.
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_SetAccRatio(double acc_ratio);
    /**
     * @brief Sets the joint proportional (P) gains for Body.
     *
     * This function configures the joint stiffness parameters (proportional
     * gains) for Body's PD control. The stiffness values
     * determine how rigidly the joints maintain their commanded positions
     * against external forces. Higher K values make joints stiffer; lower
     * values make them more compliant.
     *
     * @param[in] p Array of 6 joint proportional gain values (double) in appropriate
     *             units (typically Nm/Deg).
     *
     * @return int Returns #1 if the joint stiffness parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Excessive proportional gain can cause high current draw, mechanical
     *          resonance, and instability. Insufficient proportional gain  can cause
     *          poor position tracking and overshoot. Tune carefully, often
     *          starting with lower values and increasing gradually.
     *
     * @post If successful, the joint controllers use the new proportional gain values
     *       for subsequent control cycles. The effect is immediate for
     *       PD control.
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_SetPDP(double p[6]);
    /**
     * @brief Sets the joint derivative (D) gains for Body.
     *
     * This function configures the joint derivative gain parameters (derivative
     * gains) for Body's PD or position control. The derivative gain values
     * determine how quickly velocity errors are corrected and help to
     * suppress oscillations and vibrations. Higher D values increase
     * damping; lower values reduce it.
     *
     * @param[in] d Array of 6 joint derivative gain values (double) in appropriate
     *             units (typically Nm·s/Deg).
     *
     * @return int Returns #1 if the joint derivative gain parameters were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @warning Excessive derivative gain can cause sluggish response, high current
     *          draw, and motor overheating. Insufficient derivative gain can cause
     *          oscillations, overshoot, and instability. Derivative gain should be
     *          tuned in conjunction with proportional parameters.
     *
     * @post If successful, the joint controllers use the new derivative gain values
     *       for subsequent control cycles. The effect is immediate for
     *       PD control; for position control, it affects the
     *       derivative gain of the position loop.
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_SetPDD(double d[6]);
    /**
     * @brief Initializes a trajectory for Body with the specified number of points.
     *
     * This function initializes a trajectory buffer for Body that can hold
     * the specified number of trajectory points. Trajectory points typically
     * include position, velocity, and acceleration information for each joint
     * or Cartesian coordinate. This function allocates or prepares the
     * necessary memory and data structures for subsequent trajectory operations.
     *
     * @param[in] point_num The number of trajectory points to allocate in the
     *                     trajectory buffer. This determines the maximum
     *                     number of points that can be added to the trajectory
     *                     before execution. Range is typically 5 to 5000.
     *
     * @return int Returns #1 if the trajectory was successfully
     *                  initialized with the specified number of points.
     *                  Returns #0 if the operation failed.
     *
     * @note Initializing a trajectory does not start execution. After
     *       initialization, points must be added using functions like
     *       FX_L0_Body_Runtime_SetTraj(). Finally, execution is
     *       started with FX_L0_Body_Runtime_RunTraj().
     *
     * @pre Body should be in position control state that supports trajectory planning.
     *
     * @post If successful, a trajectory buffer is allocated and prepared
     *       to hold point_num trajectory points. The trajectory is in
     *       "initialized" state, ready to receive points but not yet
     *       executable.
     *
     * @see FX_L0_Body_Runtime_SetTraj(), FX_L0_Body_Runtime_RunTraj(),
     *      FX_L0_Body_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_InitTraj(int point_num);
    /**
     * @brief Sets a block of trajectory points for Body.
     *
     * This function sets a contiguous block of trajectory points for Body,
     * starting at the specified serial number. This allows efficient bulk
     * loading of trajectory data, particularly useful for pre-computed
     * trajectories or loading from external sources. Each point includes
     * 6 joints' positions and acceleration data for all joints coordinates.
     *
     * @param[in] serial The starting frame index (0-based) in the trajectory buffer
     *                  where the new points should be placed. Each frame contains
     *                  up to 50 points. If points already exist at these indices,
     *                  they will be overwritten.
     * @param[in] point_num The number of points to set. If it is not the last frame
     *                     to set, the point number must be 50.
     *                     The trajectory buffer must have been previously
     *                     initialized with FX_L0_Body_Runtime_InitTraj().
     * @param[in] point_data Pointer to a contiguous array of point data.
     *                      The array should contain point_num * 6 elements.
     *                      Data layout is typically:
     *                      {point0_data0, point0_data1, ..., point0_data5,
     *                       point1_data0, point1_data1, ..., point1_data5, ...}
     *
     * @return int Returns #1 if the trajectory points were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @see FX_L0_Body_Runtime_InitTraj(), FX_L0_Body_Runtime_RunTraj(),
     *      FX_L0_Body_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_SetTraj(int serial, int point_num, double *point_data);
    /**
     * @brief Starts execution of the loaded trajectory for Body.
     *
     * This function begins execution of the trajectory that has been previously
     * loaded and initialized. The arm will start moving through the trajectory
     * points according to the timing, velocity, and acceleration specifications
     * defined in the trajectory data. Execution continues until all points
     * are completed, the trajectory is stopped, or an error occurs.
     *
     * @return int Returns #1 if trajectory execution was successfully
     *                  started. Returns #0 if the operation failed.
     *
     * @note Trajectory execution is typically asynchronous; this function
     *       returns immediately after starting execution. The trajectory can be
     *       stopped using FX_L0_Body_Runtime_StopTraj().
     *
     * @pre A trajectory must be properly initialized via
     *      FX_L0_Body_Runtime_InitTraj() and populated with points via
     *      FX_L0_Body_Runtime_SetTraj().
     *
     * @post If successful, Body begins moving through the trajectory points.
     *       The trajectory execution state becomes "running". Motors are
     *       enabled and controlled to follow the trajectory path.
     *
     * @see FX_L0_Body_Runtime_InitTraj(), FX_L0_Body_Runtime_SetTraj(),
     *      FX_L0_Body_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_RunTraj(void);
    /**
     * @brief Stops execution of the currently running trajectory for Body.
     *
     * This function immediately stops execution of the trajectory that is
     * currently running. The arm will decelerate to a stop according to
     * configured deceleration profiles or emergency stop parameters.
     * After stopping, the trajectory execution state is set to "stopped"
     * and the arm remains at its current position (or the position
     * reached after deceleration).
     *
     * @return int Returns #1 if the trajectory was successfully
     *                  stopped. Returns #0 if the operation failed.
     *
     * @post If successful, trajectory execution is terminated. The body
     *       comes to a complete stop.
     *
     * @see FX_L0_Body_Runtime_InitTraj(), FX_L0_Body_Runtime_SetTraj(),
     *      FX_L0_Body_Runtime_RunTraj()
     */
    CONTROL_SDK_API int FX_L0_Body_Runtime_StopTraj(void);
    /**
     * @brief Immediately stops all motion and activates emergency stop for Lift.
     *
     * This function triggers an emergency stop (E-stop) condition for the Lift
     * robotic system. It immediately halts all servo movements, disables motors,
     * and applies brakes (if available) to bring Lift to a complete stop as
     * quickly as possible. This function should be used in critical safety
     * situations to prevent damage or injury.
     *
     * @return int Returns #1 if the emergency stop was successfully
     *                  activated. Returns #0 if the emergency stop
     *                  failed to activate due to hardware faults, communication
     *                  errors, or because Lift is already in an emergency stop
     *                  state.
     *
     * @note Once activated, the emergency stop condition typically requires
     *       manual intervention to reset. After calling this function, the
     *       system may need to be explicitly reset or re-enabled before normal
     *       operation can resume.
     *
     * @warning This function causes abrupt stopping which may cause mechanical
     *          stress, positioning errors, or dropped payloads. Use only in
     *          genuine emergency situations. Do not use for normal stopping
     *          procedures.
     *
     * @pre None. This function can be called at any time, even during motion.
     *
     * @post Lift is in emergency stop state: motors are disabled, brakes are
     *       engaged (if available), and Lift will go to ERROR state.
     */
    CONTROL_SDK_API int FX_L0_Lift_Runtime_EmergencyStop(void);
    /**
     * @brief Sets the runtime state of the Lift.
     *
     * This function transitions the Lift controller to a specified runtime state,
     * controlling the overall operational mode and behavior of the lift. State
     * changes may affect motion execution, error handling, and system responses.
     *
     * @param[in] state The desired runtime state for Lift. Valid states are
     *                  defined in the LiftState enumeration, which typically includes:
     *                  - LIFT_STATE_IDLE: Lift is disabled
     *                  - LIFT_STATE_POSITION: Lift is in position control mode
     *
     * @return int Returns #1 if the state transition was successful.
     *                  Returns #0 if the state transition failed due to
     *                  invalid target state, current state prevents transition,
     *                  hardware errors, or safety constraints.
     *
     * @pre The current state must allow transition to the target state.
     *      Some transitions may require specific conditions (e.g., all
     *      axes stopped, no active faults).
     *
     * @post If successful, Lift enters the specified runtime state with
     *       corresponding behavior changes. State-dependent functions
     *       will behave according to the new state.
     */
    CONTROL_SDK_API int FX_L0_Lift_Runtime_SetState(int state);
    /**
     * @brief Sets the joint position command for Lift.
     *
     * This function commands Lift to move to specified joint positions (angles
     * for each joint). The command specifies the target positions for all
     * 2 joints of Lift. The arm will move to these positions using the
     * currently configured motion settings.
     *
     * @param[in] joint_pos Array of 2 joint position values (double) in
     *                     degrees.
     *
     * @return int Returns #1 if the joint position command was
     *                  successfully accepted and queued for execution.
     *                  Returns #0 if the command was rejected.
     *
     * @post If successful, Lift begins moving to the specified joint positions.
     *       The motion occurs asynchronously; the function returns before
     *       motion is complete.
     */
    CONTROL_SDK_API int FX_L0_Lift_Runtime_SetJointPosCmd(double joint_pos[2]);
    /**
     * @brief Sets the velocity ratio (override) for Lift.
     *
     * This function sets a scaling factor for Lift's programmed velocities.
     * The vel_ratio parameter multiplies all commanded velocities, allowing
     * for runtime adjustment of motion speed without reprogramming paths.
     * This is commonly used for speed override during debugging, testing,
     * or operator control.
     *
     * @param[in] vel_ratio Velocity scaling factor as a multiplier.
     *                     Range is typically 1.0 to 100.0 (1% to 100% of
     *                     programmed speed).
     *
     * @return int Returns #1 if the velocity ratio was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @note The velocity ratio affects all subsequent motion commands dynamically.
     */
    CONTROL_SDK_API int FX_L0_Lift_Runtime_SetVelRatio(double vel_ratio);
    /**
     * @brief Sets the acceleration ratio (override) for Lift.
     *
     * This function sets a scaling factor for Lift's programmed accelerations
     * and decelerations. The acc_ratio parameter multiplies all commanded
     * acceleration values, allowing for runtime adjustment of motion
     * aggressiveness without reprogramming paths. This affects how quickly
     * the arm speeds up and slows down.
     *
     * @param[in] acc_ratio Acceleration scaling factor as a multiplier.
     *                     Range is typically 1.0 to 100.0 (1% to 100% of
     *                     programmed acceleration).
     *
     * @return int Returns #1 if the acceleration ratio was successfully
     *                  set. Returns #0 if the operation failed.
     *
     * @note The acceleration ratio affects all subsequent motion commands until
     *       changed. Lower ratios result in smoother, more gradual motions
     *       which may be desirable for delicate operations or payload stability.
     */
    CONTROL_SDK_API int FX_L0_Lift_Runtime_SetAccRatio(double acc_ratio);
    /**
     * @brief Initializes a trajectory for Lift with the specified number of points.
     *
     * This function initializes a trajectory buffer for Lift that can hold
     * the specified number of trajectory points. Trajectory points typically
     * include position, velocity, and acceleration information for each joint
     * or Cartesian coordinate. This function allocates or prepares the
     * necessary memory and data structures for subsequent trajectory operations.
     *
     * @param[in] point_num The number of trajectory points to allocate in the
     *                     trajectory buffer. This determines the maximum
     *                     number of points that can be added to the trajectory
     *                     before execution. Range is typically 5 to 5000.
     *
     * @return int Returns #1 if the trajectory was successfully
     *                  initialized with the specified number of points.
     *                  Returns #0 if the operation failed.
     *
     * @note Initializing a trajectory does not start execution. After
     *       initialization, points must be added using functions like
     *       FX_L0_Lift_Runtime_SetTraj(). Finally, execution is
     *       started with FX_L0_Lift_Runtime_RunTraj().
     *
     * @pre Lift should be in position control state that supports trajectory planning.
     *
     * @post If successful, a trajectory buffer is allocated and prepared
     *       to hold point_num trajectory points. The trajectory is in
     *       "initialized" state, ready to receive points but not yet
     *       executable.
     *
     * @see FX_L0_Lift_Runtime_SetTraj(), FX_L0_Lift_Runtime_RunTraj(),
     *      FX_L0_Lift_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Lift_Runtime_InitTraj(int point_num);
    /**
     * @brief Sets a block of trajectory points for Body.
     *
     * This function sets a contiguous block of trajectory points for Lift,
     * starting at the specified serial number. This allows efficient bulk
     * loading of trajectory data, particularly useful for pre-computed
     * trajectories or loading from external sources. Each point includes
     * 2 joints' positions and acceleration data for all joints coordinates.
     *
     * @param[in] serial The starting frame index (0-based) in the trajectory buffer
     *                  where the new points should be placed. Each frame contains
     *                  up to 50 points. If points already exist at these indices,
     *                  they will be overwritten.
     * @param[in] point_num The number of points to set. If it is not the last frame
     *                     to set, the point number must be 50.
     *                     The trajectory buffer must have been previously
     *                     initialized with FX_L0_Lift_Runtime_InitTraj().
     * @param[in] point_data Pointer to a contiguous array of point data.
     *                      The array should contain point_num * 6 elements.
     *                      Data layout is typically:
     *                      {point0_data0, point0_data1,
     *                       point1_data0, point1_data1,...}
     *
     * @return int Returns #1 if the trajectory points were
     *                  successfully set. Returns #0 if the operation
     *                  failed.
     *
     * @see FX_L0_Lift_Runtime_InitTraj(), FX_L0_Lift_Runtime_RunTraj(),
     *      FX_L0_Lift_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Lift_Runtime_SetTraj(int serial, int point_num, double *point_data);
    /**
     * @brief Starts execution of the loaded trajectory for Lift.
     *
     * This function begins execution of the trajectory that has been previously
     * loaded and initialized. The arm will start moving through the trajectory
     * points according to the timing, velocity, and acceleration specifications
     * defined in the trajectory data. Execution continues until all points
     * are completed, the trajectory is stopped, or an error occurs.
     *
     * @return int Returns #1 if trajectory execution was successfully
     *                  started. Returns #0 if the operation failed.
     *
     * @note Trajectory execution is typically asynchronous; this function
     *       returns immediately after starting execution. The trajectory can be
     *       stopped using FX_L0_Lift_Runtime_StopTraj().
     *
     * @pre A trajectory must be properly initialized via
     *      FX_L0_Lift_Runtime_InitTraj() and populated with points via
     *      FX_L0_Lift_Runtime_SetTraj().
     *
     * @post If successful, Lift begins moving through the trajectory points.
     *       The trajectory execution state becomes "running". Motors are
     *       enabled and controlled to follow the trajectory path.
     *
     * @see FX_L0_LiftRuntime_InitTraj(), FX_L0_Lift_Runtime_SetTraj(),
     *      FX_L0_Lift_Runtime_StopTraj()
     */
    CONTROL_SDK_API int FX_L0_Lift_Runtime_RunTraj(void);
    /**
     * @brief Stops execution of the currently running trajectory for Lift.
     *
     * This function immediately stops execution of the trajectory that is
     * currently running. The arm will decelerate to a stop according to
     * configured deceleration profiles or emergency stop parameters.
     * After stopping, the trajectory execution state is set to "stopped"
     * and the arm remains at its current position (or the position
     * reached after deceleration).
     *
     * @return int Returns #1 if the trajectory was successfully
     *                  stopped. Returns #0 if the operation failed.
     *
     * @post If successful, trajectory execution is terminated. The lift
     *       comes to a complete stop.
     *
     * @see FX_L0_Lift_Runtime_InitTraj(), FX_L0_Lift_Runtime_SetTraj(),
     *      FX_L0_Lift_Runtime_RunTraj()
     */
    CONTROL_SDK_API int FX_L0_Lift_Runtime_StopTraj(void);
    /**
     * @brief Retrieves a constant pointer to the robot runtime data structure.
     *
     * This function returns a read-only pointer to the ROBOT_RT
     * data structure, which contains comprehensive runtime information about
     * the robot's current state, including joint positions, velocities,
     * torques, system status, errors, and other real-time operational data.
     *
     * @return const ROBOT_RT* A constant pointer to the ROBOT_RT structure.
     *         Returns NULL if the robot runtime data is not available,
     *         the system is not initialized, or an error occurred.
     */
    CONTROL_SDK_API const ROBOT_RT *FX_L0_GetRobotRT(void);
    /**
     * @brief Retrieves a constant pointer to the robot non-runtime data structure.
     *
     * This function returns a read-only pointer to the ROBOT_SG
     * data structure, which contains comprehensive runtime information about
     * the robot's current state, including joint positions, velocities,
     * torques, system status, errors, and other real-time operational data.
     *
     * @return const ROBOT_SG* A constant pointer to the ROBOT_SG structure.
     *         Returns NULL if the robot runtime data is not available,
     *         the system is not initialized, or an error occurred.
     */
    CONTROL_SDK_API const ROBOT_SG *FX_L0_GetRobotSG(void);

#ifdef __cplusplus
}
#endif

#endif /* FX_L0ROBOT_H_ */
