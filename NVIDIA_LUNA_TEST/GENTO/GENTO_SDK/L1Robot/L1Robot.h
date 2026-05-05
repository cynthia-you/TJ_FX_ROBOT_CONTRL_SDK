#ifndef FX_L1ROBOT_H_
#define FX_L1ROBOT_H_
#include <stdio.h>
#include <cassert>
#include "L0Robot.h"
#include "FXCommon.h"
#include "L0Kinematics.h"
#include "FXFileClient.h"

// #include "L0KinematicsSDK/FxKineIF.h"

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
    /**
     * @brief Establish a udp link to a remote robot
     *
     * This high-level function establishes a complete link to a remote robot with the specified
     * IP address. In addition to basic network connectivity, it performs version negotiation and
     * enables/disables logging for the link session. This function builds upon lower-level connection
     * functions to provide a comprehensive link establishment service.
     *
     * @param ip1 First octet of the target system's IP address
     * @param ip2 Second octet of the target system's IP address
     * @param ip3 Third octet of the target system's IP address
     * @param ip4 Fourth octet of the target system's IP address
     * @param[out] version Pointer to an integer that will receive the negotiated protocol version
     *                     with the remote system. Set to the agreed-upon version number on success.
     * @param logSwitch Logging control flag for the link session:
     *                  - 0: Disable logging for this link
     *                  - 1: Enable logging for this link
     *                  - Other values may have implementation-specific meanings
     *
     * @return int Link establishment result:
     *         - Positive values: Link established successfully, indicate network latency
     *         - Negative values: Error codes:
     *           -1: Ports is occupied
     *           -2: No response in 1000 ms
     *           -3: Invalid IP address
     *
     * @note This is an L1 (layer 1) function indicating higher-level abstraction than L0 functions
     * @note The function handles network connection, protocol negotiation, and session setup
     * @note Logging configuration applies only to this specific link session
     */
    FX_L1_SDK_API int FX_L1_System_Link(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4, int *version, unsigned int logSwitch);
    /**
     * @brief Reboot the robot system
     *
     * This function initiates a system reboot process. It performs a controlled shutdown of system
     * services and applications, followed by a restart of the entire system. The reboot may be
     * immediate or scheduled based on system configuration and current state.
     *
     * @return int Reboot initiation result:
     *         - 1: Reboot command accepted and will be executed
     *         - 0: Reboot cannot be performed at this time
     *
     * @note This function initiates the reboot process but may not complete it before returning
     * @note The actual reboot may occur after a delay to allow for proper shutdown procedures
     */
    FX_L1_SDK_API int FX_L1_System_Reboot();
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
     * @see FX_L1_Comm_Send() Send data through the communication interface
     * @see FX_L1_Comm_SendAndWait() Send data through the communication interface
     * @see and wait until the data has been processed by controller.
     */
    FX_L1_SDK_API int FX_L1_Comm_Clear(unsigned int timeout);
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
     * @see FX_L1_Comm_Clear() Clear the communication buffer and prepare for new communication
     * @see FX_L1_Comm_SendAndWait() Send data through the communication interface
     * and wait for a response.
     */
    FX_L1_SDK_API int FX_L1_Comm_Send();
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
     * @see FX_L1_Comm_Clear() Clear the communication buffer and prepare for new communication
     * @see FX_L1_Comm_Send() Send data through the communication interface.
     */
    FX_L1_SDK_API int FX_L1_Comm_SendAndWait(unsigned int timeout);
    /**
     * @brief Get the degrees of freedom for a control object
     *
     * This function retrieves the number of degrees of freedom (DOF) for a specified control object.
     * Degrees of freedom represent the independent parameters that define the configuration or state
     * of a control object, typically used in robotics, motion control, or simulation systems.
     *
     * @param obj_type Type identifier of the control object. Valid values are defined in the
     *                 FXObjType enumeration.
     *
     * @return int The degrees of freedom for the specified control object:
     *         - Positive values: Number of degrees of freedom (e.g., 1, 2, 3, 6, etc.)
     *         - 0: Object has no controllable degrees of freedom
     *         - Negative values: Error codes:
     *           -1: Invalid object type
     *
     * @note The DOF count is acquired after FX_L1_System_Link() is called.
     */
    FX_L1_SDK_API int FX_L1_Fbk_GetCtrlObjDof(FXObjType obj_type);
    /**
     * @brief Get the current state of a control object
     *
     * This function retrieves the current operational state of a specified control object.
     * The state represents the object's mode of operation, availability, or readiness status,
     * which is essential for system monitoring, fault detection, and state-based control logic.
     *
     * @param obj_type Type identifier of the control object. Valid values are defined in the
     *                 FXObjType enumeration.
     *
     * @return FXStateType The current state of the specified control object. The return value
     *         is from the FXStateType enumeration.
     *
     * @note State transitions are controlled by separate state machine functions
     * @note The state may affect what operations are permitted on the object
     */
    FX_L1_SDK_API FXStateType FX_L1_Fbk_CurrentState(FXObjType obj_type);
    /**
     * @brief Get servo error codes for a control object
     *
     * This function retrieves detailed error codes from the servo system associated with a
     * specified control object. Servo errors typically represent faults in motion control
     * systems, such as overloads, communication failures, temperature issues, or feedback
     * errors.
     *
     * @param obj_type Type identifier of the control object. Valid values are defined in the
     *                 FXObjType enumeration.
     *
     * @param[out] error_code Array of 7 integers that will be populated with servo error codes.
     *                        Each element represents the error code for an axis of the control object.
     *
     * @return int Operation result:
     *         - 1: Error codes successfully retrieved and array populated
     *         - 0: Failed to retrieve error codes, array contents are undefined
     *
     * @note The function may return 1 even if no errors are present (all codes may be 0)
     * @note The meaning of specific error codes depends on the servo drive manufacturer and model
     */
    FX_L1_SDK_API int FX_L1_State_GetServoErrorCode(FXObjType obj_type, int error_code[7]);
    /**
     * @brief Reset errors for multiple objects
     *
     * This function resets or clears error states for a set of control objects specified
     * by a bitmask. It attempts to clear error conditions, acknowledge faults, and restore
     * the objects to an operational state. The function returns a bitmask indicating which
     * objects were successfully reset.
     *
     * @param obj_mask Bitmask specifying which objects to reset. Each bit corresponds to
     *                 a control object or object group:
     *                 - Bit 0: Left arm
     *                 - Bit 1: Right arm
     *                 - Bit 2: Head
     *                 - Bit 3: Body
     *                 - Bit 4: Lift
     *                 Set a bit to 1 to attempt reset of that object, 0 to skip it.
     *
     * @return unsigned int Bitmask indicating reset results:
     *         - Each bit set to 1 indicates the corresponding object was successfully reset
     *         - Each bit set to 0 indicates the corresponding object could not be reset
     *         - The returned mask is a subset of the input obj_mask
     *
     * @note This function attempts to clear error states but may not fix the underlying cause
     * @note Some errors may require manual intervention before they can be reset
     * @note The function may have different effects depending on error type (soft vs. hard errors)
     */
    FX_L1_SDK_API unsigned int FX_L1_State_ResetError(unsigned int obj_mask);
    /**
     * @brief Transition a control object to idle state
     *
     * This function attempts to transition a specified control object from its current state
     * to an idle state. The idle state typically represents a low-power, ready condition where
     * the object is initialized and available but not actively performing its primary function.
     * The transition is performed within a specified timeout period.
     *
     * @param obj_type Type identifier of the control object to transition to idle.
     *                 This should be an object capable of state transitions, such as:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *                 - FX_OBJ_LIFT: Lift
     *
     * @param timeout Maximum time in milliseconds allowed for the transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @return int Transition result:
     *         - 0: Operation success
     *         - Negative values: Error codes:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToIdle(FXObjType obj_type, unsigned int timeout);
    /**
     * @brief Transition a control object to position control mode
     *
     * This function transitions a specified control object to position control mode with
     * specified motion profile parameters. In position mode, the object's controller regulates
     * the position according to commanded target positions, using the provided velocity and
     * acceleration limits for motion planning.
     *
     * @param obj_type Type identifier of the control object to transition to position mode.
     *                 This should be a position-controllable object, typically:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *                 - FX_OBJ_LIFT: Lift
     *
     * @param timeout Maximum time in milliseconds allowed for the mode transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @param vel_ratio Velocity scaling factor to apply. Valid range is typically 1.0 to 100.0
     *
     * @param acc_ratio Acceleration scaling factor to apply. Valid range is typically 1.0 to 100.0
     *
     * @return int Transition result:
     *         - 0: Operation success
     *         - Negative values: Error codes:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToPositionMode(FXObjType obj_type, unsigned int timeout, double vel_ratio, double acc_ratio);
    /**
     * @brief Transition a control object to impedance control mode for joints
     *
     * This function transitions a specified control object to impedance control mode for
     * joint-level control. Impedance control regulates the dynamic relationship between
     * position and force, allowing the object to exhibit desired stiffness and damping
     * characteristics. The mode is configured with velocity/acceleration limits and
     * impedance parameters for multiple degrees of freedom.
     *
     * @param obj_type Type identifier of the control object to transition to impedance mode.
     *                 This should be a multi-joint mechanism capable of impedance control:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_BODY: Body
     *
     * @param timeout Maximum time in milliseconds allowed for the mode transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @param vel_ratio Velocity scaling factor to apply. Valid range is typically 1.0 to 100.0
     *
     * @param acc_ratio Acceleration scaling factor to apply. Valid range is typically 1.0 to 100.0
     *
     * @param k Array of 7 stiffness coefficients in appropriate units (e.g., Nm/rad, N/m).
     *          Each element corresponds to a joint or degree of freedom:
     *          - k[0]: Stiffness for joint/DOF 0
     *          - k[1]: Stiffness for joint/DOF 1
     *          - ...
     *          - k[6]: Stiffness for joint/DOF 6
     *          A value of 0 indicates no stiffness (free motion) for that DOF.
     *
     * @param d Array of 7 damping coefficients in appropriate units (e.g., Nm·s/rad, N·s/m).
     *          Each element corresponds to a joint or degree of freedom:
     *          - d[0]: Damping for joint/DOF 0
     *          - d[1]: Damping for joint/DOF 1
     *          - ...
     *          - d[6]: Damping for joint/DOF 6
     *          A value of 0 indicates no damping (potentially underdamped) for that DOF.
     *
     * @return int Transition result:
     *         - 0: Operation success
     *         - Negative values: Error codes:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     *
     * @note Impedance control enables compliant interaction with the environment
     * @note The k and d parameters define the desired mechanical impedance
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToImpJointMode(FXObjType obj_type, unsigned int timeout, double vel_ratio, double acc_ratio, double k[7], double d[7]);
    /**
     * @brief Transition a control object to impedance control mode in Cartesian space
     *
     * This function transitions a specified control object to impedance control mode
     * operating in Cartesian (task) space. Cartesian impedance control regulates the
     * dynamic relationship between end-effector pose and applied forces/torques,
     * allowing compliant behavior in the operational space. The mode is configured
     * with velocity/acceleration limits and impedance parameters for Cartesian axes.
     *
     * @param obj_type Type identifier of the control object to transition to Cartesian impedance mode.
     *                 This should be a robotic manipulator or mechanism capable of Cartesian control:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param timeout Maximum time in milliseconds allowed for the mode transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @param vel_ratio Velocity scaling factor to apply. Valid range is typically 1.0 to 100.0
     *
     * @param acc_ratio Acceleration scaling factor to apply. Valid range is typically 1.0 to 100.0
     *
     * @param k Array of 7 stiffness coefficients in appropriate units (typically N/m for linear,
     *          Nm/rad for rotational). Each element corresponds to a Cartesian degree of freedom:
     *          - k[0]: Stiffness in X direction
     *          - k[1]: Stiffness in Y direction
     *          - k[2]: Stiffness in Z direction
     *          - k[3]: Rotational stiffness about X
     *          - k[4]: Rotational stiffness about Y
     *          - k[5]: Rotational stiffness about Z
     *          - k[6]: Stiffness about zero space
     *          A value of 0 indicates free motion in that direction.
     *
     * @param d Array of 7 damping coefficients in appropriate units (typically N·s/m for linear,
     *          Nm·s/rad for rotational). Each element corresponds to a Cartesian degree of freedom:
     *          - d[0]: Damping in X direction
     *          - d[1]: Damping in Y direction
     *          - d[2]: Damping in Z direction
     *          - d[3]: Rotational damping about X
     *          - d[4]: Rotational damping about Y
     *          - d[5]: Rotational damping about Z
     *          - d[6]: Damping about zero space
     *          A value of 0 indicates no damping in that direction.
     *
     * @return int Transition result:
     *         - 0: Operation success
     *         - Negative values: Error codes:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     *
     * @note Cartesian impedance control enables compliant interaction at the end-effector
     * @note The impedance parameters define the desired end-effector compliance
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToImpCartMode(FXObjType obj_type, unsigned int timeout, double vel_ratio, double acc_ratio, double k[7], double d[7]);
    /**
     * @brief Transition a control object to impedance-force hybrid control mode
     *
     * This function transitions a specified control object to a hybrid control mode that
     * combines impedance control with force/torque regulation. This mode allows the object
     * to maintain desired force/torque setpoints while exhibiting compliant behavior
     * in other directions. The mode is configured with force and torque control parameters
     * for multiple degrees of freedom.
     *
     * @param obj_type Type identifier of the control object to transition to impedance-force mode.
     *                 This should be a device capable of both force/torque control and compliant motion:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param timeout Maximum time in milliseconds allowed for the mode transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @param force_ctrl Array of force control parameters for Cartesian directions.
     *                   The array size is FX_FORCE_DEF_NUM (typically 3 for X, Y, Z directions).
     *                   Each element contains control parameters for a specific direction:
     *                     - force_ctrl[0]: Direction of force in X direction under the arm base coordinate
     *                     - force_ctrl[1]: Direction of force in Y direction under the arm base coordinate
     *                     - force_ctrl[2]: Direction of force in Z direction under the arm base coordinate
     *                     - force_ctrl[3]: Magnitude of force (N)
     *                     - force_ctrl[4]: Maximum move distance along the force (mm)
     *
     * @param torque_ctrl Array of torque control parameters for rotational directions.
     *                    The array size is FX_TORQUE_DEF_NUM (typically 3 for Rx, Ry, Rz rotations).
     *                    Each element contains control parameters for a specific rotation:
     *                     - torque_ctrl[0]: Direction of torque in X direction under the arm base coordinate
     *                     - torque_ctrl[1]: Direction of torque in Y direction under the arm base coordinate
     *                     - torque_ctrl[2]: Direction of torque in Z direction under the arm base coordinate
     *                     - torque_ctrl[3]: Magnitude of torque (Nm)
     *                     - torque_ctrl[4]: Maximum rotate angles along the torque (rad)
     *
     * @return int Transition result:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     *
     * @note This mode enables tasks requiring both force regulation and compliant motion
     * @note FX_FORCE_DEF_NUM and FX_TORQUE_DEF_NUM are system-defined constants
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToImpForceMode(FXObjType obj_type, unsigned int timeout, double force_ctrl[FX_FORCE_DEF_NUM], double torque_ctrl[FX_TORQUE_DEF_NUM]);
    /**
     * @brief Transition a control object to joint-space drag teaching mode
     *
     * This function transitions a specified control object to joint-space drag teaching mode,
     * also known as gravity compensation mode or backdrivable mode. In this mode, the object's
     * joints become compliant and can be manually moved by an operator, with the control system
     * compensating for gravity and friction. This mode is typically used for manual teaching,
     * path recording, or intuitive programming of robotic systems.
     *
     * @param obj_type Type identifier of the control object to transition to drag teaching mode.
     *                 This should be a robotic manipulator or multi-joint mechanism that supports
     *                 backdrivability and gravity compensation:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param timeout Maximum time in milliseconds allowed for the mode transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @return int Transition result:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     *
     * @note In drag teaching mode, the object can be manually positioned for teaching purposes
     * @note The system typically compensates for gravity, friction, and inertia
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToDragJoint(FXObjType obj_type, unsigned int timeout);
    /**
     * @brief Transition a control object to Cartesian-space X-axis drag teaching mode
     *
     * This function transitions a specified control object to a specialized drag teaching mode
     * where only movement along the Cartesian X-axis is compliant and backdrivable, while other
     * degrees of freedom remain stiff or position-controlled. This mode is useful for teaching
     * linear motions along a specific axis while maintaining precise control in other directions.
     *
     * @param obj_type Type identifier of the control object to transition to Cartesian X-axis drag mode.
     *                 This should be a robotic device capable of Cartesian control and selective compliance:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param timeout Maximum time in milliseconds allowed for the mode transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @return int Transition result:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     *
     * @note Only the X-axis direction is compliant; Y, Z, and rotations remain position-controlled
     * @note Useful for teaching linear paths, pick-and-place operations, or assembly tasks
     * @note The X-axis direction is defined in the base coordinate system
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToDragCartX(FXObjType obj_type, unsigned int timeout);
    /**
     * @brief Transition a control object to Cartesian-space Y-axis drag teaching mode
     *
     * This function transitions a specified control object to a specialized drag teaching mode
     * where only movement along the Cartesian Y-axis is compliant and backdrivable, while other
     * degrees of freedom remain stiff or position-controlled. This mode is useful for teaching
     * linear motions along a specific axis while maintaining precise control in other directions.
     *
     * @param obj_type Type identifier of the control object to transition to Cartesian Y-axis drag mode.
     *                 This should be a robotic device capable of Cartesian control and selective compliance:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param timeout Maximum time in milliseconds allowed for the mode transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @return int Transition result:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     *
     * @note Only the Y-axis direction is compliant; X, Z, and rotations remain position-controlled
     * @note Useful for teaching linear paths, pick-and-place operations, or assembly tasks
     * @note The Y-axis direction is defined in the base coordinate system
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToDragCartY(FXObjType obj_type, unsigned int timeout);
    /**
     * @brief Transition a control object to Cartesian-space Z-axis drag teaching mode
     *
     * This function transitions a specified control object to a specialized drag teaching mode
     * where only movement along the Cartesian Z-axis is compliant and backdrivable, while other
     * degrees of freedom remain stiff or position-controlled. This mode is useful for teaching
     * linear motions along a specific axis while maintaining precise control in other directions.
     *
     * @param obj_type Type identifier of the control object to transition to Cartesian Z-axis drag mode.
     *                 This should be a robotic device capable of Cartesian control and selective compliance:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param timeout Maximum time in milliseconds allowed for the mode transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @return int Transition result:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     *
     * @note Only the Z-axis direction is compliant; X, Y, and rotations remain position-controlled
     * @note Useful for teaching linear paths, pick-and-place operations, or assembly tasks
     * @note The Z-axis direction is defined in the base coordinate system
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToDragCartZ(FXObjType obj_type, unsigned int timeout);
    /**
     * @brief Transition a control object to Cartesian-space posture drag teaching mode
     *
     * This function transitions a specified control object to a specialized drag teaching mode
     * where only rotation around the current position is compliant and backdrivable, while other
     * degrees of freedom remain stiff or position-controlled. This mode is useful for teaching
     * rotation motions along a specific axis while maintaining precise control in other directions.
     *
     * @param obj_type Type identifier of the control object to transition to Cartesian rotation drag mode.
     *                 This should be a robotic device capable of Cartesian control and selective compliance:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param timeout Maximum time in milliseconds allowed for the mode transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @return int Transition result:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     *
     * @note Only the Rx/Ry/Rz direction is compliant; X, Y, and Z remain position-controlled
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToDragCartR(FXObjType obj_type, unsigned int timeout);
    /**
     * @brief Transition a control object to collaborative release mode
     *
     * This function transitions a specified control object to collaborative release mode,
     * a safety-oriented state where the system minimizes resistance to allow safe human
     * intervention or manual repositioning. In this mode, the control system reduces
     * or removes motor torques, potentially enabling brakes or other safety mechanisms
     * to facilitate safe human-robot collaboration (HRC) operations.
     *
     * @param obj_type Type identifier of the control object to transition to collaborative release mode.
     *                 This should be a collaborative robot or safety-rated device designed for HRC:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param timeout Maximum time in milliseconds allowed for the mode transition to complete.
     *                The system will attempt to complete the transition within this period:
     *                - 0: Immediate transition attempt (don't wait to check the switching result)
     *                - >0: Allow up to specified time for graceful transition
     *
     * @return int Transition result:
     *           -1: Failed to get ready to pack command
     *           -2: Failed to send command
     *           -3: Packing command failed, the command list maybe too long in one frame
     *           -4: Invalid object type
     *           -5: There is no execution response in time
     *           -6: Current state is not allowed to transfer to target state
     *           -7: Control object is moving, not allowed to transfer state
     *           -8: Invalid input parameters
     */
    FX_L1_SDK_API int FX_L1_State_SwitchToCollaborativeRelease(FXObjType obj_type, unsigned int timeout);
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
     * @param[in] value The new integer value to assign to the parameter.
     *
     * @return int Returns #1 if the parameter was successfully set
     *                  to the target value. Returns #0 if the operation
     *                  failed due to invalid name.
     *
     * @warning The name buffer must be properly null-terminated. Passing a
     *          non-null-terminated string or a string longer than 29 characters
     *          (excluding null terminator) may cause undefined behavior.
     */
    FX_L1_SDK_API int FX_L1_Param_SetInt32(char *name, int value);
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
     * @param[in] value The new floating-point value to assign to the parameter.
     *
     * @return int Returns #1 if the parameter was successfully set
     *                  to the target value. Returns #0 if the operation
     *                  failed due to invalid name, insufficient permissions,
     *                  write protection, storage failure, or other errors.
     *
     * @warning The name buffer must be properly null-terminated. Passing a
     *          non-null-terminated string or a string longer than 29 characters
     *          (excluding null terminator) may cause undefined behavior.
     */
    FX_L1_SDK_API int FX_L1_Param_SetFloat(char *name, float value);
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
     * @param[out] value Pointer to an int variable where the parameter
     *                   value will be stored if found. The pointer must not be NULL.
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
    FX_L1_SDK_API int FX_L1_Param_GetInt32(char *name, int *value);
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
     * @param[out] value Pointer to an float variable where the parameter
     *                   value will be stored if found. The pointer must not be NULL.
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
    FX_L1_SDK_API int FX_L1_Param_GetFloat(char *name, float *value);
    /**
     * @brief Clear data from a specified terminal
     *
     * This function clears or resets the data associated with a specific terminal type.
     * Depending on the terminal type The function is used to prepare a terminal for new
     * operations.
     *
     * @param terminal_type Type identifier of the terminal to clear. Valid values are defined
     *                      in the FXTerminalType enumeration.
     *
     * @return int Operation result:
     *         - 1: Terminal data cleared successfully
     *         - 0: Failed to clear terminal data
     *
     * @note The exact data cleared depends on the terminal type and implementation
     * @note Some terminals may preserve certain system or configuration data
     * @note Clearing may be asynchronous; data may not be immediately removed from display
     * @note The terminal may need to be reinitialized or reconfigured after clearing
     *
     * @see FX_L1_Terminal_SetData() Send data to a terminal
     * @see FX_L1_Terminal_GetData() Receive data from a terminal
     */
    FX_L1_SDK_API int FX_L1_Terminal_ClearData(FXTerminalType terminal_type);
    /**
     * @brief Retrieve data from a specified terminal
     *
     * This function retrieves data from a specified terminal and identifies the channel type
     * from which the data originated. The function copies up to 64 bytes of terminal data
     * into the provided buffer and returns the channel type information. This is typically
     * used to read user input, receive messages, or obtain status information from terminals.
     *
     * @param terminal_type Type identifier of the terminal from which to retrieve data.
     *                      Valid values are defined in the FXTerminalType enumeration.
     *
     * @param[out] chn_type Pointer to an FXChnType variable that will receive the channel type
     *                      associated with the retrieved data.
     *
     * @param[out] data Array of 64 bytes that will be populated with the retrieved terminal data.
     *                  The actual data format and content depend on the terminal and channel types.
     *
     * @return int Operation result:
     *         - Positive values: Number of bytes actually retrieved and stored in the data array
     *         - 0: No data available from the terminal
     *         - -1: Invalid terminal type
     *
     * @see FX_L1_Terminal_SendData() Send data to a terminal
     * @see FX_L1_Terminal_ClearData() Clear terminal data buffer
     */
    FX_L1_SDK_API int FX_L1_Terminal_GetData(FXTerminalType terminal_type, FXChnType *chn_type, unsigned char data[64]);
    /**
     * @brief Send data to a specified terminal through a specific channel
     *
     * This function sends data to a specified terminal via a designated channel type.
     * The data, up to 64 bytes in length, is transmitted to the terminal for processing,
     * display, or forwarding. The channel type determines how the terminal interprets
     * and handles the data.
     *
     * @param terminal_type Type identifier of the destination terminal. Valid values are
     *                      defined in the FXTerminalType enumeration.
     *
     * @param chn_type Channel type through which the data is being sent. This parameter
     *                 determines how the terminal processes the data.
     *
     * @param data Array containing the data to be sent to the terminal. The data format
     *             and interpretation depend on the terminal and channel types.
     *
     * @param data_len Number of bytes to send from the data array. This value must be
     *                 between 0 and 64 inclusive.
     *
     * @return unsigned char Operation result:
     *         - 1: Data successfully sent to terminal
     *         - 0: Failed to send data to terminal
     *
     * @see FX_L1_Terminal_GetData() Retrieve data from a terminal
     * @see FX_L1_Terminal_ClearData() Clear terminal data
     */
    FX_L1_SDK_API int FX_L1_Terminal_SetData(FXTerminalType terminal_type, FXChnType chn_type, unsigned char data[64], unsigned int data_len);
    /**
     * @brief Set brake lock state for specified axes of a control object
     *
     * This function controls the brake lock mechanism for one or more axes of a specified
     * control object. Brake locks are safety or holding devices that prevent unintended
     * movement when engaged. The function can independently control multiple axes using
     * a bitmask, allowing selective engagement or disengagement of brakes.
     *
     * @param obj_type Type identifier of the control object to transition to position mode.
     *                 This should be a position-controllable object, typically:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *
     * @param axis_mask Bitmask specifying which axes' brake locks to control. Each bit corresponds
     *                  to a specific axis of the control object:
     *                  - Bit 0: Axis 0 brake control
     *                  - Bit 1: Axis 1 brake control
     *                  - Bit 2: Axis 2 brake control
     *                  - ...
     *                  - Bit 7: Axis 7 brake control
     *                  The interpretation of each bit (engage/disengage) depends on the
     *                  brake control logic (active high/low) and is implementation-specific.
     *
     * @return int Operation result:
     *         - 1: Brake lock command accepted and executed for specified axes
     *         - 0: Failed to set brake lock state
     *
     * @see FX_L1_Config_SetBrakeUnlock() Release brakes with specific parameters
     */
    FX_L1_SDK_API int FX_L1_Config_SetBrakeLock(FXObjType obj_type, unsigned char axis_mask);
    /**
     * @brief Set brake unlock state for specified axes of a control object
     *
     * This function controls the brake unlock (release) mechanism for one or more axes
     * of a specified control object. When brakes are unlocked, the corresponding axes
     * are free to move under motor control. The function uses a bitmask to independently
     * control multiple axes, allowing selective release of brakes.
     *
     * @param obj_type Type identifier of the control object to transition to position mode.
     *                 This should be a position-controllable object, typically:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *
     * @param axis_mask Bitmask specifying which axes' brakes to unlock. Each bit corresponds
     *                  to a specific axis of the control object:
     *                  - Bit 0: Axis 0 brake unlock
     *                  - Bit 1: Axis 1 brake unlock
     *                  - Bit 2: Axis 2 brake unlock
     *                  - ...
     *                  - Bit 7: Axis 7 brake unlock
     *                  Setting a bit to 1 typically releases (unlocks) the brake on that axis,
     *                  while 0 leaves the brake state unchanged (implementation-specific).
     *
     * @return int Operation result:
     *         - 1: Brake unlock command accepted and executed for specified axes
     *         - 0: Failed to unlock brakes
     *
     * @see FX_L1_Config_SetBrakeLock() Engage (lock) brakes
     */
    FX_L1_SDK_API int FX_L1_Config_SetBrakeUnlock(FXObjType obj_type, unsigned char axis_mask);
    /**
     * @brief Reset encoder offset for specified axes of a control object
     *
     * This function resets the encoder offset values for one or more axes of a specified
     * control object. Encoder offsets are calibration values that align the encoder's
     * electrical zero position with the mechanical or absolute reference position of the axis.
     * Resetting these offsets reinitializes the position reference for the affected axes.
     *
     * @param obj_type Type identifier of the control object to transition to position mode.
     *                 This should be a position-controllable object, typically:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *                 - FX_OBJ_LIFT: Lift
     *
     * @param axis_mask Bitmask specifying which axes' encoder offsets to reset. Each bit corresponds
     *                  to a specific axis of the control object:
     *                  - Bit 0: Reset encoder offset for axis 0
     *                  - Bit 1: Reset encoder offset for axis 1
     *                  - Bit 2: Reset encoder offset for axis 2
     *                  - ...
     *                  - Bit 7: Reset encoder offset for axis 7
     *                  Setting a bit to 1 resets the encoder offset for that axis, 0 leaves it unchanged.
     *
     * @return int Operation result:
     *         - 1: Encoder offsets successfully reset for specified axes
     *         - 0: Failed to reset encoder offsets
     *
     * @note This operation affects the position reference
     * @note The axis should typically be stationary when resetting encoder offsets
     */
    FX_L1_SDK_API int FX_L1_Config_ResetEncOffset(FXObjType obj_type, unsigned char axis_mask);
    /**
     * @brief Clear encoder errors for specified axes of a control object
     *
     * This function clears encoder-related error conditions for one or more axes of a
     * specified control object. Encoder errors may include communication faults, signal
     * quality issues, count errors, or other encoder-specific fault conditions. Clearing
     * these errors resets the encoder error state and may enable normal operation to resume.
     *
     * @param obj_type Type identifier of the control object to transition to position mode.
     *                 This should be a position-controllable object, typically:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *
     * @param axis_mask Bitmask specifying which axes' encoder errors to clear. Each bit corresponds
     *                  to a specific axis of the control object:
     *                  - Bit 0: Clear encoder errors for axis 0
     *                  - Bit 1: Clear encoder errors for axis 1
     *                  - Bit 2: Clear encoder errors for axis 2
     *                  - ...
     *                  - Bit 7: Clear encoder errors for axis 7
     *                  Setting a bit to 1 attempts to clear errors for that axis, 0 leaves error state unchanged.
     *
     * @return int Operation result:
     *         - 1: Encoder errors successfully cleared for specified axes
     *         - 0: Failed to clear encoder errors
     *
     * @note Clearing errors does not fix the underlying cause; errors may reoccur
     * @note Some encoder errors may require power cycle or hardware reset to clear
     * @note The function may only clear soft errors; hard faults may persist
     */
    FX_L1_SDK_API int FX_L1_Config_ClearEncError(FXObjType obj_type, unsigned char axis_mask);
    /**
     * @brief Clear sensor offset for a specific axis of a control object
     *
     * This function clear a calibration offset value for a sensor associated with a particular
     * axis of a control object. Sensor offsets compensate for mechanical misalignment,
     * installation tolerances, or sensor bias to ensure accurate position, velocity, or
     * force/torque measurements. The offset is applied to raw sensor readings to obtain
     * corrected values.
     *
     * @param obj_type Type identifier of the control object to transition to position mode.
     *                 This should be a position-controllable object, typically:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_BODY: Body
     *
     * @param axis_id Identifier of the specific axis whose sensor offset is being set.
     *                The axis numbering is implementation-specific but typically:
     *                - 0: First axis or primary axis
     *                - 1: Second axis
     *                - ...: Subsequent axes
     *                - Maximum value depends on the control object's configuration
     *
     * @return int Operation result:
     *         - 1: Sensor offset successfully cleared
     *         - 0: Failed to clear sensor offset
     *
     * @note Setting an offset may affect control performance and should be done carefully
     */
    FX_L1_SDK_API int FX_L1_Config_ClearAxisSensorOffset(FXObjType obj_type, unsigned int axis_id);
    /**
     * @brief Clear sensor offset calibration for a control object
     *
     * This function clears or resets all sensor offset calibration values for a specified
     * control object. This action removes any previously applied offset corrections and
     * restores sensor readings to their raw, uncalibrated state. The function affects
     * all sensors associated with the control object, including position, force, torque,
     * and other sensor types.
     *
     * @param obj_type Type identifier of the control object whose sensor offsets are to be cleared.
     *                 This should be a device with one or more calibrated sensors, such as:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_BODY: Body
     *
     * @return int Operation result:
     *         - 1: All sensor offsets successfully cleared
     *         - 0: Failed to clear sensor offsets
     *
     * @note Setting an offset may affect control performance and should be done carefully
     */
    FX_L1_SDK_API int FX_L1_Config_ClearSensorOffset(FXObjType obj_type);
    /**
     * @brief Disable software limits for specified axes of a control object
     *
     * This function disables the software travel limits for one or more axes of a specified
     * control object. Software limits are safety features that prevent axis movement beyond
     * configured position boundaries. Disabling these limits allows movement throughout the
     * full mechanical range, which may be necessary for maintenance, calibration, or
     * special operational modes.
     *
     * @param obj_type Type identifier of the control object whose software limits are to be disabled.
     *                 This should be a device with configurable software limits, such as:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *                 - FX_OBJ_LIFT: Lift
     *
     * @param axis_mask Bitmask specifying which axes' software limits to enable or disable. Each bit corresponds
     *                  to a specific axis of the control object:
     *                  - Bit 0: Disable software limits for axis 0
     *                  - Bit 1: Disable software limits for axis 1
     *                  - Bit 2: Disable software limits for axis 2
     *                  - ...
     *                  - Bit 7: Disable software limits for axis 7
     *                  Setting a bit to 1 disables software limits for that axis, 0 enables limits .
     *
     * @return int Operation result:
     *         - 1: Software limits successfully disabled for specified axes
     *         - 0: Failed to disable software limits
     *
     * @warning Disabling software limits removes a safety feature; mechanical damage may occur
     * @note The limits are typically re-enabled when the control object transfer to idle state next time
     */
    FX_L1_SDK_API int FX_L1_Config_DisableSoftLimit(FXObjType obj_type, unsigned char axis_mask);
    /**
     * @brief Configure trajectory data for a control object
     *
     * This function configures trajectory data for a specified control object. A trajectory
     * consists of a sequence of via points that define a desired path for the object to follow.
     * The function sets up the trajectory with the specified number of points and associated
     * data, which may include position information.
     *
     * @param obj_type Type identifier of the control object for which to configure trajectory.
     *                 This should be a motion-capable device that supports trajectory following:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_BODY: Body
     *                 - FX_OBJ_LIFT: Lift
     *
     * @param point_num Number of points in the trajectory. This specifies how many via points
     *                  or trajectory segments are defined in the point_data array. The maximum
     *                  allowed number is implementation-specific.
     *
     * @param point_data Pointer to an array containing trajectory point data.
     *                   - Left arm: 7 joint positions format as a point
     *                   - Right arm: 7 joint positions format as a point
     *                   - Body: 6 joint positions format as a point
     *                   - Lift: 2 joint positions format as a point
     *
     * @return int Operation result:
     *         - 1: Trajectory data successfully configured
     *         - 0: Failed to configure trajectory
     *
     * @see FX_L1_Runtime_RunTraj() Start trajectory execution
     * @see FX_L1_Runtime_StopTraj() Stop trajectory execution
     */
    FX_L1_SDK_API int FX_L1_Config_SetTraj(FXObjType obj_type, unsigned int point_num, double *point_data);
    /**
     * @brief Execute emergency stop on specified objects
     *
     * This function triggers an emergency stop procedure for a set of control objects specified
     * by a bitmask. Emergency stop immediately halts all motion and potentially disables power
     * to the affected objects to ensure safety. The function returns a bitmask indicating
     * which objects successfully executed the emergency stop.
     *
     * @param obj_mask Bitmask specifying which objects to emergency stop. Each bit corresponds to
     *                 a control object or object group:
     *                 - Bit 0: Left arm
     *                 - Bit 1: Right arm
     *                 - Bit 2: Head
     *                 - Bit 3: Body
     *                 - Bit 4: Lift
     *                 Set a bit to 1 to trigger emergency stop on that object, 0 to leave it unaffected.
     *
     * @return unsigned int Bitmask indicating emergency stop execution results:
     *         - Each bit set to 1 indicates the corresponding object successfully executed emergency stop
     *         - Each bit set to 0 indicates the corresponding object failed to execute emergency stop
     *         - The returned mask is a subset of the input obj_mask
     *
     * @note Emergency stop is a safety-critical function that overrides normal operation
     * @note After emergency stop, objects typically require explicit reset/acknowledgment before resuming
     */
    FX_L1_SDK_API unsigned int FX_L1_Runtime_EmergencyStop(unsigned int obj_mask);
    /**
     * @brief Set joint position command for a control object
     *
     * This function sets the target position command for the joints of a specified control object.
     * The command specifies desired joint angles or positions for up to 7 joints, which the
     * control system will attempt to achieve according to the current control mode and motion profile.
     * The function is typically used in position control or trajectory following modes.
     *
     * @param obj_type Type identifier of the control object to receive the position command.
     *                 This should be a multi-joint device, such as:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *                 - FX_OBJ_LIFT: Lift
     *
     * @param pos_cmd Array of 7 position command values for the joints. Each element corresponds
     *                to a specific joint and contains the desired position in appropriate units:
     *                - pos_cmd[0]: Command for joint 0 (typically base or first joint)
     *                - pos_cmd[1]: Command for joint 1
     *                - pos_cmd[2]: Command for joint 2
     *                - pos_cmd[3]: Command for joint 3
     *                - pos_cmd[4]: Command for joint 4
     *                - pos_cmd[5]: Command for joint 5
     *                - pos_cmd[6]: Command for joint 6 (if applicable)
     *                Units are typically radians or degrees, depending on system configuration.
     *                For objects with fewer than 7 joints, unused array elements may be ignored.
     *
     * @return int Operation result:
     *         - 1: Position command accepted and will be executed
     *         - 0: Failed to set position command
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetJointPosCmd(FXObjType obj_type, double pos_cmd[7]);
    /**
     * @brief Set force control parameters for a control object
     *
     * This function sets the force control parameters for a specified control object operating
     * in force control or impedance-force control mode. The parameters define the desired
     * force behavior in Cartesian directions, which the control system will attempt to achieve
     * or regulate. The parameters typically include target forces, control gains, or other
     * force-related settings.
     *
     * @param obj_type Type identifier of the control object to receive force control parameters.
     *                 This should be a device capable of force control, such as:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param force_ctrl Array of force control parameters for Cartesian directions.
     *                   The array size is FX_FORCE_DEF_NUM (typically 3 for X, Y, Z directions).
     *                   Each element contains control parameters for a specific direction:
     *                   - force_ctrl[0]: Direction of force in X direction under the arm base coordinate
     *                   - force_ctrl[1]: Direction of force in Y direction under the arm base coordinate
     *                   - force_ctrl[2]: Direction of force in Z direction under the arm base coordinate
     *                   - force_ctrl[3]: Magnitude of force (N)
     *                   - force_ctrl[4]: Maximum move distance along the force (mm)
     *
     * @return int Operation result:
     *         - 1: Force control parameters successfully set
     *         - 0: Failed to set force control parameters
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetForceCtrl(FXObjType obj_type, double force_ctrl[FX_FORCE_DEF_NUM]);
    /**
     * @brief Set torque control parameters for a control object
     *
     * This function sets the torque control parameters for a specified control object operating
     * in torque control or impedance-torque control mode. The parameters define the desired
     * torque behavior for rotational degrees of freedom, which the control system will attempt
     * to achieve or regulate. The parameters typically include target torques, control gains,
     * or other torque-related settings.
     *
     * @param obj_type Type identifier of the control object to receive torque control parameters.
     *                 This should be a device capable of torque control, such as:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param torque_ctrl Array of torque control parameters for rotational directions.
     *                    The array size is FX_TORQUE_DEF_NUM (typically 3 for Rx, Ry, Rz rotations).
     *                    Each element contains control parameters for a specific rotational axis:
     *                    - torque_ctrl[0]: Direction of torque in X direction under the arm base coordinate
     *                    - torque_ctrl[1]: Direction of torque in Y direction under the arm base coordinate
     *                    - torque_ctrl[2]: Direction of torque in Z direction under the arm base coordinate
     *                    - torque_ctrl[3]: Magnitude of torque (Nm)
     *                    - torque_ctrl[4]: Maximum rotate angles along the torque (rad)
     *
     * @return int Operation result:
     *         - 1: Torque control parameters successfully set
     *         - 0: Failed to set torque control parameters
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetTorqueCtrl(FXObjType obj_type, double torque_ctrl[FX_TORQUE_DEF_NUM]);
    /**
     * @brief Set velocity ratio for a control object
     *
     * This function sets the velocity ratio (also called override or speed scaling factor)
     * for a specified control object. The velocity ratio scales the commanded velocities
     * and accelerations by the specified factor, allowing adjustment of motion speed
     * without modifying trajectory or position commands. A ratio of 100.0 represents
     * 100% (nominal) speed, while 50 represents 50% speed, etc.
     *
     * @param obj_type Type identifier of the control object whose velocity ratio is to be set.
     *                 This should be a motion-capable device that supports velocity scaling:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *                 - FX_OBJ_LIFT: Lift
     *
     * @param vel_ratio Velocity scaling factor to apply. Valid range is typically 1.0 to 100.0
     *
     * @return int Operation result:
     *         - 1: Velocity ratio successfully set
     *         - 0: Failed to set velocity ratio
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetVelRatio(FXObjType obj_type, double vel_ratio);
    /**
     * @brief Set acceleration ratio for a control object
     *
     * This function sets the acceleration ratio (acceleration scaling factor) for a specified
     * control object. The acceleration ratio scales the commanded accelerations and decelerations
     * by the specified factor, allowing adjustment of motion dynamics without modifying
     * trajectory or velocity commands. This affects how quickly the object reaches commanded
     * speeds and how smoothly it transitions between motion segments.
     *
     * @param obj_type Type identifier of the control object whose acceleration ratio is to be set.
     *                 This should be a motion-capable device that supports acceleration scaling:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *                 - FX_OBJ_LIFT: Lift
     *
     * @param acc_ratio Acceleration scaling factor to apply. Valid range is typically 1.0 to 100.0
     *
     * @return int Operation result:
     *         - 1: Acceleration ratio successfully set
     *         - 0: Failed to set acceleration ratio
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetAccRatio(FXObjType obj_type, double acc_ratio);
    /**
     * @brief Set comprehensive speed ratios for a control object
     *
     * This function simultaneously sets both velocity and acceleration ratios (scaling factors)
     * for a specified control object. It provides a convenient way to adjust motion dynamics
     * by scaling both speed and acceleration parameters together, ensuring consistent motion
     * behavior. The ratios affect all subsequent motion commands until modified.
     *
     * @param obj_type Type identifier of the control object whose speed ratios are to be set.
     *                 This should be a motion-capable device that supports dynamic scaling:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *                 - FX_OBJ_HEAD: Head
     *                 - FX_OBJ_BODY: Body
     *                 - FX_OBJ_LIFT: Lift
     *
     * @param vel_ratio Velocity scaling factor to apply. Valid range is typically 1.0 to 100.0
     *
     * @param acc_ratio Acceleration scaling factor to apply. Valid range is typically 1.0 to 100.0
     *
     * @return int Operation result:
     *         - 1: Both velocity and acceleration ratios successfully set
     *         - 0: Failed to set one or both ratios
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetSpeedRatio(FXObjType obj_type, double vel_ratio, double acc_ratio);
    /**
     * @brief Set joint stiffness coefficients for a control object
     *
     * This function sets the joint stiffness (spring constant) coefficients for a specified
     * control object. Stiffness determines the resistance to position deviations in joint space
     * and is a key parameter in impedance control, compliance control, or force control modes.
     * The coefficients define the relationship between position error and applied torque for
     * each joint, affecting the object's dynamic behavior and interaction characteristics.
     *
     * @param obj_type Type identifier of the control object whose joint stiffness is to be set.
     *                 This should be a device operating in compliant or impedance control mode:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param k Array of 7 stiffness coefficients for the joints. Each element corresponds
     *          to a specific joint and contains the stiffness value in appropriate units:
     *          - k[0]: Stiffness for joint 0 (typically Nm/rad or Nm/degree)
     *          - k[1]: Stiffness for joint 1
     *          - k[2]: Stiffness for joint 2
     *          - k[3]: Stiffness for joint 3
     *          - k[4]: Stiffness for joint 4
     *          - k[5]: Stiffness for joint 5
     *          - k[6]: Stiffness for joint 6 (if applicable)
     *          A value of 0 indicates free motion (no stiffness) for that joint.
     *          For objects with fewer than 7 joints, unused array elements may be ignored.
     *
     * @return int Operation result:
     *         - 1: Joint stiffness coefficients successfully set
     *         - 0: Failed to set joint stiffness coefficients
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetJointK(FXObjType obj_type, double k[7]);
    /**
     * @brief Set joint damping coefficients for a control object
     *
     * This function sets the joint damping coefficients for a specified control object.
     * Damping determines the resistance to velocity in joint space and is a critical parameter
     * for system stability in impedance control, compliance control, or dynamic motion modes.
     * The coefficients define the relationship between joint velocity and damping torque,
     * affecting vibration suppression, energy dissipation, and overall dynamic response.
     *
     * @param obj_type Type identifier of the control object whose joint damping is to be set.
     *                 This should be a device operating in dynamic or compliant control modes:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param d Array of 7 damping coefficients for the joints. Each element corresponds
     *          to a specific joint and contains the damping value in appropriate units:
     *          - d[0]: Damping for joint 0 (typically Nm·s/rad or Nm·s/degree)
     *          - d[1]: Damping for joint 1
     *          - d[2]: Damping for joint 2
     *          - d[3]: Damping for joint 3
     *          - d[4]: Damping for joint 4
     *          - d[5]: Damping for joint 5
     *          - d[6]: Damping for joint 6 (if applicable)
     *          A value of 0 indicates no damping (potentially underdamped/oscillatory) for that joint.
     *          For objects with fewer than 7 joints, unused array elements may be ignored.
     *
     * @return int Operation result:
     *         - 1: Joint damping coefficients successfully set
     *         - 0: Failed to set joint damping coefficients
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetJointD(FXObjType obj_type, double d[7]);
    /**
     * @brief Set joint stiffness and damping coefficients for a control object
     *
     * This function simultaneously sets both joint stiffness and damping coefficients
     * for a specified control object. It provides a convenient way to configure the
     * complete joint-space impedance model by defining both the spring (stiffness) and
     * damper (damping) characteristics. The combined parameters determine the dynamic
     * response to position errors and velocities in impedance or compliance control modes.
     *
     * @param obj_type Type identifier of the control object whose joint impedance parameters
     *                 are to be set. This should be a device operating in impedance control:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param k Array of 7 stiffness coefficients for the joints. Each element corresponds
     *          to a specific joint and contains the stiffness value in appropriate units
     *          (typically Nm/rad or Nm/degree):
     *          - k[0]: Stiffness for joint 0
     *          - k[1]: Stiffness for joint 1
     *          - k[2]: Stiffness for joint 2
     *          - k[3]: Stiffness for joint 3
     *          - k[4]: Stiffness for joint 4
     *          - k[5]: Stiffness for joint 5
     *          - k[6]: Stiffness for joint 6 (if applicable)
     *
     * @param d Array of 7 damping coefficients for the joints. Each element corresponds
     *          to a specific joint and contains the damping value in appropriate units
     *          (typically Nm·s/rad or Nm·s/degree):
     *          - d[0]: Damping for joint 0
     *          - d[1]: Damping for joint 1
     *          - d[2]: Damping for joint 2
     *          - d[3]: Damping for joint 3
     *          - d[4]: Damping for joint 4
     *          - d[5]: Damping for joint 5
     *          - d[6]: Damping for joint 6 (if applicable)
     *
     * @return int Operation result:
     *         - 1: Joint stiffness and damping coefficients successfully set
     *         - 0: Failed to set joint impedance coefficients
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetJointKD(FXObjType obj_type, double k[7], double d[7]);
    /**
     * @brief Set Cartesian stiffness coefficients for a control object
     *
     * This function sets the Cartesian stiffness coefficients for a specified control object.
     * Stiffness determines the resistance to position/orientation deviations in Cartesian
     * (task) space and is a key parameter in Cartesian impedance control or compliant motion.
     * The coefficients define the relationship between Cartesian pose error and applied
     * force/torque at the end-effector, affecting the object's interaction behavior with
     * the environment or external forces.
     *
     * @param obj_type Type identifier of the control object whose Cartesian stiffness is to be set.
     *                 This should be a device capable of Cartesian-space compliance control:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param k Array of 7 stiffness coefficients in appropriate units (typically N/m for linear,
     *          Nm/rad for rotational). Each element corresponds to a Cartesian degree of freedom:
     *          - k[0]: Stiffness in X direction
     *          - k[1]: Stiffness in Y direction
     *          - k[2]: Stiffness in Z direction
     *          - k[3]: Rotational stiffness about X
     *          - k[4]: Rotational stiffness about Y
     *          - k[5]: Rotational stiffness about Z
     *          - k[6]: Stiffness about zero space
     *          A value of 0 indicates free motion in that direction.

     *
     * @return int Operation result:
     *         - 1: Cartesian stiffness coefficients successfully set
     *         - 0: Failed to set Cartesian stiffness coefficients
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetCartK(FXObjType obj_type, double k[7]);
    /**
     * @brief Set Cartesian damping coefficients for a control object
     *
     * This function sets the Cartesian damping coefficients for a specified control object.
     * Damping determines the resistance to linear and angular velocities in Cartesian space
     * and is essential for stability in Cartesian impedance control or dynamic motion.
     * The coefficients define the relationship between Cartesian velocity and damping
     * force/torque at the end-effector, affecting vibration suppression, energy dissipation,
     * and overall dynamic response in task space.
     *
     * @param obj_type Type identifier of the control object whose Cartesian damping is to be set.
     *                 This should be a device operating in Cartesian-space dynamic control modes:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param d Array of 7 damping coefficients for Cartesian degrees of freedom. The first
     *          6 elements typically correspond to linear and rotational damping:
     *          - d[0]: Linear damping in X direction (typically N·s/m)
     *          - d[1]: Linear damping in Y direction
     *          - d[2]: Linear damping in Z direction
     *          - d[3]: Rotational damping about X axis (typically Nm·s/rad)
     *          - d[4]: Rotational damping about Y axis
     *          - d[5]: Rotational damping about Z axis
     *          - d[6]: Damping about zero space
     *          A value of 0 indicates no damping (potentially underdamped) in that direction.
     *
     * @return int Operation result:
     *         - 1: Cartesian damping coefficients successfully set
     *         - 0: Failed to set Cartesian damping coefficients
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetCartD(FXObjType obj_type, double d[7]);
    /**
     * @brief Set Cartesian stiffness and damping coefficients for a control object
     *
     * This function simultaneously sets both Cartesian stiffness and damping coefficients
     * for a specified control object. It provides a convenient way to configure the complete
     * Cartesian-space impedance model by defining both the spring (stiffness) and damper
     * (damping) characteristics in task space. The combined parameters determine the dynamic
     * response to pose errors and velocities at the end-effector in Cartesian impedance control.
     *
     * @param obj_type Type identifier of the control object whose Cartesian impedance parameters
     *                 are to be set. This should be a device operating in Cartesian impedance control:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
     *
     * @param k Array of 7 stiffness coefficients for Cartesian degrees of freedom. The first
     *          6 elements typically correspond to linear and rotational stiffness:
     *          - k[0]: Linear stiffness in X direction (typically N/m)
     *          - k[1]: Linear stiffness in Y direction
     *          - k[2]: Linear stiffness in Z direction
     *          - k[3]: Rotational stiffness about X axis (typically Nm/rad)
     *          - k[4]: Rotational stiffness about Y axis
     *          - k[5]: Rotational stiffness about Z axis
     *          - k[6]: Stiffness about zero space
     *
     * @param d Array of 7 damping coefficients for Cartesian degrees of freedom. The first
     *          6 elements typically correspond to linear and rotational damping:
     *          - d[0]: Linear damping in X direction (typically N·s/m)
     *          - d[1]: Linear damping in Y direction
     *          - d[2]: Linear damping in Z direction
     *          - d[3]: Rotational damping about X axis (typically Nm·s/rad)
     *          - d[4]: Rotational damping about Y axis
     *          - d[5]: Rotational damping about Z axis
     *          - d[6]: Damping about zero space
     *
     * @return int Operation result:
     *         - 1: Cartesian stiffness and damping coefficients successfully set
     *         - 0: Failed to set Cartesian impedance coefficients
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetCartKD(FXObjType obj_type, double k[7], double d[7]);
    /**
     * @brief Sets the tool offset in the tool frame for left/right arm.
     *
     * This function configures the translational and rotational offset
     * (transformation) from the end-effector flange to the tool tip or
     * working point. The tool offset defines where the effective tool
     * frame is located relative to the robot's flange, affecting all
     * subsequent Cartesian commands and measurements.
     *
     * @param obj_type Type identifier of the control object whose Cartesian impedance parameters
     *                 are to be set. This should be a device operating in Cartesian impedance control:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
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
    FX_L1_SDK_API int FX_L1_Runtime_SetToolK(FXObjType obj_type, double k[6]);
    /**
     * @brief Sets the tool dynamics parameters for Arm0.
     *
     * This function configures the dynamic properties of the tool attached
     * to Arm0's end-effector. These parameters are used for accurate
     * dynamics compensation, gravity compensation, and proper control
     * performance. The tool dynamics are essential for applications
     * requiring precise force control or when using tools with significant mass.
     *
     * @param obj_type Type identifier of the control object whose Cartesian impedance parameters
     *                 are to be set. This should be a device operating in Cartesian impedance control:
     *                 - FX_OBJ_ARM0: Left arm
     *                 - FX_OBJ_ARM1: Right arm
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
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetToolD(FXObjType obj_type, double d[10]);
    /**
     * @brief Set tool stiffness and damping coefficients for a control object
     *
     * This function sets the stiffness and damping coefficients associated with a tool
     * or end-effector attached to a specified control object. Tool impedance parameters
     * define the dynamic behavior at the tool center point (TCP) or tool interface,
     * which may differ from the robot's base Cartesian impedance. This allows independent
     * control of tool compliance for tasks like assembly, polishing, or contact operations.
     *
     * @param obj_type Type identifier of the control object whose tool impedance parameters
     *                 are to be set. This should be a device with a defined tool or end-effector:
     *                 - Robotic manipulator with tool interface
     *                 - Force-controlled end-effector
     *                 - Tool-changing system
     *                 - Other equipment with tool-specific compliance
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
     * @return int Operation result:
     *         - 1: Tool stiffness and damping coefficients successfully set
     *         - 0: Failed to set tool impedance coefficients
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetToolKD(FXObjType obj_type, double k[6], double d[10]);
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
    FX_L1_SDK_API int FX_L1_Runtime_SetBodyPDP(double p[6]);
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
    FX_L1_SDK_API int FX_L1_Runtime_SetBodyPDD(double d[6]);
    /**
     * @brief Set body proportional and derivative control gains
     *
     * This function sets the proportional (P) and derivative (D) control gains for the
     * body or base motion control of the system. These gains determine the response
     * characteristics of the body-level controller, affecting stability, responsiveness,
     * and damping of the overall system motion. The function is typically used for
     * systems with mobile bases, floating bases, or body-level motion control.
     *
     * @param[in] p Array of 6 joint stiffness values (double) in appropriate
     *             units (typically Nm/Deg).
     *
     * @param[in] d Array of 6 joint damping values (double) in appropriate
     *             units (typically Nm·s/Deg).
     *
     * @return int Operation result:
     *         - 1: Body P and D gains successfully set
     *         - 0: Failed to set body control gains
     */
    FX_L1_SDK_API int FX_L1_Runtime_SetBodyPD(double p[6], double d[6]);
    /**
     * @brief Execute trajectories on multiple control objects
     *
     * This function initiates trajectory execution on a set of control objects specified
     * by a bitmask. The objects must have pre-configured trajectory data (via configuration
     * functions) and be in appropriate states for trajectory execution. The function
     * starts synchronized or coordinated motion according to the defined trajectories.
     *
     * @param obj_mask Bitmask specifying which objects should execute their trajectories.
     *                 Each bit corresponds to a control object or object group:
     *                 - Bit 0: Left arm
     *                 - Bit 1: Right arm
     *                 - Bit 3: Body
     *                 - Bit 4: Lift
     *
     * @return unsigned int Bitmask indicating trajectory execution results:
     *         - Each bit set to 1 indicates the corresponding object successfully started trajectory execution
     *         - Each bit set to 0 indicates the corresponding object failed to start trajectory execution
     *         - The returned mask is a subset of the input obj_mask
     *
     * @note Trajectory data must be properly configured before calling this function
     * @note Objects typically need to be in position modes for trajectory execution
     */
    FX_L1_SDK_API unsigned int FX_L1_Runtime_RunTraj(unsigned int obj_mask);
    /**
     * @brief Stop trajectory execution on multiple control objects
     *
     * This function stops trajectory execution on a set of control objects specified
     * by a bitmask. The function initiates a controlled stop of ongoing trajectory
     * following, typically using configured deceleration profiles to bring the objects
     * to a halt. The stop may be immediate (emergency stop) or controlled (decelerated stop)
     * depending on system configuration and object state.
     *
     * @param obj_mask Bitmask specifying which objects should stop trajectory execution.
     *                 Each bit corresponds to a control object or object group:
     *                 - Bit 0: Left arm
     *                 - Bit 1: Right arm
     *                 - Bit 3: Body
     *                 - Bit 4: Lift
     *
     * @return unsigned int Bitmask indicating trajectory stop results:
     *         - Each bit set to 1 indicates the corresponding object successfully stopped trajectory execution
     *         - Each bit set to 0 indicates the corresponding object failed to stop or was not executing a trajectory
     *         - The returned mask is a subset of the input obj_mask
     */
    FX_L1_SDK_API unsigned int FX_L1_Runtime_StopTraj(unsigned int obj_mask);

    /*=============================================================================
     * RT SG 数据接口
     *============================================================================*/
    FX_L1_SDK_API int RobotCache_Init();
    FX_L1_SDK_API const ROBOT_RT *RobotCache_GetRT();
    FX_L1_SDK_API const ROBOT_SG *RobotCache_GetSG();
    FX_L1_SDK_API void RobotCache_Cleanup();

    /*=============================================================================
     * 运动学与规划接口
     *============================================================================*/

    /* 不透明句柄 */
    /**
     * @brief Opaque handle for the kinematics context.
     *
     * The handle owns the single-arm kinematics, MAX body kinematics,
     * and motion planning related objects.
     */
    typedef struct FX_MotionContext *FX_MotionHandle;

    /* 生命周期管理 */
    /**
     * @brief Create a kinematics context.
     *
     * @return A valid handle on success, or `nullptr` on failure.
     */
    FX_L1_SDK_API FX_MotionHandle FX_L1_Kinematics_Create(void);
    /**
     * @brief Destroy a kinematics context.
     *
     * @param[in] handle Context handle created by `FX_L0_Kinematics_create`.
     */
    FX_L1_SDK_API void FX_L1_Kinematics_Destroy(FX_MotionHandle handle);
    /**
     * @brief Enable or disable kinematics logging.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] on Log switch, `1` to enable and `0` to disable.
     */
    FX_L1_SDK_API void FX_L1_Kinematics_LogSwitch(FX_MotionHandle handle, int on);

    /* 初始化 */
    /**
     * @brief Initialize the environment for a single arm.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] env_path Path to the robot environment configuration.
     * @param[in] robot_serial Arm index, `0` for left arm and `1` for right arm.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_InitSingleArm(FX_MotionHandle handle, int RobotSerial, int *type, double DH[8][4], double PNVA[8][4], double BOUND[4][3],
                                                     double GRV[3], double MASS[7], double MCP[7][3], double I[7][6]);
    /**
     * @brief Initialize the environment for dual-arm planning.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] env_path Path to the robot environment configuration.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_InitDualArm(FX_MotionHandle handle, int type[2], double DH[2][8][4], double PNVA[2][8][4], double BOUND[2][4][3]);

    /* 单臂运动学 */
    /**
     * @brief Solve forward kinematics for one arm.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index, `0` for left arm and `1` for right arm.
     * @param[in] joints Input joint values, length 7.
     * @param[out] pose_matrix Output TCP pose matrix, length 16 in 4x4 row-major order.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_ForwardKinematics(FX_MotionHandle handle, int robot_serial,
                                                         double joints[7], double pose_matrix[16]);
    /**
     * @brief Solve the Jacobian matrix for one arm.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index, `0` for left arm and `1` for right arm.
     * @param[in] joints Input joint values, length 7.
     * @param[out] jacobian Output Jacobian, length 42 in 6x7 row-major order.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_Jacobian(FX_MotionHandle handle, int robot_serial,
                                                double joints[7], double jacobian[42]);
    /**
     * @brief Solve inverse kinematics for one arm.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index, `0` for left arm and `1` for right arm.
     * @param[in,out] params IK input/output parameters including target pose,
     *                      reference joints, and solution buffer.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_InverseKinematics(FX_MotionHandle handle, int robot_serial,
                                                         FX_InvKineSolverParams *params);

    /* 身体运动学 */
    /**
     * @brief Set MAX body kinematics parameters.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] std_body Standard body position parameters, length 3.
     * @param[in] k_body Body stiffness parameters, length 3.
     * @param[in] std_left_len Standard length of the left arm.
     * @param[in] k_left Stiffness coefficient of the left arm.
     * @param[in] std_right_len Standard length of the right arm.
     * @param[in] k_right Stiffness coefficient of the right arm.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_SetBodyCondition(FX_MotionHandle handle,
                                                        double std_body[3], double k_body[3],
                                                        double std_left_len, double k_left,
                                                        double std_right_len, double k_right);
    /**
     * @brief Solve forward kinematics for the MAX body.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] jv Input body joint values, length 3.
     * @param[out] left_shoulder_matrix Output left shoulder matrix, length 16 in
     *                                  4x4 row-major order.
     * @param[out] right_shoulder_matrix Output right shoulder matrix, length 16 in
     *                                   4x4 row-major order.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_BodyForward(FX_MotionHandle handle, double jv[3],
                                                   double left_shoulder_matrix[16], double right_shoulder_matrix[16]);
    /**
     * @brief Compute body joint values from dual-arm TCP positions.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] left_tcp Left TCP position, length 3.
     * @param[in] right_tcp Right TCP position, length 3.
     * @param[out] out_body_joints Output body joints, length 3.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_CalcBodyPosition(FX_MotionHandle handle,
                                                        double left_tcp[3], double right_tcp[3],
                                                        double out_body_joints[3]);
    /**
     * @brief Compute body joint values with a reference body pose.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] ref_body_joints Reference body joints, length 3.
     * @param[in] left_tcp Left TCP position, length 3.
     * @param[in] right_tcp Right TCP position, length 3.
     * @param[out] out_body_joints Output body joints, length 3.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_CalcBodyPositionWithRef(FX_MotionHandle handle,
                                                               double ref_body_joints[3],
                                                               double left_tcp[3], double right_tcp[3],
                                                               double out_body_joints[3]);

    /* 运动规划（单臂） */
    /**
     * @brief Plan a joint-space MoveJ path.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index. The parameter is currently reserved by this API.
     * @param[in] start_joints Start joint values, length 7.
     * @param[in] end_joints Target joint values, length 7.
     * @param[in] vel_ratio Velocity ratio.
     * @param[in] acc_ratio Acceleration ratio.
     * @param[in,out] point_set_handle Path point set handle created by
     *                                 `FX_L0_CPointSet_Create`.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_PlanJointMove(FX_MotionHandle handle, int robot_serial,
                                                     double start_joints[7], double end_joints[7],
                                                     double vel_ratio, double acc_ratio,
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
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
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
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_PlanLinearKeepJoints(FX_MotionHandle handle, int robot_serial,
                                                            double start_joints[7], double end_joints[7],
                                                            double vel, double acc, int freq,
                                                            double *point_set_handle, int *point_num);
    /* 多端直线运动规划（单臂） */
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
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
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
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
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
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_PlanLinearMove_MultiPoints_GetPath(FX_MotionHandle handle,
                                                                          double *point_set_handle, int *point_num);

    /* 双臂同步规划（固定身体） */
    /**
     * @brief Plan a synchronized dual-arm linear path with fixed body state.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] params Dual-arm fixed-body planning parameters.
     * @param[in,out] left_point_set Left arm path point set handle created by
     *                               `FX_L0_CPointSet_Create`.
     * @param[in,out] right_point_set Right arm path point set handle created by
     *                                `FX_L0_CPointSet_Create`.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    FX_L1_SDK_API int FX_L1_Kinematics_PlanDualArmFixedBody(FX_MotionHandle handle,
                                                            DualArmFixedBodyParams *params,
                                                            double *left_point_set, double *right_point_set, int *point_num);

    /* 辅助工具 */
    /**
     * @brief Convert an `XYZABC` pose to a 4x4 transform matrix.
     *
     * @param[in] xyzabc Input pose, length 6, angles in degrees.
     * @param[out] matrix Output matrix, length 16 in 4x4 row-major order.
     */
    FX_L1_SDK_API void FX_L1_XYZABC2Matrix(double xyzabc[6], double matrix[16]);
    /**
     * @brief Convert a 4x4 transform matrix to an `XYZABC` pose.
     *
     * @param[in] matrix Input matrix, length 16 in 4x4 row-major order.
     * @param[out] xyzabc Output pose, length 6, angles in degrees.
     */
    FX_L1_SDK_API void FX_L1_Matrix2XYZABC(double matrix[16], double xyzabc[6]);

    /* PointSet 管理 */
    /**
     * @brief Create a point set object.
     *
     * @return A valid point set handle on success, or `nullptr` on failure.
     */
    FX_L1_SDK_API void *FX_L1_CPointSet_Create();
    /**
     * @brief Destroy a point set object.
     *
     * @param[in] pset Point set handle.
     */
    FX_L1_SDK_API void FX_L1_CPointSet_Destroy(void *pset);
    /**
     * @brief Initialize the point set type.
     *
     * @param[in] pset Point set handle.
     * @param[in] ptype Point set type value corresponding to `PoinType`.
     * @return `FX_TRUE` on success, otherwise `FX_FALSE`.
     */
    FX_L1_SDK_API int FX_L1_CPointSet_OnInit(void *pset, int ptype);
    /**
     * @brief Get the number of points stored in the point set.
     *
     * @param[in] pset Point set handle.
     * @return Number of points in the point set.
     */
    FX_L1_SDK_API int FX_L1_CPointSet_OnGetPointNum(void *pset);
    /**
     * @brief Get a point by index.
     *
     * @param[in] pset Point set handle.
     * @param[in] pos Point index.
     * @return Pointer to point data on success, or `nullptr` on failure.
     */
    FX_L1_SDK_API double *FX_L1_CPointSet_OnGetPoint(void *pset, int pos);
    /**
     * @brief Append one point into the point set.
     *
     * @param[in] pset Point set handle.
     * @param[in] point_value Input point data.
     * @return `FX_TRUE` on success, otherwise `FX_FALSE`.
     */
    FX_L1_SDK_API int FX_L1_CPointSet_OnSetPoint(void *pset, double point_value[]);
    /**
     * @brief Transfer CPointset to double type.
     *
     * @param[in] pset Point set handle.
     * @param[out] point_value Output point data.
     * @return `FX_TRUE` on success, otherwise `FX_FALSE`.
     */
    FX_L1_SDK_API int FX_L1_CPointSet_OnAppendPoint(void *pset, double *point_value);

    /*=============================================================================
     * 文件传输服务
     *============================================================================*/
    FX_L1_SDK_API int FX_L1_SendFile(char *local_file_path, char *remote_file_path);
    FX_L1_SDK_API int FX_L1_RecvFile(char *local_file_path, char *remote_file_path);

#ifdef __cplusplus
}
#endif

#endif
