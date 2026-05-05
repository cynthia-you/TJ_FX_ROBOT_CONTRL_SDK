#ifndef FX_L0KINEMATICS_H
#define FX_L0KINEMATICS_H

#include "Arm_Kinematics.h"
#include "MAX_Body_Kinematics.h"
#include "Motion_Planner.h"
#include "FXKineCommon.h"
#include "AxisPln.h"
#include <cstring>
#include <cstdlib>
#include <stdint.h>

#if defined(_WIN32) || defined(_WIN64)
#define KINEMATICS_SDK_API __declspec(dllexport)
#elif defined(__linux__)
#define KINEMATICS_SDK_API
#endif

#ifdef __cplusplus
extern "C"
{
#endif

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
     * @return A valid handle on success, or `nullptr` on failure.
     */
    KINEMATICS_SDK_API FX_MotionHandle FX_L0_Kinematics_create(void);

    /**
     * @brief Destroy a kinematics context.
     *
     * @param[in] handle Context handle created by `FX_L0_Kinematics_create`.
     */
    KINEMATICS_SDK_API void FX_L0_Kinematics_destroy(FX_MotionHandle handle);

    /**
     * @brief Enable or disable kinematics logging.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] on Log switch, `1` to enable and `0` to disable.
     */
    KINEMATICS_SDK_API void FX_L0_Kinematics_log_switch(FX_MotionHandle handle, int on);

    /**
    * @brief Initialize the kinematics and dynamics environment for a single robot arm.
    * @param[in]  handle        Kinematics context handle.
    * @param[in]  RobotSerial  Robot identifier (e.g., 0 for left/A arm, 1 for right/B arm).
    * @param[in]  type         Robot type identifier (input). CCS structure = 1017 and SRS structrue = 1007
    * @param[in]  DH           Denavit–Hartenberg parameters array (8x4).
    * ```
                        Each row typically represents [alpha, a, d, theta].
    ```
    * @param[in]  PNVA         Kinematic auxiliary parameters (8x4), used for extended modeling.
    * ```
                        Each row typically represents [positive joint limit,negtive joint limit,velocity limit,accelaration limit].
    ```
    * @param[in]  BOUND        Constraint boundaries (4x3).
    * ```
                        Each row may represent [a,b,c] which for y=a*x^2+b*x+c.
    ```
    * @param[in]  GRV          Gravity vector (3 elements), e.g., [gx, gy, gz].
    * @param[in]  MASS         Link masses (7 elements), corresponding to each joint/link.
    * @param[in]  MCP          Center of mass positions for each link (7x3).
    * @param[in]  I            Inertia parameters for each link (7x6),
    * ```
                        typically in the form [Ixx, Ixy, Ixz, Iyy, Iyz, Izz].
    ```
    *
    * @return FX_MOTION_OK     Initialization succeeded.
    * @return FX_MOTION_ERROR  Initialization failed due to invalid parameters or internal errors.
    *
    * @note
    * * All input pointers must be valid and point to properly sized memory.
    * * This function does not allocate or take ownership of input data.
    * * The handle must be initialized before calling this function.
    * * One kinematics handle should not be shared across multiple robot arms after initialization.
    */

    KINEMATICS_SDK_API int FX_L0_Kinematics_init_single_arm(
        FX_MotionHandle handle,
        int RobotSerial, int *type, double DH[8][4], double PNVA[8][4], double BOUND[4][3],
        double GRV[3], double MASS[7], double MCP[7][3], double I[7][6]);

    /**
    * @brief Initialize the environment for dual-arm planning.
    *
    * @param[in] handle Kinematics context handle.
    * @param[in]  RobotSerial  Robot identifier (e.g., 0 for left/A arm, 1 for right/B arm).
    * @param[in]  type         Robot type identifier (input). CCS structure = 1017 and SRS structrue = 1007
    * @param[in]  DH           Denavit–Hartenberg parameters array (8x4).
    * ```
                        Each row typically represents [alpha, a, d, theta].
    ```
    * @param[in]  PNVA         Kinematic auxiliary parameters (8x4), used for extended modeling.
    * ```
                        Each row typically represents [positive joint limit,negtive joint limit,velocity limit,accelaration limit].
    ```
    * @param[in]  BOUND        Constraint boundaries (4x3).
    * ```
                        Each row may represent [a,b,c] which for y=a*x^2+b*x+c.
    ```
    *
    * @return FX_MOTION_OK     Initialization succeeded.
    * @return FX_MOTION_ERROR  Initialization failed due to invalid parameters or internal errors.
    *
     */
    KINEMATICS_SDK_API int FX_L0_Kinematics_init_dual_arm(
        FX_MotionHandle handle,
        int type[2], double DH[2][8][4], double PNVA[2][8][4], double BOUND[2][4][3]);

    /**
     * @brief Solve forward kinematics for one arm.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index, `0` for left arm and `1` for right arm.
     * @param[in] joints Input joint values, length 7.
     * @param[out] pose_matrix Output TCP pose matrix, length 16 in 4x4 row-major order.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    KINEMATICS_SDK_API int FX_L0_Kinematics_forward_kinematics(
        FX_MotionHandle handle,
        int robot_serial,
        double joints[7],
        double pose_matrix[16]);

    /**
     * @brief Solve the Jacobian matrix for one arm.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index, `0` for left arm and `1` for right arm.
     * @param[in] joints Input joint values, length 7.
     * @param[out] jacobian Output Jacobian, length 42 in 6x7 row-major order.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    KINEMATICS_SDK_API int FX_L0_Kinematics_jacobian(
        FX_MotionHandle handle,
        int robot_serial,
        double joints[7],
        double jacobian[42]);

    /**
     * @brief Solve inverse kinematics for one arm.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] robot_serial Arm index, `0` for left arm and `1` for right arm.
     * @param[in,out] params IK input/output parameters including target pose,
     *                      reference joints, and solution buffer.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    KINEMATICS_SDK_API int FX_L0_Kinematics_inverse_kinematics(
        FX_MotionHandle handle,
        int robot_serial,
        FX_InvKineSolverParams *params);

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
    KINEMATICS_SDK_API int FX_L0_Kinematics_set_body_condition(
        FX_MotionHandle handle,
        double std_body[3],
        double k_body[3],
        double std_left_len,
        double k_left,
        double std_right_len,
        double k_right);

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
    KINEMATICS_SDK_API int FX_L0_Kinematics_body_forward(
        FX_MotionHandle handle,
        double jv[3],
        double left_shoulder_matrix[16],
        double right_shoulder_matrix[16]);

    /**
     * @brief Compute body joint values from dual-arm TCP positions.
     *
     * @param[in] handle Kinematics context handle.
     * @param[in] left_tcp Left TCP position, length 3.
     * @param[in] right_tcp Right TCP position, length 3.
     * @param[out] out_body_joints Output body joints, length 3.
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    KINEMATICS_SDK_API int FX_L0_Kinematics_calc_body_position(
        FX_MotionHandle handle,
        double left_tcp[3],
        double right_tcp[3],
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
    KINEMATICS_SDK_API int FX_L0_Kinematics_calc_body_position_with_ref(
        FX_MotionHandle handle,
        double ref_body_joints[3],
        double left_tcp[3],
        double right_tcp[3],
        double out_body_joints[3]);

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
    KINEMATICS_SDK_API int FX_L0_Kinematics_plan_joint_move(
        FX_MotionHandle handle,
        int robot_serial,
        double start_joints[7],
        double end_joints[7],
        double vel_ratio,
        double acc_ratio,
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
    KINEMATICS_SDK_API int FX_L0_Kinematics_plan_linear_move(
        FX_MotionHandle handle,
        int robot_serial,
        double start_xyzabc[6],
        double end_xyzabc[6],
        double ref_joints[7],
        double vel,
        double acc,
        int freq,
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
    KINEMATICS_SDK_API int FX_L0_Kinematics_plan_linear_keep_joints(
        FX_MotionHandle handle,
        int robot_serial,
        double start_joints[7],
        double end_joints[7],
        double vel,
        double acc,
        int freq,
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
     * @return `FX_MOTION_OK` on success, otherwise `FX_MOTION_ERROR`.
     */
    KINEMATICS_SDK_API int FX_L0_Kinematics_multi_points_set_movl_start(FX_MotionHandle handle, int robot_serial,
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
    KINEMATICS_SDK_API int FX_L0_Kinematics_multi_points_set_movl_next_points(FX_MotionHandle handle, int robot_serial,
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
    KINEMATICS_SDK_API int FX_L0_Kinematics_multi_points_get_movl_path(FX_MotionHandle handle,
                                                                       double *point_set_handle, int *point_num);
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
    KINEMATICS_SDK_API int FX_L0_Kinematics_plan_dual_arm_fixed_body(
        FX_MotionHandle handle,
        DualArmFixedBodyParams *params,
        double *left_point_set,
        double *right_point_set,
        int *point_num);

    /**
     * @brief Convert an `XYZABC` pose to a 4x4 transform matrix.
     *
     * @param[in] xyzabc Input pose, length 6, angles in degrees.
     * @param[out] matrix Output matrix, length 16 in 4x4 row-major order.
     */
    KINEMATICS_SDK_API void FX_L0_XYZABC2Matrix(
        double xyzabc[6],
        double matrix[16]);

    /**
     * @brief Convert a 4x4 transform matrix to an `XYZABC` pose.
     *
     * @param[in] matrix Input matrix, length 16 in 4x4 row-major order.
     * @param[out] xyzabc Output pose, length 6, angles in degrees.
     */
    KINEMATICS_SDK_API void FX_L0_Matrix2XYZABC(
        double matrix[16],
        double xyzabc[6]);

    /**
     * @brief Create a point set object.
     *
     * @return A valid point set handle on success, or `nullptr` on failure.
     */
    KINEMATICS_SDK_API void *FX_L0_CPointSet_Create();

    /**
     * @brief Destroy a point set object.
     *
     * @param[in] pset Point set handle.
     */
    KINEMATICS_SDK_API void FX_L0_CPointSet_Destroy(void *pset);

    /**
     * @brief Initialize the point set type.
     *
     * @param[in] pset Point set handle.
     * @param[in] ptype Point set type value corresponding to `PoinType`.
     * @return `FX_TRUE` on success, otherwise `FX_FALSE`.
     */
    KINEMATICS_SDK_API int FX_L0_CPointSet_OnInit(void *pset, int ptype);

    /**
     * @brief Get the number of points stored in the point set.
     *
     * @param[in] pset Point set handle.
     * @return Number of points in the point set.
     */
    KINEMATICS_SDK_API int FX_L0_CPointSet_OnGetPointNum(void *pset);

    /**
     * @brief Get a point by index.
     *
     * @param[in] pset Point set handle.
     * @param[in] pos Point index.
     * @return Pointer to point data on success, or `nullptr` on failure.
     */
    KINEMATICS_SDK_API double *FX_L0_CPointSet_OnGetPoint(void *pset, int pos);

    /**
     * @brief Append one point into the point set.
     *
     * @param[in] pset Point set handle.
     * @param[in] point_value Input point data.
     * @return `FX_TRUE` on success, otherwise `FX_FALSE`.
     */
    KINEMATICS_SDK_API int FX_L0_CPointSet_OnSetPoint(void *pset, double point_value[]);

    /**
     * @brief Transfer CPointset to double type.
     *
     * @param[in] pset Point set handle.
     * @param[out] point_value Output point data.
     * @return `FX_TRUE` on success, otherwise `FX_FALSE`.
     */
    KINEMATICS_SDK_API int FX_L0_CPointSet_OnAppendPoint(void *pset, double *point_value);

#ifdef __cplusplus
}
#endif

#endif
