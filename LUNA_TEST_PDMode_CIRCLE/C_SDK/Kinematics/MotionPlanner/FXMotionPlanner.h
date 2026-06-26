#ifndef _FX_FXPLN_H_
#define _FX_FXPLN_H_

#include "FXArmKinematics.h"
#include "FXAxisPln.h"

/**
 * @brief Motion-planning interface for single-arm and dual-arm MAX trajectories.
 *
 * The planner supports joint-space motion, Cartesian linear motion, continuous
 * multi-point Cartesian linear motion, dual-arm synchronized linear motion with
 * a fixed body, pose conversion utilities, and CPointSet helper wrappers.
 */
class CFxPln
{
public:
	/**
	* @brief Initialize the kinematics and dynamics environment for a robot arm.
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
	* @return FX_TRUE on success; otherwise FX_FALSE.
	* @note
	* * All input arrays must be pre-allocated with correct dimensions.
	* * The pointer @p type must be valid (non-null).
	* * This function does not take ownership of input memory.
	* * One kinematics instance should not be shared across multiple robot arms after initialization.
	*/
	FX_BOOL OnInitEnv_SingleArm(FX_INT32 RobotSerial, FX_INT32 *type, FX_DOUBLE DH[8][4], FX_DOUBLE PNVA[8][4], FX_DOUBLE BOUND[4][3]);

	/**
	 * @brief Set the tool transformation matrix for the specified robot arm.
	 *
	 * This function configures the tool pose relative to the robot flange.
	 * The tool matrix is a 4x4 homogeneous transformation matrix in row-major order.
	 *
	 * @param[in] RobotSerial Robot arm index, `0` for left arm and `1` for right arm.
	 * @param[in] tool Tool transformation matrix relative to the flange coordinate system.
	 *
	 * @return `FX_TRUE` if the tool is set successfully, otherwise `FX_FALSE`.
	 */
	FX_BOOL OnPlnSetTool(FX_INT32 RobotSerial, Matrix4 tool);

	/**
	 * @brief Remove the tool configuration for the specified robot arm.
	 *
	 * This function clears the currently configured tool transformation
	 * and restores the default flange coordinate system.
	 *
	 * @param[in] RobotSerial Robot arm index, `0` for left arm and `1` for right arm.
	 */
	FX_VOID OnPlnRemoveTool(FX_INT32 RobotSerial);
	/**
	 * @brief Plan a joint-space trajectory between two 7-axis joint states for one robot arm.
	 * @param start_joint Start joint angles in degrees.
	 * @param end_joint End joint angles in degrees.
	 * @param vel_ratio Velocity ratio used by the joint planner.
	 * @param acc_ratio Acceleration ratio used by the joint planner.
	 * @param ret_pset Output point set that receives the planned 7D joint trajectory.
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 */
	FX_BOOL OnMovJ(Vect7 start_joint, Vect7 end_joint, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio, FX_INT32 freq, CPointSet *ret_pset);

	/**
	 * @brief Plan a single-arm Cartesian linear trajectory.
	 * @param Start_XYZABC Start TCP pose [X, Y, Z, A, B, C]. Translation is in millimeters and rotation is in degrees.
	 * @param End_XYZABC End TCP pose [X, Y, Z, A, B, C]. Translation is in millimeters and rotation is in degrees.
	 * @param Ref_Joints Reference joint angles in degrees used for inverse-kinematics solution selection.
	 * @param Vel Cartesian linear velocity in mm/s.
	 * @param ACC Cartesian linear acceleration in mm/s².
	 * @param freq Output trajectory frequency in hertz.
	 * @param pset Output point set that receives the planned 7D joint trajectory.
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 */
	FX_BOOL OnMovL(Vect6 Start_XYZABC, Vect6 End_XYZABC, Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32 freq, CPointSet *pset);

	/**
	 * @brief Plan a Cartesian linear trajectory constrained by start and stop joint states.
	 * @param startjoints Start joint angles in degrees.
	 * @param stopjoints Stop joint angles in degrees.
	 * @param vel Cartesian linear velocity in mm/s.
	 * @param acc Cartesian linear acceleration in mm/s² squared.
	 * @param freq Output trajectory frequency in hertz.
	 * @param pset Output point set that receives the planned 7D joint trajectory.
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 * @note The Cartesian line is generated from the FK poses of startjoints and stopjoints while preserving the requested end joint state.
	 */
	FX_BOOL OnMovL_KeepJ(Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq, CPointSet *pset);

	/**
	 * @brief Start a continuous multi-point Cartesian linear trajectory.
	 * @param refjoints Reference joint angles in degrees used for inverse-kinematics solution selection.
	 * @param Start_XYZABC Start TCP pose [X, Y, Z, A, B, C]. Translation is in millimeters and rotation is in degrees.
	 * @param End_XYZABC First target TCP pose [X, Y, Z, A, B, C]. Translation is in millimeters and rotation is in degrees.
	 * @param Allow_Range Blend allowance between adjacent linear segments in millimeters. A larger value gives smoother blending with more path deviation.
	 * @param ZSP_Type Null-space constraint type for IK solution selection. Use 0 for nearest reference-joint distance; use 1 for reference arm-plane constraint.
	 * @param ZSP_Para Null-space constraint parameters. For type 0 pass zeros; for type 1 pass the arm-plane tangent vector in the first three elements.
	 * @param Vel Cartesian linear velocity in mm/s.
	 * @param ACC Cartesian linear acceleration in mm/s².
	 * @param freq Output trajectory frequency in hertz.
	 * @note This function needs to be used in combination. After calling this function, call MultiPoints_Set_MovL_NextPoints() to append the next target pose, and call MultiPoints_Get_MovL_Path() to retrieve the generated trajectory after all targets have been set.
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 */
	FX_BOOL MultiPoints_Set_MovL_Start(Vect7 refjoints, Vect6 Start_XYZABC, Vect6 End_XYZABC,
										  FX_DOUBLE Allow_Range, FX_INT32 ZSP_Type, Vect6 ZSP_Para, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32 freq);

	/**
	 * @brief Append the next target to a continuous multi-point Cartesian linear trajectory.
	 * @param Next_XYZABC Next TCP pose [X, Y, Z, A, B, C]. Translation is in millimeters and rotation is in degrees.
	 * @param Allow_Range Blend allowance between adjacent linear segments in millimeters.
	 * @param ZSP_Type Null-space constraint type for IK solution selection.
	 * @param ZSP_Para Null-space constraint parameters matching ZSP_Type.
	 * @param Vel Cartesian linear velocity in mm/s.
	 * @param ACC Cartesian linear acceleration in mm/s².
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 * @note The start pose and reference joints are taken from the previous segment end state.
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 */
	FX_BOOL MultiPoints_Set_MovL_NextPoints(Vect6 Next_XYZABC,
											   FX_DOUBLE Allow_Range, FX_INT32 ZSP_Type, Vect6 ZSP_Para, FX_DOUBLE Vel, FX_DOUBLE ACC);

	/**
	 * @brief Retrieve the generated continuous multi-point Cartesian trajectory.
	 * @param ret_pset Output point set that receives all planned joint trajectory points.
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 */
	FX_BOOL MultiPoints_Get_MovL_Path(CPointSet *ret_pset);

	/**
	 * @brief Plan time-synchronized Cartesian linear trajectories for both arms with a fixed body posture.
	 * @param ASPP Input dual-arm planning parameters, including body posture, left/right start and end poses, reference joints, null-space constraints, velocity, acceleration, frequency, synchronization type, and coordinate-frame flag. For details, please refer to the struct definition of DualArm_FixedBody.
	 * @param Left_Arm_Pln_Path Output point set for the left-arm planned joint trajectory.
	 * @param Right_Arm_Pln_Path Output point set for the right-arm planned joint trajectory.
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 */
	FX_BOOL OnMovL_DualArm_FixBody(ArmsSynchronousPlanningParams *ASPP, CPointSet *Arm0_Pln_Path, CPointSet *Arm1_Pln_Path);

	/**
	 * @brief Convert an XYZABC pose vector to a 4x4 homogeneous matrix using degrees.
	 * @param xyzabc Input pose [X, Y, Z, A, B, C]. Translation is in millimeters and rotation is in degrees.
	 * @param m Output 4x4 homogeneous pose matrix.
	 */
	FX_VOID XYZABC2Matrix4_DEG(FX_DOUBLE xyzabc[6], FX_DOUBLE m[4][4]);

	/**
	 * @brief Convert a 4x4 homogeneous matrix to an XYZABC pose vector using degrees.
	 * @param m Input 4x4 homogeneous pose matrix.
	 * @param xyzabc Output pose [X, Y, Z, A, B, C]. Translation is in millimeters and rotation is in degrees.
	 */
	FX_VOID Matrix42XYZABC_DEG(FX_DOUBLE m[4][4], FX_DOUBLE xyzabc[6]);

public:
	/** @brief Construct an uninitialized motion planner. */
	CFxPln();

	/** @brief Destroy the motion planner. */
	~CFxPln();
	FX_INT32 GetLastError() const;

protected:
	CAxisPln m_AxisPln;
	CAxisJointPln m_AxisJointPln;
	CFxKineIF m_KineIF; // for single arm cartesian space or joint space motion planning

	CFxKineIF m_Kine_Left_Arm; // for dual arm cartesian space motion planning
	CFxKineIF m_Kine_Right_Arm;
	FX_INT32 m_LastError;
};

#endif
