#ifndef _FX_KINE_IF_H
#define _FX_KINE_IF_H

#include "FX_Env.h"
#include "FxKinematics.h"
#include "AxisPln.h"
#include "PointSet.h"

/**
 * @brief Single-arm kinematics interface.
 *
 * The object is initialized for one robot arm at a time. After
 * L0_OnInitEnv() succeeds, all forward kinematics, inverse kinematics,
 * Jacobian, tool, and load-identification calls use the selected arm.
 */
class CFxKineIF
{
public:
	/**
	 * @brief Enable or disable SDK logging for the kinematics module.
	 * @param log_tag_input Set to 1 to enable logging; set to 0 to disable it.
	 */
	FX_VOID L0_OnLogSwitch(FX_INT32 log_tag_input);

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
	* @param[in]  GRV          Gravity vector (3 elements), e.g., [gx, gy, gz].
	* @param[in]  MASS         Link masses (7 elements), corresponding to each joint/link.
	* @param[in]  MCP          Center of mass positions for each link (7x3).
	* @param[in]  I            Inertia parameters for each link (7x6),
	* ```
						typically in the form [Ixx, Ixy, Ixz, Iyy, Iyz, Izz].
	```
	*
	* @return FX_TRUE on success; otherwise FX_FALSE.
	* @note
	* * All input arrays must be pre-allocated with correct dimensions.
	* * The pointer @p type must be valid (non-null).
	* * This function does not take ownership of input memory.
	* * One kinematics instance should not be shared across multiple robot arms after initialization.
    */

	FX_BOOL L0_OnInitEnv(FX_INT32 RobotSerial, FX_INT32 *type, FX_DOUBLE DH[8][4], FX_DOUBLE PNVA[8][4], FX_DOUBLE BOUND[4][3],
						 FX_DOUBLE GRV[3], FX_DOUBLE MASS[7], FX_DOUBLE MCP[7][3], FX_DOUBLE I[7][6]);

	/**
	 * @brief Set the active tool transform for the initialized arm.
	 * @param tool 4x4 homogeneous transform from the flange frame to the tool/TCP frame.
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 * @note If no tool is set, FK and IK are computed at the flange frame. After a tool is set, FK and IK are computed at the tool/TCP frame.
	 */
	FX_BOOL L0_OnSetTool(Matrix4 tool);

	/**
	 * @brief Remove the active tool transform from the initialized arm.
	 */
	FX_VOID L0_OnRmvTool();

	/**
	 * @brief Solve forward kinematics for the initialized arm.
	 * @param joints Seven joint angles in degrees.
	 * @param pgos Output 4x4 homogeneous pose matrix in the base frame. The pose is for the tool/TCP frame when a tool is set, otherwise for the flange frame.
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 */
	FX_BOOL L0_OnSolveArmFK(Vect7 joints, Matrix4 pgos);

	/**
	 * @brief Calculate the 6x7 geometric Jacobian for the initialized arm.
	 * @param joints Seven joint angles in degrees.
	 * @param jcb Output 6x7 Jacobian matrix.
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 */
	FX_BOOL L0_OnSolveArmJcb(Vect7 joints, FX_DOUBLE jcb[6][7]);

	/**
	 * @brief Solve inverse kinematics for the initialized arm.
	 * @param solve_para Input/output solve parameter block. Inputs include the target TCP pose, reference joints, and null-space constraint settings. Outputs include the selected joint solution, all candidate solutions, reachability, singularity, and joint-limit flags. For details, please refer to the structdefinition.
	 * @return FX_TRUE when a valid IK solution is found; otherwise FX_FALSE.
	 * @note Joint values use degrees. Target translations use millimeters.
	 */
	FX_BOOL L0_OnSolveArmIK(FX_InvKineSolvePara *solve_para);

	/**
	 * @brief Identify tool load dynamic parameters from recorded trajectory data.
	 * @param path Directory that contains the load-identification data files.
	 * @param mass Output identified tool mass.
	 * @param mr Output first moment vector of the identified tool load.
	 * @param I Output inertia parameters of the identified tool load.
	 * @return Nonzero on success; otherwise 0.
	 */
	FX_INT32 L0_OnIdenDynaLoad(FX_CHAR *path, FX_DOUBLE *mass, Vect3 mr, Vect6 I);

	/** @brief Initialization state flag for the current arm. */
	FX_BOOL m_InitTag;
	/** @brief Current initialized arm selector: 0 for left/A, 1 for right/B. */
	FX_INT32 Kine_Serial;

public:
	/** @brief Construct an uninitialized kinematics interface. */
	CFxKineIF();

	/** @brief Destroy the kinematics interface. */
	~CFxKineIF();

	/**
	 * @brief Get configured position, velocity, and acceleration limits for the initialized arm.
	 * @param type Output robot or arm type identifier.
	 * @param PosNeg Output negative position limits.
	 * @param PosPos Output positive position limits.
	 * @param VelLmt Output velocity limits.
	 * @param AccLmt Output acceleration limits.
	 * @note For internal use
	 * @return FX_TRUE on success; otherwise FX_FALSE.
	 */
	FX_BOOL OnGetArmLmt(FX_INT32 &type, FX_DOUBLE PosNeg[8], FX_DOUBLE PosPos[8], FX_DOUBLE VelLmt[8], FX_DOUBLE AccLmt[8]);

protected:
	CFxIFEnv m_Env;
	CAxisPln m_AxisPln;
};

#endif
