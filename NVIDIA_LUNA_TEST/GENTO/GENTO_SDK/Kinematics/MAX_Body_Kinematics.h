#ifndef _FX_KINE_MAX_H_
#define _FX_KINE_MAX_H_

#include "FXMath.h"

/**
 * @brief MAX body kinematics and body-compliance compensation interface.
 *
 * This class converts body posture values to the left and right arm base
 * poses, and estimates compensated body posture from left/right target
 * positions and stiffness settings.
 */
class CFxKineMAX
{
public:
	/** @brief Construct a MAX body kinematics object with default parameters. */
	CFxKineMAX();

	/** @brief Destroy the MAX body kinematics object. */
	~CFxKineMAX();

	/**
	 * @brief Enable or disable SDK logging for the MAX body kinematics module.
	 * @param log_tag_input Set to 1 to enable logging; set to 0 to disable it.
	 */
	FX_VOID L0_OnLogSwitch(FX_INT32 log_tag_input);

	/**
	 * @brief Set the reference body posture and stiffness parameters used by body compensation.
	 * @param std_Body Reference body vector [lift, pitch, rotation]. Lift is in millimeters, angles are in degrees.
	 * @param k_Body Body stiffness vector for lift, pitch, and rotation compensation.
	 * @param std_L_len Reference left-side support/load length used by the compensation model.
	 * @param k_L Left-side support/load stiffness coefficient.
	 * @param std_R_len Reference right-side support/load length used by the compensation model.
	 * @param k_R Right-side support/load stiffness coefficient.
	 */
	FX_VOID L0_OnSetCondition(Vect3 std_Body,Vect3 k_Body,FX_DOUBLE std_L_len,FX_DOUBLE k_L, FX_DOUBLE std_R_len, FX_DOUBLE k_R);

	/**
	 * @brief Calculate left and right arm base poses from a MAX body posture.
	 * @param jv Body vector [lift, pitch, rotation]. Lift is in millimeters, angles are in degrees.
	 * @param pgL Output 4x4 pose matrix of the left arm base in the world frame.
	 * @param pgR Output 4x4 pose matrix of the right arm base in the world frame.
	 */
	FX_VOID L0_OnKineLR(Vect3 jv, Matrix4 pgL, Matrix4 pgR);

	/**
	 * @brief Estimate the compensated MAX body posture without an explicit reference posture.
	 * @param tpos1 Left-side target/support point position in the world frame.
	 * @param tpos2 Right-side target/support point position in the world frame.
	 * @param ret_pos Output body vector [lift, pitch, rotation]. Lift is in millimeters, angles are in degrees.
	 */
	FX_VOID L0_OnCalBody(Vect3 tpos1, Vect3 tpos2, Vect3 ret_pos);

	/**
	 * @brief Estimate the compensated MAX body posture from a reference posture.
	 * @param refjv Reference body vector [lift, pitch, rotation]. Lift is in millimeters, angles are in degrees.
	 * @param tpos1 Left-side target/support point position in the world frame.
	 * @param tpos2 Right-side target/support point position in the world frame.
	 * @param ret_pos Output body vector [lift, pitch, rotation]. Lift is in millimeters, angles are in degrees.
	 */
	FX_VOID L0_OnCalBody_withref(Vect3 refjv, Vect3 tpos1, Vect3 tpos2, Vect3 ret_pos);
protected:

	/**
	 * @brief Calculate body, left-arm, and right-arm base poses and Jacobians for a body posture.
	 * @param jv Body vector [lift, pitch, rotation].
	 * @param pgB Output 4x4 pose matrix of the body base.
	 * @param pgL Output 4x4 pose matrix of the left arm base.
	 * @param pgR Output 4x4 pose matrix of the right arm base.
	 * @param jcbL Output left-arm body Jacobian.
	 * @param jcbR Output right-arm body Jacobian.
	 */
	FX_VOID OnKineBLR(Vect3 jv, Matrix4 pgB, Matrix4 pgL, Matrix4 pgR, Matrix3 jcbL, Matrix3 jcbR);

	/**
	 * @brief Calculate left and right arm base poses and Jacobians for a body posture.
	 * @param jv Body vector [lift, pitch, rotation].
	 * @param pgL Output 4x4 pose matrix of the left arm base.
	 * @param pgR Output 4x4 pose matrix of the right arm base.
	 * @param jcbL Output left-arm body Jacobian.
	 * @param jcbR Output right-arm body Jacobian.
	 */
	FX_VOID OnKineLR(Vect3 jv, Matrix4 pgL, Matrix4 pgR, Matrix3 jcbL, Matrix3 jcbR);

	Vect3 m_std_body;
	Vect3 m_k_body;
	FX_DOUBLE m_std_L_len;
	FX_DOUBLE m_k_L;
	FX_DOUBLE m_std_R_len;
	FX_DOUBLE m_k_R;
	FX_DOUBLE m_dArm2Pitch;
	FX_DOUBLE m_dShoulder;

	/**
	 * @brief Calculate the distance from a pose origin to a target point.
	 * @param ps Source pose matrix.
	 * @param t Target point.
	 * @param ret_vct Output vector from the pose origin to the target point.
	 * @return Distance between the pose origin and the target point.
	 */
	FX_DOUBLE CalLen(Matrix4 ps, Vect3 t, Vect3 ret_vct);

	/**
	 * @brief Calculate a stiffness-scaled force vector for a support/load point.
	 * @param k Stiffness coefficient.
	 * @param stdlen Reference length.
	 * @param ps Source pose matrix.
	 * @param pos Target/support point position.
	 * @param ret_vct Output force direction vector scaled by the length error and stiffness.
	 * @return Length-error scale value used to build the force vector.
	 */
	FX_DOUBLE CalForce(FX_DOUBLE k, FX_DOUBLE stdlen, Matrix4 ps, Vect3 pos, Vect3 ret_vct);

	/**
	 * @brief Calculate force/torque contribution for a support/load point.
	 * @param k Stiffness coefficient.
	 * @param stdlen Reference length.
	 * @param pb Body base pose matrix.
	 * @param ps Source pose matrix.
	 * @param pos Target/support point position.
	 * @param ret_tf Output force/torque vector contribution.
	 */
	FX_VOID CalFT(FX_DOUBLE k, FX_DOUBLE stdlen, Matrix4 pb, Matrix4 ps, Vect3 pos, Vect3 ret_tf);

	Matrix4 m_ArmL_Set;
	Matrix4 m_ArmR_Set;

	Matrix4 m_ArmL_SetB;
	Matrix4 m_ArmR_SetB;

};
#endif
