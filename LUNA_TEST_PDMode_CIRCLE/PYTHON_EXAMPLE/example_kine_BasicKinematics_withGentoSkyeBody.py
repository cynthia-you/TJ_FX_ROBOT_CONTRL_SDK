"""
@file example_kine_BasicKinematics_withGentoSkyeBody.py
@brief Example demonstrating how to calculate forward kinematics martix from joints,
       and inverse kinematics from matrix to joints.
       .

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Initial kinematics of two arms
    4. Forward and inverse solutions verify each other

"""

import sys
import time
import threading
import copy
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map, FX_InvKineSolverParams


def main():
    log_mask = FXLogMask.FX_LOG_INFO_FLAG

    robot = GentoRobot()

    print(f"\n### 1/16 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    if robot.get_robot_type()!='GentoSkye':
        print(f"Example is for GentoSkye robot, your robot is {robot.get_robot_type()} ")
        return

    print(f"\n### 2/16. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/16. Initial kinematics...")
    if robot.init_single_arm_config(0) != 0:
        print("Failed to initialize arm0 kinematics")
        return
    if robot.init_single_arm_config(1) != 0:
        print("Failed to initialize arm1 kinematics")
        return
    print("Arms kinematics initialized")
    robot.kine_log_level(log_mask)

    print(f"\n### 4/16. Set Gento Skey body reference pos...")
    ref_body_joints = [80.0, 5.0, 0.0]
    ret = robot.skye_body_forward_kinematics(ref_body_joints)
    if isinstance(ret, tuple):
        body_mat_under_arm0_base = ret[0]
        body_mat_under_arm1_base = ret[1]
        print(f'body matrix under arm0 base:{body_mat_under_arm0_base}')
        print(f'body matrix under arm1 base:{body_mat_under_arm1_base}')
    else:
        print("Body FK failed.")

    print(f"\n### 5/16. Arms FK...")
    left_ref_joints = [44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.10]
    right_ref_joints = [44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.10]

    ret = robot.forward_kinematics(0, left_ref_joints)
    if ret:
        arm0_mat_under_tcp = ret
        print(f"arm0 fk matrix:{arm0_mat_under_tcp}")
    else:
        print("arm0 FK failed")

    ret = robot.forward_kinematics(1, right_ref_joints)
    if ret:
        arm1_mat_under_tcp = ret
        print(f"arm1 fk matrix:{arm1_mat_under_tcp}")
    else:
        print("arm1 FK failed")

    print("\n### 6/16. Body pose matrix under arms's TCP...")
    body_mat_under_arm0_tcp = robot.matrix_multiply(body_mat_under_arm0_base, arm0_mat_under_tcp)
    body_mat_under_arm1_tcp = robot.matrix_multiply(body_mat_under_arm1_base, arm1_mat_under_tcp)
    print(f"body under arm0 tcp:{body_mat_under_arm0_tcp}")
    print(f"body under arm1 tcp:{body_mat_under_arm1_tcp}")

    print("\n### 7/16. Body move offset...")
    body_target_mat_under_arm0_tcp = copy.deepcopy(body_mat_under_arm0_tcp)
    body_target_mat_under_arm1_tcp = copy.deepcopy(body_mat_under_arm1_tcp)

    body_target_mat_under_arm0_tcp[0][-1] += 20
    body_target_mat_under_arm0_tcp[2][-1] += 15
    body_target_mat_under_arm1_tcp[0][-1] += 20
    body_target_mat_under_arm1_tcp[2][-1] += 5

    body_target_xyzabc_under_arm0_tcp = robot.matrix2xyzabc(body_target_mat_under_arm0_tcp)
    body_target_xyzabc_under_arm1_tcp = robot.matrix2xyzabc(body_target_mat_under_arm1_tcp)
    print(
        f"After body move, the pose of body under arms tcp:\n under arm0 tcp:{body_target_xyzabc_under_arm0_tcp}\n under arm1 tcp:{body_target_xyzabc_under_arm1_tcp}")

    print("\n### 8/16. Obtain the translational component for body IK...")
    arm0_base_ref_pos = [body_mat_under_arm0_base[0][3], body_mat_under_arm0_base[1][3], body_mat_under_arm0_base[2][3]]
    arm1_base_ref_pos = [body_mat_under_arm1_base[0][3], body_mat_under_arm1_base[1][3], body_mat_under_arm1_base[2][3]]

    arm0_tcp_ref_pos = [body_mat_under_arm0_tcp[0][3], body_mat_under_arm0_tcp[1][3],body_mat_under_arm0_tcp[2][3]]
    arm1_tcp_ref_pos = [body_mat_under_arm1_tcp[0][3], body_mat_under_arm1_tcp[1][3],body_mat_under_arm1_tcp[2][3]]

    body_target_pos_under_arm0 = [body_target_mat_under_arm0_tcp[0][3], body_target_mat_under_arm0_tcp[1][3],body_target_mat_under_arm0_tcp[2][3]]
    body_target_pos_under_arm1 = [body_target_mat_under_arm1_tcp[0][3], body_target_mat_under_arm1_tcp[1][3],body_target_mat_under_arm1_tcp[2][3]]


    print("\n### 9/16. Get reference length...")
    arm0_std_len = robot.distance3(arm0_base_ref_pos, arm0_tcp_ref_pos)
    arm1_std_len = robot.distance3(arm1_base_ref_pos, arm1_tcp_ref_pos)
    print(f"arm0_base_ref_pos:{arm0_base_ref_pos}")
    print(f"arm1_base_ref_pos:{arm1_base_ref_pos}")
    print(f"arm0_tcp_ref_pos:{arm0_tcp_ref_pos}")
    print(f"arm1_tcp_ref_pos:{arm1_tcp_ref_pos}")
    print(f"body_target_pos_under_arm0:{body_target_pos_under_arm0}")
    print(f"body_target_pos_under_arm1:{body_target_pos_under_arm1}")
    print(f"arm0_std_len:{arm0_std_len}")
    print(f"arm1_std_len:{arm1_std_len}")

    print("\n### 10/16. Set body condition...")
    body_stiffness = [10.0, 10.0, 10.0]
    ret = robot.set_skye_body_condition(ref_body_joints, body_stiffness, arm0_std_len, 1, arm1_std_len, 1)
    if ret != 1:
        print("[ERROR] Set body condition failed.")

    print("\n### 11/16. Body IK...")
    ret = robot.skye_body_inverse_kinematics(body_target_pos_under_arm0, body_target_pos_under_arm1)
    if ret:
        body_target_joints = ret
        print(f'after IK, the joints[ypr] of target pose is:{body_target_joints}')
    else:
        print(f'body IK FAILED')

    print("\n### 12/16. Body FK to shoulders...")
    body_target_mat_under_arm0_base=[[0]*4,
                                     [0]*4,
                                     [0]*4,
                                     [0]*4]
    body_target_mat_under_arm1_base=[[0]*4,
                                     [0]*4,
                                     [0]*4,
                                     [0]*4]
    ret = robot.skye_body_forward_kinematics(body_target_joints)
    if ret :
        body_target_mat_under_arm0_base = ret[0]
        body_target_mat_under_arm1_base = ret[1]
        print(f'target body joint fk:\nbody matrix under arm0 base:{body_target_mat_under_arm0_base}')
        print(f'body matrix under arm1 base:{body_target_mat_under_arm1_base}')
    else:
        print("[ERROR] Body FK failed.")

    print("\n### 13/16. Transform the target pose from the body coordinate system to the shoulder coordinate system...")
    arm0_shoulder_inv = robot.invert_rigid_matrix4x4(body_target_mat_under_arm0_base)
    arm1_shoulder_inv = robot.invert_rigid_matrix4x4(body_target_mat_under_arm1_base)
    print(f"arm0_shoulder_inv:{arm0_shoulder_inv}")
    print(f"arm1_shoulder_inv:{arm1_shoulder_inv}")

    arm0_target_base = robot.matrix_multiply(arm0_shoulder_inv, body_target_mat_under_arm0_tcp)
    arm1_target_base = robot.matrix_multiply(arm1_shoulder_inv, body_target_mat_under_arm1_tcp)
    print(f"arm0_target_base:{arm0_target_base}")
    print(f"arm1_target_base:{arm1_target_base}")

    print("\n### 14/16. Arms IK...")
    arm0_ik_params = FX_InvKineSolverParams()
    arm1_ik_params = FX_InvKineSolverParams()
    for i in range(7):
        arm0_ik_params.m_Input_IK_RefJoint[i]=left_ref_joints[i]
        arm1_ik_params.m_Input_IK_RefJoint[i] = right_ref_joints[i]
    for i in range(4):
        for j in range(4):
            arm0_ik_params.m_Input_IK_TargetTCP[i * 4 + j] = arm0_target_base[i][j]
            arm1_ik_params.m_Input_IK_TargetTCP[i * 4 + j] = arm1_target_base[i][j]
    arm0_ik_joints = [0] * 7
    arm1_ik_joints = [0] * 7

    arm0_ik_result_structure = robot.inverse_kinematics(0, arm0_ik_params)
    if arm0_ik_result_structure:
        print(f'ik joints(close to reference joints)::{arm0_ik_result_structure.m_Output_RetJoint[:]}')
        print(f'IK exceed joints: {arm0_ik_result_structure.m_Output_IsJntExd}')
        print(f'IK exceed joint tags: {arm0_ik_result_structure.m_Output_JntExdTags[:]}')
        print(f'Positive limits of joints: {arm0_ik_result_structure.m_Output_RunLmtP[:]}')
        print(f'Negative limits of joints: {arm0_ik_result_structure.m_Output_RunLmtN[:]}')
        print(f'Number of ik results:{arm0_ik_result_structure.m_OutPut_Result_Num}')
        print(f'All ik results:{robot.convert_to_8x8_matrix(arm0_ik_result_structure.m_OutPut_AllJoint[:])}')
        arm0_ik_joints = arm0_ik_result_structure.m_Output_RetJoint[:]
    else:
        print("Arm0 IK failed")

    arm1_ik_result_structure = robot.inverse_kinematics(1, arm1_ik_params)
    if arm1_ik_result_structure:
        print(f'ik joints(close to reference joints)::{arm1_ik_result_structure.m_Output_RetJoint[:]}')
        print(f'IK exceed joints: {arm1_ik_result_structure.m_Output_IsJntExd}')
        print(f'IK exceed joint tags: {arm1_ik_result_structure.m_Output_JntExdTags[:]}')
        print(f'Positive limits of joints: {arm1_ik_result_structure.m_Output_RunLmtP[:]}')
        print(f'Negative limits of joints: {arm1_ik_result_structure.m_Output_RunLmtN[:]}')
        print(f'Number of ik results:{arm1_ik_result_structure.m_OutPut_Result_Num}')
        print(f'All ik results:{robot.convert_to_8x8_matrix(arm1_ik_result_structure.m_OutPut_AllJoint[:])}')
        arm1_ik_joints = arm1_ik_result_structure.m_Output_RetJoint[:]
    else:
        print("Arm1 IK failed")

    print("\n### 15/16. FK verify...")
    arm0_fk_mat_solution = robot.forward_kinematics(0, arm0_ik_joints)
    arm1_fk_mat_solution = robot.forward_kinematics(1, arm1_ik_joints)

    arm0_verify_mat = robot.matrix_multiply(body_target_mat_under_arm0_base, arm0_fk_mat_solution)
    arm1_verify_mat = robot.matrix_multiply(body_target_mat_under_arm1_base, arm1_fk_mat_solution)
    print(f"body_mat_under_arm0_base:{body_mat_under_arm0_base}")
    print(f"body_mat_under_arm1_base:{body_mat_under_arm1_base}")
    print(f"arm0_fk_mat_solution:{arm0_fk_mat_solution}")
    print(f"arm1_fk_mat_solution:{arm1_fk_mat_solution}")
    print(f"arm0_verify_mat:{arm0_verify_mat}")
    print(f"arm1_verify_mat:{arm1_verify_mat}")

    arm0_verify_xyzabc = robot.matrix2xyzabc(arm0_verify_mat)
    arm1_verify_xyzabc = robot.matrix2xyzabc(arm1_verify_mat)
    print("arm0 verified pose in body frame", arm0_verify_xyzabc)
    print("arm1 verified pose in body frame", arm1_verify_xyzabc)
    arm0_verify_pos = [arm0_verify_mat[0][3], arm0_verify_mat[1][3], arm0_verify_mat[2][3]]
    arm1_verify_pos = [arm1_verify_mat[0][3], arm1_verify_mat[1][3], arm1_verify_mat[2][3]]
    print(f"arm0 position error  = {robot.distance3(arm0_verify_pos, body_target_pos_under_arm0):.6f} mm")
    print(f"arm1 position error  = {robot.distance3(arm1_verify_pos, body_target_pos_under_arm1):.6f} mm")

    robot.cleanup()
    print("\n### 16/16. Task finished.")


if __name__ == "__main__":
    main()
