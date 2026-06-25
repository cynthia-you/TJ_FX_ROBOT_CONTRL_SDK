"""
@file example_kine_arms_BasicKinematics.py
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
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map, FX_InvKineSolverParams


def main():
    log_mask = FXLogMask.FX_LOG_INFO_FLAG
    ctrl_obj1 = FXObjType.OBJ_ARM0
    ctrl_obj2 = FXObjType.OBJ_ARM1

    robot = GentoRobot()

    print(f"\n### 1/5 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/5. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/5. Initial kinematics and calculate planning trajectory")
    if robot.init_single_arm_config(0) != 0:
        print("Failed to initialize arm0 kinematics")
        return
    if robot.init_single_arm_config(1) != 0:
        print("Failed to initialize arm1 kinematics")
        return
    print("Arms kinematics initialized")
    robot.kine_log_level(log_mask)

    print(f"\n### 4/5. Forward and inverse solutions verify each other...")
    rt_dict = robot.get_rt_dict()
    fb_joints1 = rt_dict["arms"][0]["fb"]["fb_pos"]
    fb_joints2 = rt_dict["arms"][1]["fb"]["fb_pos"]
    print(f"{ctrl_obj1} current pos: {fb_joints1}")
    print(f"{ctrl_obj2} current pos: {fb_joints2}")

    arm0_mat_under_tcp = None
    arm1_mat_under_tcp = None
    ret = robot.forward_kinematics(0, fb_joints1)
    if ret:
        arm0_mat_under_tcp = ret
        print(f"\narm0 fk matrix:{arm0_mat_under_tcp}")
        arm0_xyzabc_under_tcp = robot.matrix2xyzabc(arm0_mat_under_tcp)
        print(f"arm0 TCP xyzabc: {arm0_xyzabc_under_tcp}")
    else:
        print("arm0 FK failed")

    ret = robot.forward_kinematics(1, fb_joints2)
    if ret:
        arm1_mat_under_tcp = ret
        print(f"arm1 fk matrix:{arm1_mat_under_tcp}")
        arm1_xyzabc_under_tcp = robot.matrix2xyzabc(arm1_mat_under_tcp)
        print(f"arm1 TCP xyzabc: {arm1_xyzabc_under_tcp}")
    else:
        print("arm1 FK failed")

    arm0_ik_params = FX_InvKineSolverParams()
    arm1_ik_params = FX_InvKineSolverParams()
    for i in range(7):
        arm0_ik_params.m_Input_IK_RefJoint[i] = fb_joints1[i]
        arm1_ik_params.m_Input_IK_RefJoint[i] = fb_joints2[i]
    for i in range(4):
        for j in range(4):
            arm0_ik_params.m_Input_IK_TargetTCP[i * 4 + j] = arm0_mat_under_tcp[i][j]
            arm1_ik_params.m_Input_IK_TargetTCP[i * 4 + j] = arm1_mat_under_tcp[i][j]
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

    if robot.check_sequences_approx_equal(fb_joints1,arm0_ik_joints):
        print(f"Arm0 IK joints == current joints")
    else:
        print(f"Error: Arm0 IK joints != current joints")

    if robot.check_sequences_approx_equal(fb_joints2,arm1_ik_joints):
        print(f"Arm1 IK joints == current joints")
    else:
        print(f"Error: Arm1 IK joints != current joints")

    robot.cleanup()
    print(f"\n### 5/5. Task finished.")


if __name__ == "__main__":
    main()
