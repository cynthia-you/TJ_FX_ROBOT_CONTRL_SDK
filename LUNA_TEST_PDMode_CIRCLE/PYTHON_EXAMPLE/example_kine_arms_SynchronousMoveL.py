"""
@file example_kine_arms_SynchronousMoveL.py
@brief Example demonstrating how to control the L1 robot arm
       run planning trajectory in joint-space position control mode (STATE_POSITION).

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Switch ARM0 into position control mode (STATE_POSITION)(Check and recover the arm state (ERROR → IDLE))
    4. Use the planning function to plan points and execute
    5. Switch ARM0 to IDLE state

Position mode moves the arm along a planned trajectory
to reach the specified joint angles.

@warning Ensure the robot arm is in a safe position and workspace
         before starting position control.

@warning Press Enter key at any time to stop the robot and exit.
"""

import sys
import time
import threading
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map,error_dict, ArmsSynchronousPlanningParams


def main():
    def emergency_stop_thread(robot):
        input()
        print("\nStopping run trajectory...")
        ret_mask = robot.runtime_stop_traj(ctrl_obj_masks)
        if ret_mask != ctrl_obj_masks:
            print(f"brake trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
            print("\nTrigger Emergency stop...")
            robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
            time.sleep(0.1)
            print("Exiting program.")
            sys.exit(0)
        time.sleep(0.1)

    log_mask = FXLogMask.FX_LOG_INFO_FLAG
    ctrl_obj1 = FXObjType.OBJ_ARM0
    ctrl_obj2 = FXObjType.OBJ_ARM1
    ctrl_obj_masks = FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
    target_state = "Position"
    initial_vel = 10
    initial_acc = 10
    global_vel = 100
    global_acc = 100
    zsp_type=1
    zsp_params=[0, 0, -1, 0, 0, 0]
    pln_vel = 100
    pln_acc = 100
    pln_freq = 50
    arm0_start_xyzabc = [509.733, 33.610, 265.953, -169.144, 55.012, -146.752]
    arm1_start_xyzabc = [509.733, -33.610, 265.953, 169.144, 55.012, 146.752]
    arm0_start_pos = [0.876, -25.548, -0.000, -87.472, -18.026, -7.201, -18.925]
    arm1_start_pos = [-0.876, -25.548, 0.000, -87.472, 18.026, -7.201, 18.925]
    arm0_end_xyzabc = [509.733, 233.610, 265.953, -169.144, 55.012, -146.752]
    arm1_end_xyzabc = [509.733, -233.610, 265.953, 169.144, 55.012, 146.752]

    robot = GentoRobot()
    threading.Thread(target=emergency_stop_thread, args=(robot,), daemon=True).start()

    print(f"\n### 1/10 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/10. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/10. Position state switching...")
    arm0_state = robot.current_state(ctrl_obj1)
    if state_map[arm0_state] == "Error":
        ret, system_errorcode = robot.reset_error(ctrl_obj1, 1000)
        if ret != 0:
            print(f"Arm0 reset error failed. Error msg: {error_dict[system_errorcode]}")
            return
    elif state_map[arm0_state] != "IDLE":
        ret = robot.switch_to_idle(ctrl_obj1, 1000)
        if ret != 0:
            print(f"Arm0 switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

    arm1_state = robot.current_state(ctrl_obj2)
    if state_map[arm1_state] == "Error":
        ret, system_errorcode = robot.reset_error(ctrl_obj2, 1000)
        if ret != 0:
            print(f"Arm1 reset failed: {error_dict[system_errorcode]}")
            return
    elif state_map[arm1_state] != "IDLE":
        ret = robot.switch_to_idle(ctrl_obj2, 1000)
        if ret != 0:
            print(f"Arm1 switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

    ret = robot.switch_to_position_mode(ctrl_obj1, 2000, initial_vel, initial_acc)
    if ret != 0:
        print(f"Arm0 switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ret = robot.switch_to_position_mode(ctrl_obj2, 2000, initial_vel, initial_acc)
    if ret != 0:
        print(f"Arm1 switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    rt_dict = robot.get_rt_dict()
    sg_dict = robot.get_sg_dict()
    print("arm0:")
    print(f"current state：{rt_dict["arms"][0]["state"]["cur"]}")
    print(f"current joints:{rt_dict["arms"][0]["fb"]["fb_pos"]}")
    print(f"current vel:{sg_dict["arms"][0]['set']["vel_ratio"]}")
    print(f"current acc:{sg_dict["arms"][0]['set']["acc_ratio"]}")
    print("arm1:")
    print(f"current state：{rt_dict["arms"][1]["state"]["cur"]}")
    print(f"current joints:{rt_dict["arms"][1]["fb"]["fb_pos"]}")
    print(f"current vel:{sg_dict["arms"][1]['set']["vel_ratio"]}")
    print(f"current acc:{sg_dict["arms"][1]['set']["acc_ratio"]}")

    print(f"\n### 4/10. Run initial pos...")
    ret = robot.runtime_set_joint_pos_cmd(ctrl_obj1, arm0_start_pos)
    if ret != 0:
        print(f"Arm0 set joint command failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ret = robot.runtime_set_joint_pos_cmd(ctrl_obj2, arm1_start_pos)
    if ret != 0:
        print(f"Arm1 set joint command failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    while 1:
        rt_dict = robot.get_rt_dict()
        fb_joints1 = rt_dict["arms"][0]["fb"]["fb_pos"]
        fb_joints2 = rt_dict["arms"][1]["fb"]["fb_pos"]
        if robot.check_sequences_approx_equal(fb_joints1, arm0_start_pos) and robot.check_sequences_approx_equal(
            fb_joints2, arm1_start_pos):
            break
        time.sleep(0.001)
    print(f"Arm0 reached at {fb_joints1}")
    print(f"Arm1 reached at {fb_joints2}")

    print(
        f"\n### 5/10. Set the global speed and acceleration to the maximum to prevent limiting the planned speed and acceleration....")
    time.sleep(0.5)
    ret = robot.runtime_set_speed_ratio(ctrl_obj1, global_vel, global_acc)
    if ret != 0:
        print(f"Arm0 set vel and acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ret = robot.runtime_set_speed_ratio(ctrl_obj2, global_vel, global_acc)
    if ret != 0:
        print(f"Arm1 set vel and acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    time.sleep(0.02)
    sg_dict = robot.get_sg_dict()
    print("arm0:")
    print(f"current vel:{sg_dict["arms"][0]['set']["vel_ratio"]}")
    print(f"current acc:{sg_dict["arms"][0]['set']["acc_ratio"]}")
    print("arm1:")
    print(f"current vel:{sg_dict["arms"][1]['set']["vel_ratio"]}")
    print(f"current acc:{sg_dict["arms"][1]['set']["acc_ratio"]}")

    print(f"\n### 6/10. Initial kinematics and calculate planning trajectory")
    if robot.init_single_arm_config(0) != 0:
        print("Failed to initialize arm0 kinematics")
        return
    if robot.init_single_arm_config(1) != 0:
        print("Failed to initialize arm1 kinematics")
        return
    print("Arms kinematics initialized")
    robot.kine_log_level(log_mask)

    print(f"\n### 7/10. Calculate planning trajectory, send trajectory to controller")
    arms_structure_params = ArmsSynchronousPlanningParams()
    arms_structure_params.World_Co_Flag = 0
    arms_structure_params.Sync_Type = 0
    arms_structure_params.Freq = pln_freq
    arms_structure_params.Vel = pln_vel
    arms_structure_params.Acc = pln_acc
    arms_structure_params.Arm0_ZSP_Type = zsp_type
    arms_structure_params.Arm1_ZSP_Type = zsp_type

    for i in range(7):
        arms_structure_params.Arm0_Ref_Joints[i] = arm0_start_pos[i]
        arms_structure_params.Arm1_Ref_Joints[i] = arm1_start_pos[i]

        if i < 6:
            arms_structure_params.Arm0_Start_XYZABC[i] = arm0_start_xyzabc[i]
            arms_structure_params.Arm1_Start_XYZABC[i] = arm1_start_xyzabc[i]

            arms_structure_params.Arm0_End_XYZABC[i] = arm0_end_xyzabc[i]
            arms_structure_params.Arm1_End_XYZABC[i] = arm1_end_xyzabc[i]

            arms_structure_params.Arm0_ZSP_Para[i] = zsp_params[i]
            arms_structure_params.Arm1_ZSP_Para[i] = zsp_params[i]

    ret = robot.plan_linear_synchronous(arms_structure_params)
    if not isinstance(ret, tuple):
        print(f"Synchronous planning failed, error msg: {robot._get_kinematics_error_msg(ret)}")
        return
    raw_array0, raw_array1, point_num = ret
    ret = robot.config_set_traj(FXObjType.OBJ_ARM0, raw_array0, point_num)
    if ret != 0:
        print(f"Arm0 send planning points failed: {robot._get_operate_error_msg(ret)}")
        return
    ret = robot.config_set_traj(FXObjType.OBJ_ARM1, raw_array1, point_num)
    if ret != 0:
        print(f"Arm1 send planning points failed: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 8/10. Run trajectory")
    mask = FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
    ret_mask = robot.runtime_run_traj(mask)
    if ret_mask != mask:
        print(f"Run planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
        return
    while 1:
        sg_dict = robot.get_sg_dict()
        arm0_traj_tag = sg_dict["arms"][0]["get"]["traj_state"]
        arm1_traj_tag = sg_dict["arms"][1]["get"]["traj_state"]
        if arm0_traj_tag == 0 and arm1_traj_tag == 0:
            break
        time.sleep(0.001)
    print(f"Planning trajectories execution completed.")

    print(f"\n### 9/10. Switching to IDLE...")
    ret = robot.switch_to_idle(ctrl_obj1, 2000)
    if ret != 0:
        print(f"Arm0 switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    ret = robot.switch_to_idle(ctrl_obj2, 2000)
    if ret != 0:
        print(f"Arm1 switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 10/10. Task finished.")


if __name__ == "__main__":
    main()
