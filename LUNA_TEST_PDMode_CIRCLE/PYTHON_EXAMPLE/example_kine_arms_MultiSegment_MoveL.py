"""
@file example_kine_arms_MultiSegment_MoveL.py
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
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map,error_dict


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
    allow_range=5
    zsp_type=1
    zsp_params=[0, 0, -1, 0, 0, 0]
    pln_vel = 100
    pln_acc = 100
    pln_freq = 50
    arm0_multi_points = [[509.731, 233.614, 265.949, -169.144, 55.011, -146.752],
                         [509.731, 233.614, 65.949, -169.144, 55.011, -146.752],
                         [509.731, 33.614, 65.949, -169.144, 55.011, -146.752],
                         [509.731, 33.614, 265.949, -169.144, 55.011, -146.752]]
    arm1_multi_points = [[509.731, -233.614, 265.949, 169.144, 55.011, 146.752],
                         [509.731, -233.614, 65.949, 169.144, 55.011, 146.752],
                         [509.731, -33.614, 65.949, 169.144, 55.011, 146.752],
                         [509.731, -33.614, 265.949, 169.144, 55.011, 146.752]]
    arm0_start_pos = [17.970, -35.197, 11.414, -73.344, -9.154, -17.035, 7.086]
    arm1_start_pos = [-17.970, -35.197, -11.414, -73.344, 9.154, -17.035, -7.086]


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
        ret, system_errorcode = robot.reset_error(ctrl_obj1, 2000)
        if ret != 0:
            print(f"Arm0 reset error failed. Error msg: {error_dict[system_errorcode]}")
            return
    elif state_map[arm0_state] != "IDLE":
        ret = robot.switch_to_idle(ctrl_obj1, 2000)
        if ret != 0:
            print(f"Arm0 switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

    arm1_state = robot.current_state(ctrl_obj2)
    if state_map[arm1_state] == "Error":
        ret, system_errorcode = robot.reset_error(ctrl_obj2, 2000)
        if ret != 0:
            print(f"Arm1 reset failed: {error_dict[system_errorcode]}")
            return
    elif state_map[arm1_state] != "IDLE":
        ret = robot.switch_to_idle(ctrl_obj2, 2000)
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
    print(f"current state:{rt_dict["arms"][0]["state"]["cur"]}")
    print(f"current joints:{rt_dict["arms"][0]["fb"]["fb_pos"]}")
    print(f"current vel:{sg_dict["arms"][0]['set']["vel_ratio"]}")
    print(f"current acc:{sg_dict["arms"][0]['set']["acc_ratio"]}")
    print("arm1:")
    print(f"current state:{rt_dict["arms"][1]["state"]["cur"]}")
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

    print(f"\n### 5/10. Set the global speed and acceleration to the maximum to prevent limiting the planned speed and acceleration....")
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
    ret0 = robot.plan_linear_multi_points_set_start(0, arm0_start_pos, arm0_multi_points[0], arm0_multi_points[1],
                                                    allow_range, zsp_type, zsp_params,
                                                    pln_vel, pln_acc, pln_freq)
    if ret0 != 0:
        print(f"Arm0 planning set start failed. Error msg: {robot._get_operate_error_msg(ret0)}")
        return
    for next_one in arm0_multi_points[2:]:
        ret1 = robot.plan_linear_multi_points_set_next(0, next_one, allow_range, zsp_type, zsp_params,  pln_vel, pln_acc, )
        if ret1 != 0:
            print(f"Arm0 planning set next failed. Error msg: {robot._get_operate_error_msg(ret1)}")
            return
    point0 = robot.plan_linear_multi_points_get_points()
    if isinstance(point0, tuple):
        ret2 = robot.config_set_traj(FXObjType.OBJ_ARM0, point0[0], point0[1])
        if ret2 != 0:
            print(f"Arm0 send planning points failed. Error msg: {robot._get_operate_error_msg(ret2)}")
            return
    else:
        print(f"Arm0 get planning points failed. Error msg: {robot._get_operate_error_msg(point0)}")
        return


    ret01 = robot.plan_linear_multi_points_set_start(1, arm1_start_pos, arm1_multi_points[0], arm1_multi_points[1],
                                                    allow_range, zsp_type, zsp_params,
                                                    pln_vel, pln_acc, pln_freq)
    if ret01 != 0:
        print(f"Arm1 planning set start failed. Error msg: {robot._get_kinematics_error_msg(ret01)}")
        return
    for next_one1 in arm1_multi_points[2:]:
        ret11 = robot.plan_linear_multi_points_set_next(1, next_one1, allow_range, zsp_type, zsp_params, pln_vel, pln_acc)
        if ret11 != 0:
            print(f"Arm0 planning set next failed. Error msg: {robot._get_operate_error_msg(ret11)}")
            return
    point1 = robot.plan_linear_multi_points_get_points()
    if isinstance(point1, tuple):
        ret21 = robot.config_set_traj(FXObjType.OBJ_ARM1, point1[0], point1[1])
        if ret21 != 0:
            print(f"Arm1 send planning points failed. Error msg: {robot._get_operate_error_msg(ret21)}")
            return
    else:
        print(f"Arm1 get planning points failed. Error msg: {robot._get_operate_error_msg(point1)}")
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
