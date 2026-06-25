"""
@file example_basic_Position.py
@brief Example demonstrating how to control the L1 robot arm
       in joint-space position control mode (STATE_POSITION).

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Switch ARM0 into position control mode (STATE_POSITION)(Check and recover the arm state (ERROR → IDLE))
    4. Move ARM0 to two different joint target positions
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
        print("\n[Emergency stop] Triggered. Stopping robot...")
        robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
        time.sleep(0.1)
        print("Exiting program.")
        sys.exit(0)

    ctrl_obj=FXObjType.OBJ_ARM0
    target_state="Position"
    arm_idx=0
    if ctrl_obj==FXObjType.OBJ_ARM1:
        arm_idx=1
    log_mask=FXLogMask.FX_LOG_INFO_FLAG
    vel=10
    acc=10
    pos1=[0]*7
    pos2=[10,10,10,-90,10,10,10]

    robot = GentoRobot()
    threading.Thread(target=emergency_stop_thread, args=(robot,), daemon=True).start()

    print(f"\n### 1/6 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/6. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/6. Position state switching...")
    arm0_state=robot.current_state(ctrl_obj)
    if state_map[arm0_state] == "Error":
        ret, system_errorcode = robot.reset_error(ctrl_obj, 1000)
        if ret != 0:
            print(f"Reset error failed. Error msg: {error_dict[system_errorcode]}")
            return
    elif state_map[arm0_state] != "IDLE":
        ret = robot.switch_to_idle(ctrl_obj, 1000)
        if ret != 0:
            print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

    ret=robot.switch_to_position_mode(ctrl_obj,2000,vel,acc)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f"Already in {target_state} state.")
    rt_dict = robot.get_rt_dict()
    print(f"current state:{rt_dict["arms"][arm_idx]["state"]["cur"]}")
    print(f"current joints:{rt_dict["arms"][arm_idx]["fb"]["fb_pos"]}")
    sg_dict = robot.get_sg_dict()
    print(f"current vel:{sg_dict["arms"][arm_idx]['set']["vel_ratio"]}")
    print(f"current acc:{sg_dict["arms"][arm_idx]['set']["acc_ratio"]}")

    print(f"\n### 4/6. Run pos1 and pos2...")
    ret = robot.runtime_set_joint_pos_cmd(ctrl_obj, pos1)
    if ret != 0:
        print(f"Set joint command failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    while 1:
        rt_dict = robot.get_rt_dict()
        fb_joints = rt_dict["arms"][arm_idx]["fb"]["fb_pos"]
        if robot.check_sequences_approx_equal(fb_joints, pos1):
            break
        time.sleep(0.001)
    print(f"Arm0 reached at {fb_joints}")

    time.sleep(0.5)

    ret = robot.runtime_set_joint_pos_cmd(ctrl_obj, pos2)
    if ret != 0:
        print(f"Set joint command failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    while 1:
        rt_dict = robot.get_rt_dict()
        fb_joints = rt_dict["arms"][arm_idx]["fb"]["fb_pos"]
        if robot.check_sequences_approx_equal(fb_joints, pos2):
            break
        time.sleep(0.001)
    print(f"Arm0 reached at {fb_joints}")

    print(f"\n### 5/6. Switching to IDLE...")
    ret = robot.switch_to_idle(ctrl_obj, 2000)
    if ret != 0:
        print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 6/6. Task finished.")

if __name__ == "__main__":
    main()
