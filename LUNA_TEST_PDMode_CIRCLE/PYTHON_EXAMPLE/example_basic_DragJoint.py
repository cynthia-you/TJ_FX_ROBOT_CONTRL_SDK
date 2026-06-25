"""
@file example_basic_DragJoint.cpp
@brief Example demonstrating how to switch the L1 robot arm into
       joint-space drag mode (STATE_DRAG_JOINT).

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Switch ARM0 into joint drag mode using stiffness and damping parameters(STATE_DRAG_JOINT)(Check and recover the arm state (ERROR → IDLE))
    4. Allow manual dragging via the arm terminal button
    5. Switch ARM to IDLE state

In STATE_DRAG_JOINT, each joint can be physically guided by hand,
and the arm behaves according to per-joint impedance control.

@warning Ensure the robot arm is in a safe position and workspace
         before entering drag mode to avoid collisions or unexpected motion.

@warning Press Enter key at any time to stop the robot and exit.
"""

import sys
import time
import threading
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask,FXObjMask, FXObjType,state_map

def main():

    ctrl_obj=FXObjType.OBJ_ARM0
    target_state="DragJoint"
    arm_idx=0
    if ctrl_obj==FXObjType.OBJ_ARM1:
        arm_idx=1
    log_mask=FXLogMask.FX_LOG_INFO_FLAG
    k=[3, 3, 3, 2, 1, 1, 1]
    d=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

    robot = GentoRobot()

    print(f"\n### 1/6 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/6. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/6. Drag joint state switching...")
    arm0_state = robot.current_state(ctrl_obj)
    if state_map[arm0_state] == "Error":
        ret, system_errorcode = robot.reset_error(ctrl_obj, 2000)
        if ret != 0:
            print(f"Reset error failed. Error msg: {system_errorcode}")
            return
    elif state_map[arm0_state] != "IDLE":
        ret = robot.switch_to_idle(ctrl_obj, 2000)
        if ret != 0:
            print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

    ret=robot.switch_to_drag_joint(ctrl_obj,2000,k,d)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f"Already in {target_state} state.")
    rt_dict = robot.get_rt_dict()
    sg_dict = robot.get_sg_dict()
    print(f"current state：{rt_dict["arms"][arm_idx]["state"]["cur"]}")
    print(f"current drag type：{rt_dict["arms"][arm_idx]["cmd"]["drag_type"]}")
    print(f"current k:{sg_dict["arms"][arm_idx]['set']["joint_k"]}")
    print(f"current d:{sg_dict["arms"][arm_idx]['set']["joint_d"]}")
    print(f"current joints:{rt_dict["arms"][arm_idx]["fb"]["fb_pos"]}")


    print(f"\n### 4/6. Arm0 is in STATE_DRAG_JOINT state now, please press the drag button on Arm0's terminal to drag the arm...")
    input("If drag finished, press Enter to IDLE...")

    print(f"\n### 5/6. Switching to IDLE...")
    ret = robot.switch_to_idle(ctrl_obj, 2000)
    if ret != 0:
        print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 6/6. Task finished.")

if __name__ == "__main__":
    main()
