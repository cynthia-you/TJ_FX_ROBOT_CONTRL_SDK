"""
@file example_basic_DragCartX.cpp
@brief Example demonstrating how to switch the L1 robot arm into
       Cartesian-space drag mode (STATE_DRAG_CART).

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Position state run to drag pose
    3. Switch to cartesian drag mode using stiffness and damping parameters(STATE_DRAG_JOINT)(Check and recover the arm state (ERROR → IDLE))
    4. Allow manual dragging via the arm terminal button
    5. Switch ARM to IDLE state

In STATE_DRAG_CART, the arm can be physically guided by hand
while respecting Cartesian impedance behavior.

@warning Ensure the robot arm is in a safe position and workspace
         before entering drag mode to avoid collisions or unexpected motion.

"""

import sys
import time
import threading
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXObjMask, FXObjType,state_map,drag_type_map,error_dict

def main():

    ctrl_obj=FXObjType.OBJ_ARM0
    target_state="DragCartX"
    arm_idx=0
    if ctrl_obj==FXObjType.OBJ_ARM1:
        arm_idx=1
    log_mask=FXLogMask.FX_LOG_INFO_FLAG
    k=[3000,3000,3000,100,100,100,50]
    d=[0.1,0.1,0.1,0.1,0.1,0.1,0.11]
    vel=10
    acc=10
    pos1=[0]*7
    ref_drag_pos=[75.093, -87.92, -95.201, -55.945, 5.712, -26.761, -6.909]

    robot = GentoRobot()

    print(f"\n### 1/7 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/7. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/7. Position state run to drag pos...")
    arm0_state = robot.current_state(ctrl_obj)
    if state_map[arm0_state] == "Error":
        ret, system_errorcode = robot.reset_error(ctrl_obj, 2000)
        if ret != 0:
            print(f"Reset error failed. Error msg: {error_dict[system_errorcode]}")
            return
    elif state_map[arm0_state] != "IDLE":
        ret = robot.switch_to_idle(ctrl_obj, 2000)
        if ret != 0:
            print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

    ret = robot.switch_to_position_mode(ctrl_obj, 2000, vel, acc)
    if ret != 0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
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

    ret = robot.runtime_set_joint_pos_cmd(ctrl_obj, ref_drag_pos)
    if ret != 0:
        print(f"Set joint command failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    while 1:
        rt_dict = robot.get_rt_dict()
        fb_joints = rt_dict["arms"][arm_idx]["fb"]["fb_pos"]
        if robot.check_sequences_approx_equal(fb_joints, ref_drag_pos):
            break
        time.sleep(0.001)
    print(f"Arm0 reached at {fb_joints}")

    ret = robot.switch_to_idle(ctrl_obj, 2000)
    if ret != 0:
        print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 4/7. Drag cartesian X direction state switching...")
    ret=robot.switch_to_drag_cart_x(ctrl_obj,2000,k,d)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f"Already in {target_state} state.")
    rt_dict = robot.get_rt_dict()
    sg_dict = robot.get_sg_dict()
    print(f"current state：{rt_dict["arms"][arm_idx]["state"]["cur"]}")
    print(f"current drag type：{rt_dict["arms"][arm_idx]["cmd"]["drag_type"]}")
    print(f"current k:{sg_dict["arms"][arm_idx]['set']["cart_k"]}")
    print(f"current d:{sg_dict["arms"][arm_idx]['set']["cart_d"]}")
    print(f"current joints:{rt_dict["arms"][arm_idx]["fb"]["fb_pos"]}")

    print(f"\n### 5/7. Arm0 is in {target_state} state now, please press the drag button on Arm0's terminal to drag the arm...")
    input("If drag finished, press Enter to IDLE...")

    print(f"\n### 6/7. Switching to IDLE...")
    ret = robot.switch_to_idle(ctrl_obj, 2000)
    if ret != 0:
        print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 7/7. Task finished.")

if __name__ == "__main__":
    main()
