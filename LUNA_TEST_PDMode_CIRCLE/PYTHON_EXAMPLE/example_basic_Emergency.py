"""
@file example_basic_Emergency.py
@brief Example demonstrating how to trigger an emergency stop (E-Stop)
       on the L1 robot arm during motion.

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Switch the arm to IDLE and then POSITION mode(Check and recover the arm state (ERROR → IDLE))
    4. Command joint-space motion targets
    5. Trigger an emergency stop during motion
    6. Observe the resulting arm state after E-Stop

The emergency stop immediately halts motion and transitions the arm
into a protective state.

@warning warning An emergency stop may cause abrupt deceleration.
         Ensure the robot is in a safe configuration before testing.

@warning Press Enter key at any time to stop the robot and exit.
"""

import sys
import time
import threading
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask,FXObjMask, FXObjType,state_map,error_dict

def main():

    ctrl_obj=FXObjType.OBJ_ARM0
    emergency_stop_mask=FXObjMask.OBJ_ARM0_FLAG
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

    print(f"\n### 1/6 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/6. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/6. Position state switching...")
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

    ret=robot.switch_to_position_mode(ctrl_obj,2000,vel,acc)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f"Already in {target_state} state.")
    rt_dict = robot.get_rt_dict()
    print(f"current state：{rt_dict["arms"][arm_idx]["state"]["cur"]}")
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
    time.sleep(2)

    print(f"\n### 5/6. Emergency stop triggered while moving to pos2...")
    ret = robot.emergency_stop(emergency_stop_mask)
    if ret != emergency_stop_mask:
        print(f"Set Emergency stop failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    time.sleep(0.01)

    rt_dict = robot.get_rt_dict()
    fb_joints = rt_dict["arms"][arm_idx]["fb"]["fb_pos"]
    arm_err=rt_dict['arms'][arm_idx]['state']['err']
    if arm_err in error_dict :
        print(f"Arm0 emergency stop. Stop at {fb_joints}. Current error:{error_dict[arm_err]}.")
    else:
        print(f"Arm0 emergency stop failed. Current pos:{fb_joints}. Current error:{error_dict[arm_err]}.")
        return

    print(f"\n### 6/6. Task finished.")

if __name__ == "__main__":
    main()
