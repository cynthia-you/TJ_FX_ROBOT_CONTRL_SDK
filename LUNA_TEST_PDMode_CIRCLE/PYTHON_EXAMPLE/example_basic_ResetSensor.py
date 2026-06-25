"""
@file example_basic_ResetSensor.py
@brief Example demonstrating how to reset joint torque sensor offsets
       for the L1 robot arm.

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Run to zero joints in position state(Check and recover the arm state (ERROR → IDLE))
    4. Read current joint torque sensor feedback
    5. Reset torque sensor offsets to zero

Resetting torque sensor offsets is essential for accurate torque control
and collision detection.

@warning Before running this example, ensure that **no tool is mounted**
         on Arm0’s end effector. Otherwise, torque control behavior may
         become incorrect.

@warning Press Enter key at any time to stop the robot and exit.
"""

import sys
import time
import threading
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXObjMask, FXObjType,state_map,error_dict

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
    axis_mask = 0x7F
    arm_idx=0
    if ctrl_obj==FXObjType.OBJ_ARM1:
        arm_idx=1
    log_mask=FXLogMask.FX_LOG_INFO_FLAG
    vel=10
    acc=10
    pos1=[0]*7

    robot = GentoRobot()
    threading.Thread(target=emergency_stop_thread, args=(robot,), daemon=True).start()

    print(f"\n### 1/8 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/8. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/8. Position state switching...")
    arm0_state = robot.current_state(ctrl_obj)
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

    if state_map[arm0_state] != target_state:
        ret=robot.switch_to_position_mode(ctrl_obj,2000,vel,acc)
        if ret !=0:
            print(f"Error msg: {robot._get_operate_error_msg(ret)}")
            return

        rt_dict = robot.get_rt_dict()
        print(f"current state：{rt_dict["arms"][arm_idx]["state"]["cur"]}")
        print(f"current joints:{rt_dict["arms"][arm_idx]["fb"]["fb_pos"]}")
        sg_dict = robot.get_sg_dict()
        print(f"current vel:{sg_dict["arms"][arm_idx]['set']["vel_ratio"]}")
        print(f"current acc:{sg_dict["arms"][arm_idx]['set']["acc_ratio"]}")

    else:
        print(f"Already in {target_state} state, check vel and acc")
        sg_dict = robot.get_sg_dict()
        print(f"current vel:{sg_dict["arms"][arm_idx]['set']["vel_ratio"]}")
        print(f"current acc:{sg_dict["arms"][arm_idx]['set']["acc_ratio"]}")

        if int(sg_dict["arms"][arm_idx]['set']["vel_ratio"])!=vel:
            print(f"Set vel to {vel}")
            ret=robot.runtime_set_vel_ratio(ctrl_obj,vel)
            if ret!=0:
                print(f"set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            time.sleep(0.01)
            sg_dict = robot.get_sg_dict()
            print(f"After set, current vel:{sg_dict["arms"][0]['set']["vel_ratio"]}")

        if int(sg_dict["arms"][0]['set']["acc_ratio"])!=acc:
            print(f"Set acc to {acc}")
            ret=robot.runtime_set_acc_ratio(ctrl_obj,acc)
            if ret!=0:
                print(f"set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            time.sleep(0.01)
            sg_dict = robot.get_sg_dict()
            print(f"After set, current acc:{sg_dict["arms"][0]['set']["acc_ratio"]}")

    print(f"\n### 4/8. Run zero joints...")
    ret = robot.runtime_set_joint_pos_cmd(ctrl_obj, pos1)
    if ret != 0:
        print(f"Error msg: {robot._get_operate_error_msg(ret)}")
        return
    while 1:
        rt_dict = robot.get_rt_dict()
        fb_joints = rt_dict["arms"][arm_idx]["fb"]["fb_pos"]
        if robot.check_sequences_approx_equal(fb_joints, pos1):
            break
        time.sleep(0.001)
    print(f"Arm0 reached at {fb_joints}")
    time.sleep(0.5)

    print(f"\n### 5/8. Switching to IDLE...")
    ret = robot.switch_to_idle(ctrl_obj, 2000)
    if ret != 0:
        print(f"Error msg: {robot._get_operate_error_msg(ret)}")
        return
    arm0_state = robot.current_state(ctrl_obj)
    if state_map[arm0_state]!="IDLE":
        print(f"switch to idle faile")
        return
    print(f"Arm0 in idle state: {state_map[arm0_state]}")

    print(f"\n### 6/8. Reset sensor torque...")
    rt_dict = robot.get_rt_dict()
    fb_sensor = rt_dict["arms"][arm_idx]["fb"]["fb_sensor"]
    print(f"current sensor torque:{fb_sensor}")
    ret = robot.config_clear_sensor_offset(ctrl_obj)
    if ret != 0:
        print(f"Error msg: {robot._get_operate_error_msg(ret)}")
        return
    time.sleep(1)

    print(f"\n### 7/8. Read sensor torque...")
    rt_dict = robot.get_rt_dict()
    fb_sensor = rt_dict["arms"][arm_idx]["fb"]["fb_sensor"]
    print(f"After reset, current sensor torque:{fb_sensor}")

    print(f"\n### 8/8. Task finished.")

if __name__ == "__main__":
    main()
