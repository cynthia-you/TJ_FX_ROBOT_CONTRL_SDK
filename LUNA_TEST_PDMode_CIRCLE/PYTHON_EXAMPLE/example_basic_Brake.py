"""
@file example_basic_Brake.py
@brief Example demonstrating how to control the brakes of L1 robot arms.

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Verify the arm state and recover to IDLE if necessary
    4. Unlock all brakes of ARM0
    5. Lock all brakes of ARM0

@warning Releasing the brakes may cause the arm to drop due to gravity.
         Ensure the arm is held securely before unlocking the brakes.

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

    ctrl_obj=FXObjType.OBJ_ARM0
    axis_mask = 0x7F
    arm_idx=0
    if ctrl_obj==FXObjType.OBJ_ARM1:
        arm_idx=1
    log_mask=FXLogMask.FX_LOG_INFO_FLAG

    robot = GentoRobot()

    print(f"\n### 1/6 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/6. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/6. IDLE state switching...")
    arm0_state=robot.current_state(ctrl_obj)
    if state_map[arm0_state] == "Error":
        ret, system_errorcode = robot.reset_error(ctrl_obj, 2000)
        if ret != 0:
            print(f"Reset error failed. Error msg: {error_dict[system_errorcode]}")
            return
    if state_map[arm0_state] != "IDLE":
        ret = robot.switch_to_idle(ctrl_obj, 2000)
        if ret != 0:
            print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return
        rt_dict = robot.get_rt_dict()
        print(f"current state：{rt_dict["arms"][arm_idx]["state"]["cur"]}")

    print(f"\n### 4/6. Unlock all brakes...")
    input("Press Enter key to unlock all brakes of arm0, please hold the arm carefully or it will drop!")
    ret=robot.config_brake_unlock(ctrl_obj, axis_mask)
    if ret != 0:
        print(f"Unbrake failed, Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 5/6. Lock all brakes...")
    input("Press Enter key to lock all brakes of arm0")
    ret = robot.config_brake_lock(ctrl_obj, axis_mask)
    if ret != 0:
        print(f"Brake failed, Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 6/6. Task finished.")

if __name__ == "__main__":
    main()
