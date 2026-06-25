"""
@file example_basic_ResetError.py
@brief Example demonstrating how to detect and reset errors on the L1 robot arm.

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Detect whether the robot arm is in an ERROR state
    4. Optionally trigger an emergency stop to enter the ERROR state
    5. Read system-level and servo-level error codes
    6. Reset the arm error and return to IDLE state

Error handling is a critical part of safe robot operation and should be
well understood before deploying user applications.
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
    emergency_stop_mask = FXObjMask.OBJ_ARM0_FLAG
    target_state="Error"
    arm_idx=0
    if ctrl_obj==FXObjType.OBJ_ARM1:
        arm_idx=1
    log_mask=FXLogMask.FX_LOG_INFO_FLAG

    robot = GentoRobot()
    threading.Thread(target=emergency_stop_thread, args=(robot,), daemon=True).start()

    print(f"\n### 1/5 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/5. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/5. Error state switching...")
    arm0_state=robot.current_state(ctrl_obj)
    if state_map[arm0_state] == "Error":
        print(f"Arm0 is in error state now")
        ret, system_errorcode = robot.reset_error(ctrl_obj, 2000)
        if ret != 0:
            print(f"Reset error failed. Error msg: {error_dict[system_errorcode]}")
            return
        else:
            print(f"Arm0 reset error.")
            return
    else:
        print(f"Trigger an emergency error ")
        ret = robot.emergency_stop(emergency_stop_mask)
        if ret != emergency_stop_mask:
            print(f"Set Emergency stop failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return
        time.sleep(0.01)

    print(f"\n### 4/5. Read and print system error code...")
    rt_dict = robot.get_rt_dict()
    fb_joints = rt_dict["arms"][arm_idx]["fb"]["fb_pos"]
    arm_err = rt_dict['arms'][arm_idx]['state']['err']
    if arm_err in error_dict:
        print(f"Arm0 emergency stop. Current error:{error_dict[arm_err]}. Stop at {fb_joints}.")
    else:
        print(f"Arm0 emergency stop failed. Current error:{arm_err}. Current pos:{fb_joints}. ")
        return
    ret, msg = robot.get_servo_error_codes(ctrl_obj)
    if ret != 0:
        print(f"Get servo error failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    else:
        print(f"Arm0 servo error msg:\n{msg}")

    print(f"\n### 5/5. Task finished.")

if __name__ == "__main__":
    main()
