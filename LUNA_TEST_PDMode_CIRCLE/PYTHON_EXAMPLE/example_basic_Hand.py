"""
@file example_basic_Hand.py
@brief Basic example demonstrating how to control the left hand of L1 robot.

Workflow overview:
    1. SDK and controller version query
    2. Hand error handling and recovery
    3. Sending predefined hand gestures
"""

import sys
import time
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXHandType, FXHandState,FXHandAction,state_map,error_dict

def main():

    hand_type=FXHandType.FX_HAND_LEFT
    log_mask = FXLogMask.FX_LOG_INFO_FLAG
    gesture1=[30]*24
    gesture2=[60]*24
    open_palm=[0]*24

    robot = GentoRobot()

    print(f"\n### 1/5 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/5. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/5. Check left hand error state...")
    rt_dict = robot.get_rt_dict()
    hand_state=rt_dict["hands"][0]["fb"]["state"]
    if hand_state==FXHandState.FX_HAND_STATE_ERROR:
        ret = robot.runtime_set_hand_action(hand_type, FXHandAction.FX_HAND_ACTION_RESET)
        if ret != 0:
            print(f"Set hand reset failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return
        time.sleep(0.1)

        ret = robot.runtime_set_hand_action(hand_type, FXHandAction.FX_HAND_ACTION_ENABLE)
        if ret != 0:
            print(f"Set hand enable failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return
        time.sleep(0.1)

        rt_dict = robot.get_rt_dict()
        hand_state = rt_dict["hands"][0]["fb"]["state"]
        if hand_state == FXHandState.FX_HAND_STATE_ERROR:
            print("Left hand still in error!")
            return

    print(f"\n### 4/5. Hand action...")
    print("Left hand is ready to work.")
    input("Press Enter key to make a gesture1")
    ret = robot.runtime_set_hand_pos(hand_type, gesture1)
    if ret != 0:
        print(f"Set hand gesture1 failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    input("Press Enter key to make a gesture2")
    ret = robot.runtime_set_hand_pos(hand_type, gesture2)
    if ret != 0:
        print(f"Set hand gesture2 failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return


    input("Press Enter key to make an open palm")
    ret = robot.runtime_set_hand_pos(hand_type, open_palm)
    if ret != 0:
        print(f"Set hand open palm failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 5/5. Task finished.")

if __name__ == "__main__":
    main()
