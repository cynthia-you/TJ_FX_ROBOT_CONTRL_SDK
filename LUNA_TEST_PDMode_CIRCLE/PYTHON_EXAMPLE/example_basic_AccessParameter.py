"""
@file example_basic_AccessParameter.py
@brief Example demonstrating how to access and modify basic L1 robot parameters.

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Read basic robot parameters
    4. Modify and verify an encoder error threshold parameter
    5. Restore the original parameter value

@note This example is intended for users who need to understand the basic
       parameter read/write interfaces of the L1 robot SDK.
"""

import sys
import time
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXObjType,state_map

def main():

    log_mask=FXLogMask.FX_LOG_INFO_FLAG

    robot = GentoRobot()

    print(f"\n### 1/9 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/9. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/9. Read robot name...")
    ret, value=robot.param_get_string("R.BASIC.Name")
    if ret!=0:
        print(f"Get parameter failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f"R.BASIC.Name is: {value}")

    print(f"\n### 4/9. Read bus frequency...")
    ret,value=robot.param_get_int("R.BASIC.BusFreq")
    if ret!=0:
        print(f"Get parameter failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f"R.BASIC.BusFreq is: {value}")

    print(f"\n### 5/9. Read encoder error threshold...")
    ret,cur_value=robot.param_get_float("R.A0.BASIC.EncErrorValve")
    if ret!=0:
        print(f"Get parameter failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f"R.A0.BASIC.EncErrorValve is: {cur_value}")

    print(f"\n### 6/9. Modify encoder error threshold...")
    ret=robot.param_set_float("R.A0.BASIC.EncErrorValve", 2.5)
    if ret!=0:
        print(f"Set parameter failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f'set "R.A0.BASIC.EncErrorValve" to 2.5')

    print(f"\n### 7/9. Verify modified parameter...")
    ret,value=robot.param_get_float("R.A0.BASIC.EncErrorValve")
    if ret!=0:
        print(f"Get parameter failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f"R.A0.BASIC.EncErrorValve is: {value}")

    print(f"\n### 8/9. Restore original parameter value...")
    ret=robot.param_set_float("R.A0.BASIC.EncErrorValve",cur_value)
    if ret!=0:
        print(f"Set parameter failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f'set "R.A0.BASIC.EncErrorValve" to {cur_value}')

    print(f"\n### 9/9. Task finished.")

if __name__ == "__main__":
    main()
