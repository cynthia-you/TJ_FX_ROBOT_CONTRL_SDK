"""
@file example_basic_LinkRobot.py
@brief Example demonstrating how to establish and terminate communication
       with the L1 robot controller system.

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions

This is the most fundamental example and should be executed before
any other robot control examples.
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

    print(f"\n### 1/4 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/4 .Checking link state...")
    ret_link = robot.check_link_state()
    if ret_link == -1:
        print("Link is established, but no data arrived in 100ms,\n please check the cables and firewall ")
        return
    elif ret_link == 0:
        print(f"Link is not established")
        return

    print(f"\n### 3/4. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 4/4. Task finished.")
    robot.unlink()

if __name__ == "__main__":
    main()
    input("Press Enter to exit...")







