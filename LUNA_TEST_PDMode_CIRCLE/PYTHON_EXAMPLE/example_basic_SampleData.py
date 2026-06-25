"""
@file example_basic_SampleData.py
@brief Example demonstrating how to register and sample user-defined
       feedback data at 100 Hz using the L1 SDK.

This example shows how to:
- Connect to the robot controller
- Register user data sets for RT and SG frame serials
- Validate the registered data layout
- Start a background thread to sample user data at ~100 Hz
- Print the sampled data periodically

The workflow mirrors the C example example_basic_SampleData.cpp.

@warning Ensure the robot arm is in a safe position before running.
"""

import sys
import time
import threading
from pathlib import Path
import ctypes
from ctypes import c_int, Structure, create_string_buffer, cast, POINTER

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))

from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXUserDataType


class UserSampleData(Structure):
    _fields_ = [
        ("rt_serial", c_int),
        ("sg_serial", c_int),
    ]


def sample_thread_func(robot, exit_event, run_event):
    run_count = 0
    buf = create_string_buffer(ctypes.sizeof(UserSampleData))
    while not exit_event.is_set():
        if run_event.is_set():
            robot.fbk_get_user_data(buf)
            data = cast(buf, POINTER(UserSampleData)).contents
            run_count += 1
            if run_count % 100 == 0:
                print(f"UserSampleData: rt_serial = {data.rt_serial}, sg_serial = {data.sg_serial}")
        time.sleep(0.01)


def main():
    log_mask = FXLogMask.FX_LOG_INFO_FLAG

    robot = GentoRobot()

    print(f"\n### 1/5. Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/5. Getting versions...")
    print(f"Controller version: {robot.get_controller_version()}")
    print(f"SDK version: {robot.get_sdk_version()}")

    input("Press Enter to configure user sample data...")

    robot.fbk_reset_user_data_set()

    ret = robot.fbk_register_user_data_set(
        "ROBOT_RT.m_RT_FrameSerial", FXUserDataType.FX_int, 0, 1
    )
    if ret != 0:
        print(f"Failed to register RT frame serial. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    ret = robot.fbk_register_user_data_set(
        "ROBOT_SG.m_SG_FrameSerial", FXUserDataType.FX_int, 0, 1
    )
    if ret != 0:
        print(f"Failed to register SG frame serial. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    ret = robot.fbk_check_user_data_set(ctypes.sizeof(UserSampleData))
    if ret != 0:
        print(f"Invalid user data definition. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print("Config user sample data success.")

    exit_event = threading.Event()
    run_event = threading.Event()
    sampler = threading.Thread(target=sample_thread_func, args=(robot, exit_event, run_event), daemon=True)
    sampler.start()

    input("Press Enter to start sampling data at 100 Hz...")
    run_event.set()

    input("Press Enter to stop sampling data...")
    run_event.clear()

    exit_event.set()
    sampler.join(timeout=2.0)
    print("Sampling stopped.")


if __name__ == "__main__":
    main()
