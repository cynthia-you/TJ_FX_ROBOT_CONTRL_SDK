"""
@file example_basic_PD.py
@brief Example demonstrating how to control the L1 robot arm
       in PD (Position-Direct) control mode (STATE_PD).

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Switch ARM0 into PD control mode(Check and recover the arm state (ERROR -> IDLE))
    4. Execute a predefined 961-point joint-space trajectory
    5. Collect command, feedback, and velocity data during execution
    6. Save sampled data to a text file
    7. Switch ARM0 to IDLE state

In STATE_PD, joint position commands are sent cyclically at a configurable
rate without interpolation. This mode is suitable for direct joint control
applications requiring low-latency command delivery.

@warning Ensure the robot arm is in a safe position and workspace
         before starting PD control.

@warning Press Enter key at any time to stop the robot and exit.
"""

import os
import re
import sys
import time
import threading
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map


TRAJ_POINT_NUM = 961


def load_trajectory():
    """Parse the joint trajectory array from C_EXAMPLE/example_basic_PD.cpp.

    @return: List of 7-element position lists, or None on failure.
    """
    cpp_path = os.path.join(os.path.dirname(__file__), '..', 'C_EXAMPLE', 'example_basic_PD.cpp')
    if not os.path.exists(cpp_path):
        print(f"Warning: trajectory source not found at {cpp_path}")
        return None

    with open(cpp_path, 'r') as f:
        content = f.read()

    match = re.search(r'double pos_array\[TRAJ_POINT_NUM\]\[7\]\s*=\s*\{([^}]+(?:{[^}]*}[^}]*)*)\};', content)
    if not match:
        print("Warning: could not parse trajectory array from C++ source")
        return None

    rows = re.findall(r'\{([^}]+)\}', match.group(1))
    traj = []
    for row in rows:
        values = [float(x.strip()) for x in row.split(',') if x.strip()]
        if len(values) == 6:
            values.append(0.0)
        if len(values) == 7:
            traj.append(values)

    if len(traj) != TRAJ_POINT_NUM:
        print(f"Warning: expected {TRAJ_POINT_NUM} trajectory points, got {len(traj)}")
        return None

    return traj


def write_sample_file(sample_data):
    """Write sampled PD data to PD_sample.txt.

    Each row: 7 cmd positions, 7 fb positions, 7 fb velocities (21 floats).

    @param sample_data: List of tuples (cmd_pos, fb_pos, fb_vel), each 7 floats.
    """
    filepath = os.path.join(os.path.dirname(__file__) or '.', 'PD_sample.txt')
    with open(filepath, 'w') as fp:
        for cmd, fb_pos, fb_vel in sample_data:
            row = list(cmd) + list(fb_pos) + list(fb_vel)
            fp.write(' '.join(f'{v:.15f}' for v in row) + '\n')
    print(f"Sample data saved to {filepath}")


def main():
    def emergency_stop_thread(robot):
        input()
        print("\n[Emergency stop] Triggered. Stopping robot...")
        robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
        time.sleep(0.1)
        print("Exiting program.")
        sys.exit(0)

    # --- Load trajectory ---
    pos_array = load_trajectory()
    if pos_array is None:
        print("Cannot proceed without trajectory data.")
        return

    ctrl_obj = FXObjType.OBJ_ARM0
    target_state = "PD"
    arm_idx = 0
    if ctrl_obj == FXObjType.OBJ_ARM1:
        arm_idx = 1
    log_mask = FXLogMask.FX_LOG_INFO_FLAG

    k = [3.0, 3.0, 3.0, 2.0, 1.0, 1.0, 1.0]
    d = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    vel_ratio = 100
    acc_ratio = 100
    pd_cycle_time = 0
    mechanical_home = [0.0] * 7

    robot = GentoRobot()
    threading.Thread(target=emergency_stop_thread, args=(robot,), daemon=True).start()

    print(f"\n### 1/7. Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/7. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/7. PD state switching...")
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

    ret = robot.config_set_pd_cmd_cycle_time(pd_cycle_time)
    if ret != 0:
        print(f"Set PD command cycle time failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    ret = robot.switch_to_pd_mode(ctrl_obj, 2000, vel_ratio, acc_ratio, k, d)
    if ret != 0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f"Already in {target_state} state.")
    sg_dict = robot.get_sg_dict()
    print(f"current vel:{sg_dict['arms'][arm_idx]['set']['vel_ratio']}")
    print(f"current acc:{sg_dict['arms'][arm_idx]['set']['acc_ratio']}")
    print(f"current k:{sg_dict['arms'][arm_idx]['set']['joint_k']}")
    print(f"current d:{sg_dict['arms'][arm_idx]['set']['joint_d']}")

    # --- Move to mechanical home ---
    print(f"\n### 4/7. Moving to mechanical home...")
    ret = robot.runtime_set_joint_pos_pd_cmd(ctrl_obj, mechanical_home)
    if ret != 0:
        print(f"Set joint PD command failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    time.sleep(0.5)
    rt_dict = robot.get_rt_dict()
    print(f"Arm0 home position:{rt_dict['arms'][arm_idx]['fb']['fb_pos']}")

    # --- Execute trajectory ---
    print(f"\n### 5/7. Executing trajectory ({TRAJ_POINT_NUM} points)...")
    sample_data = []
    stats_quality_sum = 0
    stats_quality_worst = 100

    t_start = time.time()
    for i, pos in enumerate(pos_array):
        robot.runtime_set_joint_pos_pd_cmd(ctrl_obj, pos)

        rt_dict = robot.get_rt_dict()
        sg_dict = robot.get_sg_dict()

        comm_quality = sg_dict['arms'][arm_idx]['get']['pd_cmd_quality']
        if comm_quality < stats_quality_worst:
            stats_quality_worst = comm_quality
        stats_quality_sum += comm_quality

        cmd = tuple(pos)
        fb_pos = tuple(rt_dict['arms'][arm_idx]['fb']['fb_pos'])
        fb_vel = tuple(rt_dict['arms'][arm_idx]['fb']['fb_vel'])
        sample_data.append((cmd, fb_pos, fb_vel))

        time.sleep(0.01)

    elapsed = time.time() - t_start
    stats_quality_avg = stats_quality_sum // TRAJ_POINT_NUM

    print(f"Trajectory finished ({TRAJ_POINT_NUM} points in {elapsed:.1f}s).")
    print(f"Communication quality: avg={stats_quality_avg}, worst={stats_quality_worst}")

    # --- Save data ---
    print(f"\n### 6/7. Saving sampled data...")
    write_sample_file(sample_data)

    # --- Return to IDLE ---
    print(f"\n### 7/7. Switching to IDLE...")
    ret = robot.switch_to_idle(ctrl_obj, 2000)
    if ret != 0:
        print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\nTask finished.")


if __name__ == "__main__":
    main()
