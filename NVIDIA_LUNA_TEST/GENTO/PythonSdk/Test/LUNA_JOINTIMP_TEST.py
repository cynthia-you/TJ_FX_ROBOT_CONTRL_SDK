import atexit
import ctypes
import math
import os
import sys
import time
import traceback


current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from GentoRobot import GentoRobot, FXObjType

from luna_feedback_tools import (
    append_luna_feedback_sample,
    finalize_luna_feedback_collector,
    safe_float,
    safe_get,
    safe_vec,
    start_luna_feedback_collector,
)
from luna_plot_tools import print_luna_output_summary, save_and_plot_luna_result


# ==================== Config ====================
TEST_NAME = "LUNA_JOINTIMP_TEST"

ROBOT_IP = (6, 6, 7, 190)
LOG_SWITCH = 0
ALL_OBJ_MASK = 0x1F

INIT_VEL_RATIO = 10
INIT_ACC_RATIO = 10
TEST_VEL_RATIO = 100
TEST_ACC_RATIO = 100

# Joint impedance K/D settings.
# Keep values conservative by default; tune per hardware condition when needed.
ARM_JOINT_K = [12.0, 12.0, 12.0, 11.0, 11.0, 11.0, 10.5] 
ARM_JOINT_D = [0.1] * 7
BODY_JOINT_K = [15.0, 15.0, 15.0, 15.0, 15.0, 15.0]
BODY_JOINT_D = [3.0, 3.0, 3.0, 2.0, 1.0, 1.0]
# switch_to_imp_joint_mode currently expects 7-dim K/D buffers in SDK wrapper.
BODY_JOINT_K_FOR_SWITCH = BODY_JOINT_K + [0.0]
BODY_JOINT_D_FOR_SWITCH = BODY_JOINT_D + [0.0]

DT = 0.01
TIME_PER_JOINT = 10.0
SIN_FREQ_HZ = 0.5

POLL_PERIOD_S = 0.005
STALE_THRESHOLD_S = 0.100
MOVE_TO_DEFAULT_WAIT_S = 3.0
RESTORE_BETWEEN_JOINT_S = 0.8

# Per-joint amplitudes in degrees.
# Arm amplitudes apply to both left_arm and right_arm.
ARM_J1_AMP_DEG = 30.0
ARM_J2_AMP_DEG = 10.0
ARM_J3_AMP_DEG = 10.0
ARM_J4_AMP_DEG = 10.0
ARM_J5_AMP_DEG = 10.0
ARM_J6_AMP_DEG = 10.0
ARM_J7_AMP_DEG = 30.0

HEAD_J1_AMP_DEG = 10.0
HEAD_J2_AMP_DEG = 10.0

BODY_J1_AMP_DEG = 5.0
BODY_J2_AMP_DEG = 5.0
BODY_J3_AMP_DEG = 5.0
BODY_J4_AMP_DEG = 5.0
BODY_J5_AMP_DEG = 5.0
BODY_J6_AMP_DEG = 5.0

ARM_JOINT_AMP_DEG = [
    ARM_J1_AMP_DEG,
    ARM_J2_AMP_DEG,
    ARM_J3_AMP_DEG,
    ARM_J4_AMP_DEG,
    ARM_J5_AMP_DEG,
    ARM_J6_AMP_DEG,
    ARM_J7_AMP_DEG,
]
HEAD_JOINT_AMP_DEG = [
    HEAD_J1_AMP_DEG,
    HEAD_J2_AMP_DEG,
]
BODY_JOINT_AMP_DEG = [
    BODY_J1_AMP_DEG,
    BODY_J2_AMP_DEG,
    BODY_J3_AMP_DEG,
    BODY_J4_AMP_DEG,
    BODY_J5_AMP_DEG,
    BODY_J6_AMP_DEG,
]

GROUP_TESTS = [
    {
        "group_name": "left_arm",
        "group_type": "arm",
        "group_index": 0,
        "joint_count": 7,
        "amp_deg": ARM_JOINT_AMP_DEG,
    },
    {
        "group_name": "right_arm",
        "group_type": "arm",
        "group_index": 1,
        "joint_count": 7,
        "amp_deg": ARM_JOINT_AMP_DEG,
    },
    {
        "group_name": "head",
        "group_type": "head",
        "group_index": None,
        "joint_count": 2,
        "amp_deg": HEAD_JOINT_AMP_DEG,
    },
    {
        "group_name": "body",
        "group_type": "body",
        "group_index": None,
        "joint_count": 6,
        "amp_deg": BODY_JOINT_AMP_DEG,
    },
]

LEFT_DEFAULT_POS = [90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0]
RIGHT_DEFAULT_POS = [90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0]

RESULT_ROOT_DIR = os.path.join(current_dir, "results_luna_jointimp_test")


def sleep_until(deadline, spin_threshold=0.0005):
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > spin_threshold:
            time.sleep(remaining - spin_threshold)
        else:
            while time.perf_counter() < deadline:
                pass
            return


winmm = None
high_res_timer_enabled = False
if os.name == "nt":
    try:
        winmm = ctypes.WinDLL("winmm")
        if winmm.timeBeginPeriod(1) == 0:
            high_res_timer_enabled = True
            print("enabled Windows high-resolution timer: 1ms")
        else:
            print("warning: failed to enable Windows high-resolution timer")
    except Exception as exc:
        print(f"warning: failed to load winmm timer APIs: {exc}")


def _release_high_res_timer():
    global high_res_timer_enabled
    if high_res_timer_enabled and winmm is not None:
        try:
            winmm.timeEndPeriod(1)
        finally:
            high_res_timer_enabled = False


atexit.register(_release_high_res_timer)


def check_rt_frame_refresh(robot, try_count=5, interval_s=0.1):
    motion_tag = 0
    frame_last = None
    for _ in range(try_count):
        rt = robot.get_rt_dict()
        frame_serial = safe_get(rt, ["frame_serial"], 0)
        print(f"connect frames: {frame_serial}")
        if frame_serial != 0 and frame_serial != frame_last:
            motion_tag += 1
            frame_last = frame_serial
        time.sleep(interval_s)
    return motion_tag > 0


def send_all_pos_cmds(robot, arm0_pos, arm1_pos, head_pos, body_pos, stage):
    if not robot.comm_clear(50):
        raise RuntimeError(f"comm_clear failed ({stage})")
    if not robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_ARM0, arm0_pos):
        raise RuntimeError(f"runtime_set_joint_pos_cmd failed for ARM0 ({stage})")
    if not robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_ARM1, arm1_pos):
        raise RuntimeError(f"runtime_set_joint_pos_cmd failed for ARM1 ({stage})")
    if not robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_HEAD, head_pos):
        raise RuntimeError(f"runtime_set_joint_pos_cmd failed for HEAD ({stage})")
    if not robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_BODY, body_pos):
        raise RuntimeError(f"runtime_set_joint_pos_cmd failed for BODY ({stage})")
    if not robot.comm_send():
        raise RuntimeError(f"comm_send failed ({stage})")


def main():
    robot = None
    collector = None

    head_default_pos = [0.0, 0.0]
    head_default_cmd_pos = [0.0, 0.0, 0.0]
    body_default_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    try:
        robot = GentoRobot()

        version = robot.link(*ROBOT_IP, log_switch=LOG_SWITCH)
        if version <= 0:
            raise RuntimeError(f"failed to connect to robot, error code: {version}")
        print(f"robot connected, SDK version: {version}")

        cleared_mask = robot.reset_error(ALL_OBJ_MASK)
        print(f"reset_error mask: 0x{cleared_mask:02X}")

        if not check_rt_frame_refresh(robot, try_count=5, interval_s=0.1):
            raise RuntimeError("failed: robot connection failed (RT frame not refreshing)")
        print("success: robot connected")

        ret_a = robot.switch_to_position_mode(FXObjType.OBJ_ARM0, 1000, INIT_VEL_RATIO, INIT_ACC_RATIO)
        ret_b = robot.switch_to_position_mode(FXObjType.OBJ_ARM1, 1000, INIT_VEL_RATIO, INIT_ACC_RATIO)
        ret_h = robot.switch_to_position_mode(FXObjType.OBJ_HEAD, 1000, INIT_VEL_RATIO, INIT_ACC_RATIO)
        ret_body = robot.switch_to_position_mode(FXObjType.OBJ_BODY, 1000, INIT_VEL_RATIO, INIT_ACC_RATIO)
        if ret_a != 0 or ret_b != 0 or ret_h != 0 or ret_body != 0:
            raise RuntimeError(
                f"switch_to_position_mode failed, ret_a={ret_a}, ret_b={ret_b}, "
                f"ret_h={ret_h}, ret_body={ret_body}"
            )

        rt_init = robot.get_rt_dict()
        # SDK runtime_set_joint_pos_cmd for HEAD still expects 3 values.
        # Keep a 3-axis command vector for sending, while test/metrics only use J1-J2.
        head_default_cmd_pos = safe_vec(safe_get(rt_init, ["head", "fb_pos"], [0.0, 0.0, 0.0]), 3)
        head_default_pos = head_default_cmd_pos[:2]
        body_default_pos = safe_vec(safe_get(rt_init, ["body", "fb_pos"], [0.0] * 6), 6)

        send_all_pos_cmds(
            robot,
            LEFT_DEFAULT_POS,
            RIGHT_DEFAULT_POS,
            head_default_cmd_pos,
            body_default_pos,
            stage="before sending default poses",
        )
        time.sleep(MOVE_TO_DEFAULT_WAIT_S)

        ret_a = robot.switch_to_imp_joint_mode(
            FXObjType.OBJ_ARM0,
            1000,
            TEST_VEL_RATIO,
            TEST_ACC_RATIO,
            ARM_JOINT_K,
            ARM_JOINT_D,
        )
        ret_b = robot.switch_to_imp_joint_mode(
            FXObjType.OBJ_ARM1,
            1000,
            TEST_VEL_RATIO,
            TEST_ACC_RATIO,
            ARM_JOINT_K,
            ARM_JOINT_D,
        )
        ret_body = robot.switch_to_imp_joint_mode(
            FXObjType.OBJ_BODY,
            1000,
            TEST_VEL_RATIO,
            TEST_ACC_RATIO,
            BODY_JOINT_K_FOR_SWITCH,
            BODY_JOINT_D_FOR_SWITCH,
        )
        if ret_a != 0 or ret_b != 0 or ret_body != 0:
            raise RuntimeError(
                f"switch_to_imp_joint_mode failed, ret_a={ret_a}, ret_b={ret_b}, "
                f"ret_body={ret_body}"
            )

        if not robot.comm_clear(50):
            raise RuntimeError("comm_clear failed before setting impedance K/D")

        if not robot.runtime_set_joint_kd(FXObjType.OBJ_ARM0, ARM_JOINT_K, ARM_JOINT_D):
            raise RuntimeError("runtime_set_joint_kd failed for ARM0")
        if not robot.runtime_set_joint_kd(FXObjType.OBJ_ARM1, ARM_JOINT_K, ARM_JOINT_D):
            raise RuntimeError("runtime_set_joint_kd failed for ARM1")
        if not robot.runtime_set_body_pd(BODY_JOINT_K, BODY_JOINT_D):
            raise RuntimeError("runtime_set_body_pd failed for BODY")
        if not robot.comm_send():
            raise RuntimeError("comm_send failed after setting impedance K/D")

        collector = start_luna_feedback_collector(
            robot=robot,
            poll_period_s=POLL_PERIOD_S,
            stale_threshold_s=STALE_THRESHOLD_S,
        )

        run_t0 = time.perf_counter()

        for group_cfg in GROUP_TESTS:
            group_name = group_cfg["group_name"]
            group_type = group_cfg["group_type"]
            group_index = group_cfg["group_index"]
            joint_count = int(group_cfg["joint_count"])
            amp_deg_list = list(group_cfg["amp_deg"])

            for joint_idx in range(joint_count):
                print(f"start test: {group_name} J{joint_idx + 1}")

                count = 0
                max_count = int(TIME_PER_JOINT / DT)
                next_deadline = time.perf_counter()

                while True:
                    fade_in = min(1.0, count * DT / 2.0) #start in smoothly
                    sin_term = fade_in * math.sin(2.0 * math.pi * count * DT * SIN_FREQ_HZ)

                    left_pos = LEFT_DEFAULT_POS.copy()
                    right_pos = RIGHT_DEFAULT_POS.copy()
                    head_pos = head_default_cmd_pos.copy()
                    body_pos = body_default_pos.copy()

                    amp = safe_float(
                        amp_deg_list[joint_idx] if joint_idx < len(amp_deg_list) else 0.0,
                        0.0,
                    )

                    if group_name == "left_arm":
                        left_pos[joint_idx] += amp * sin_term
                        q_cmd_active_group = left_pos

                    elif group_name == "right_arm":
                        right_pos[joint_idx] += amp * sin_term
                        q_cmd_active_group = right_pos

                    elif group_name == "head":
                        head_pos[joint_idx] += amp * sin_term
                        q_cmd_active_group = head_pos[:2]

                    elif group_name == "body":
                        body_pos[joint_idx] += amp * sin_term
                        q_cmd_active_group = body_pos[:6]

                    else:
                        raise RuntimeError(f"Unsupported group_name: {group_name}")

                    send_all_pos_cmds(
                        robot,
                        left_pos,
                        right_pos,
                        head_pos,
                        body_pos,
                        stage="in loop",
                    )

                    send_ts_perf_s = time.perf_counter()

                    append_luna_feedback_sample(
                        collector=collector,
                        group_name=group_name,
                        group_type=group_type,
                        group_index=group_index,
                        joint_idx=joint_idx,
                        q_cmd_deg=q_cmd_active_group,
                        t_cmd_s=send_ts_perf_s - run_t0,
                        send_ts_perf_s=send_ts_perf_s,
                        test_name=TEST_NAME,
                    )

                    count += 1
                    if count > max_count:
                        break

                    next_deadline += DT
                    now = time.perf_counter()
                    if next_deadline < now:
                        next_deadline = now
                    sleep_until(next_deadline)

                send_all_pos_cmds(
                    robot,
                    LEFT_DEFAULT_POS,
                    RIGHT_DEFAULT_POS,
                    head_default_cmd_pos,
                    body_default_pos,
                    stage="restore after one joint",
                )
                time.sleep(RESTORE_BETWEEN_JOINT_S)
                print(f"finish test: {group_name} J{joint_idx + 1}")

        collector_summary = finalize_luna_feedback_collector(collector)
        collector = None

        output = save_and_plot_luna_result(
            test_name=TEST_NAME,
            collector_summary=collector_summary,
            group_tests=GROUP_TESTS,
            result_root_dir=RESULT_ROOT_DIR,
            robot_ip=ROBOT_IP,
            dt=DT,
            sin_freq_hz=SIN_FREQ_HZ,
            time_per_joint=TIME_PER_JOINT,
            vel_ratio=TEST_VEL_RATIO,
            acc_ratio=TEST_ACC_RATIO,
            poll_period_s=POLL_PERIOD_S,
            stale_threshold_s=STALE_THRESHOLD_S,
        )
        print_luna_output_summary(output)

    except Exception as exc:
        print(f"test aborted: {exc}")
        traceback.print_exc()

    finally:
        if collector is not None:
            try:
                finalize_luna_feedback_collector(collector)
            except Exception as finalize_exc:
                print(f"warning: finalize_luna_feedback_collector failed: {finalize_exc}")
            collector = None

        if robot is not None:
            try:
                ret_a = robot.switch_to_position_mode(
                    FXObjType.OBJ_ARM0, 1000, INIT_VEL_RATIO, INIT_ACC_RATIO
                )
                ret_b = robot.switch_to_position_mode(
                    FXObjType.OBJ_ARM1, 1000, INIT_VEL_RATIO, INIT_ACC_RATIO
                )
                ret_h = robot.switch_to_position_mode(
                    FXObjType.OBJ_HEAD, 1000, INIT_VEL_RATIO, INIT_ACC_RATIO
                )
                ret_body = robot.switch_to_position_mode(
                    FXObjType.OBJ_BODY, 1000, INIT_VEL_RATIO, INIT_ACC_RATIO
                )
                if ret_a != 0 or ret_b != 0 or ret_h != 0 or ret_body != 0:
                    print(
                        f"warning: finally switch_to_position_mode failed, "
                        f"ret_a={ret_a}, ret_b={ret_b}, ret_h={ret_h}, ret_body={ret_body}"
                    )

                send_all_pos_cmds(
                    robot,
                    LEFT_DEFAULT_POS,
                    RIGHT_DEFAULT_POS,
                    head_default_cmd_pos,
                    body_default_pos,
                    stage="finally restore defaults",
                )
                time.sleep(0.2)
            except Exception as restore_exc:
                print(f"warning: restore defaults failed: {restore_exc}")

            try:
                robot.switch_to_idle(FXObjType.OBJ_ARM0, 1000)
                robot.switch_to_idle(FXObjType.OBJ_ARM1, 1000)
                robot.switch_to_idle(FXObjType.OBJ_HEAD, 1000)
                robot.switch_to_idle(FXObjType.OBJ_BODY, 1000)
            except Exception as idle_exc:
                print(f"warning: switch_to_idle failed: {idle_exc}")

            try:
                robot.cleanup()
            except Exception as cleanup_exc:
                print(f"warning: cleanup failed: {cleanup_exc}")

        _release_high_res_timer()


if __name__ == "__main__":
    main()
