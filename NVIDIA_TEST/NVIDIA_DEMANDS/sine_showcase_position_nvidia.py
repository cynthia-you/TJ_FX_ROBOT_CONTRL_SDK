import atexit
import ctypes
import logging
import math
import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot
from nvidia_feedback_tools import (
    append_feedback_sample,
    finalize_feedback_collector,
    start_feedback_collector,
)
from nvidia_plot_tools import generate_showcase_plots


# 配置日志系统
logging.basicConfig(format="%(message)s")
logger = logging.getLogger("debug_printer")
logger.setLevel(logging.INFO)


# ==================== 配置区（给客户看的） ====================
ROBOT_IP = "192.168.1.190"

DT = 0.02
RUN_TIME_S = 20.0
SIN_FREQ_HZ = 0.5

VEL_RATIO = 100
ACC_RATIO = 100
MOVE_TO_DEFAULT_WAIT_S = 5.0

LEFT_DEFAULT_POS = [90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0]
RIGHT_DEFAULT_POS = [90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0]

SIN_AMP_LEFT_J1_DEG = 10.0
SIN_AMP_LEFT_J4_DEG = 10.0
SIN_AMP_LEFT_J6_DEG = 30.0

SIN_AMP_RIGHT_J1_DEG = 10.0
SIN_AMP_RIGHT_J4_DEG = 10.0
SIN_AMP_RIGHT_J6_DEG = 30.0


def sleep_until(deadline, spin_threshold=0.0005):
    """Sleep until an absolute perf_counter deadline with sub-ms trim."""
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


# 激活 Windows 高精度定时器
winmm = None
high_res_timer_enabled = False
if os.name == "nt":
    try:
        winmm = ctypes.WinDLL("winmm")
        if winmm.timeBeginPeriod(1) == 0:
            high_res_timer_enabled = True
            logger.info("enabled Windows high-resolution timer: 1ms")
        else:
            logger.warning("failed to enable Windows high-resolution timer")
    except Exception as exc:
        logger.warning(f"failed to load winmm timer APIs: {exc}")


def _release_high_res_timer():
    if high_res_timer_enabled and winmm is not None:
        winmm.timeEndPeriod(1)


atexit.register(_release_high_res_timer)


def main():
    dcss = DCSS()
    robot = Marvin_Robot()

    try:
        # connect
        init = robot.connect(ROBOT_IP)
        if init == 0:
            raise RuntimeError("failed to connect to the robot, port is occupied")

        # clear error
        robot.check_error_and_clear(dcss)

        # 通过左右臂 frame 刷新确认 UDP 通道都正常
        left_motion_tag = 0
        right_motion_tag = 0
        left_frame_last = None
        right_frame_last = None
        for _ in range(5):
            sub_data = robot.subscribe(dcss)
            left_frame = sub_data["outputs"][0]["frame_serial"]
            right_frame = sub_data["outputs"][1]["frame_serial"]
            print(f"connect frame A: {left_frame}, B: {right_frame}")

            if left_frame != 0 and left_frame != left_frame_last:
                left_motion_tag += 1
                left_frame_last = left_frame
            if right_frame != 0 and right_frame != right_frame_last:
                right_motion_tag += 1
                right_frame_last = right_frame
            time.sleep(0.1)

        if left_motion_tag <= 0 or right_motion_tag <= 0:
            raise RuntimeError("failed: robot connection failed (A/B frame not updating)")
        logger.info("success: robot connected")

        # set mode(position)
        robot.clear_set()
        robot.set_state(arm="A", state=0)
        robot.set_state(arm="B", state=0)
        robot.set_vel_acc(arm="A", velRatio=VEL_RATIO, AccRatio=ACC_RATIO)
        robot.set_vel_acc(arm="B", velRatio=VEL_RATIO, AccRatio=ACC_RATIO)
        robot.send_cmd()
        time.sleep(1.0)

        robot.clear_set()
        robot.set_state(arm="A", state=1)
        robot.send_cmd()
        time.sleep(2.0)
        robot.clear_set()
        robot.set_state(arm="B", state=1)
        robot.send_cmd()
        time.sleep(1.0)

        sub_data = robot.subscribe(dcss)
        logger.info("-----------")
        logger.info("A arm:")
        logger.info(f"current state: {sub_data['states'][0]['cur_state']}")
        logger.info(f"arm error code: {sub_data['states'][0]['err_code']}")
        logger.info(
            f"vel={sub_data['inputs'][0]['joint_vel_ratio']}, "
            f"acc={sub_data['inputs'][0]['joint_acc_ratio']}"
        )
        logger.info("-----------")
        logger.info("B arm:")
        logger.info(f"current state: {sub_data['states'][1]['cur_state']}")
        logger.info(f"arm error code: {sub_data['states'][1]['err_code']}")
        logger.info(
            f"vel={sub_data['inputs'][1]['joint_vel_ratio']}, "
            f"acc={sub_data['inputs'][1]['joint_acc_ratio']}"
        )

        # move to default
        robot.clear_set()
        robot.set_joint_cmd_pose(arm="A", joints=LEFT_DEFAULT_POS)
        robot.set_joint_cmd_pose(arm="B", joints=RIGHT_DEFAULT_POS)
        robot.send_cmd()
        time.sleep(MOVE_TO_DEFAULT_WAIT_S)

        # 启动后台异步采样器
        collector = start_feedback_collector(
            robot=robot,
            dcss=dcss,
            collection_mode="async",
            poll_period_s=0.02,
            stale_threshold_s=0.100,
        )

        # 跑双臂同步正弦
        run_t0 = time.perf_counter()
        next_deadline = run_t0
        count = 0
        max_count = int(RUN_TIME_S / DT)

        while True:
            sin_term = math.sin(2.0 * math.pi * count * DT * SIN_FREQ_HZ)

            left_q_cmd = LEFT_DEFAULT_POS.copy()
            left_q_cmd[0] += SIN_AMP_LEFT_J1_DEG * sin_term
            left_q_cmd[3] += SIN_AMP_LEFT_J4_DEG * sin_term
            left_q_cmd[5] += SIN_AMP_LEFT_J6_DEG * sin_term

            right_q_cmd = RIGHT_DEFAULT_POS.copy()
            right_q_cmd[0] += SIN_AMP_RIGHT_J1_DEG * sin_term
            right_q_cmd[3] += SIN_AMP_RIGHT_J4_DEG * sin_term
            right_q_cmd[5] += SIN_AMP_RIGHT_J6_DEG * sin_term

            robot.clear_set()
            robot.set_joint_cmd_pose(arm="A", joints=left_q_cmd)
            robot.set_joint_cmd_pose(arm="B", joints=right_q_cmd)
            robot.send_cmd()

            send_ts_perf_s = time.perf_counter()
            append_feedback_sample(
                collector=collector,
                left_q_cmd_deg=left_q_cmd,
                right_q_cmd_deg=right_q_cmd,
                t_cmd_s=send_ts_perf_s - run_t0,
                send_ts_perf_s=send_ts_perf_s,
            )

            
            if count > max_count:
                break
            count += 1

            next_deadline += DT
            sleep_until(next_deadline)

        # 收尾整理
        result = finalize_feedback_collector(collector)

        # 出图和导出 json
        plot_paths, metrics, json_path = generate_showcase_plots(
            result=result,
            output_dir=os.path.join(current_dir, "results_nvidia_showcase"),
            tag=time.strftime("%Y%m%d_%H%M%S"),
            mode_title="NVIDIA Position Showcase",
            ctrl_hz=1.0 / DT,
            save_json=True,
        )

        # 最短打印逻辑（双臂）
        print(f"json: {json_path}")
        for arm_name in ("left", "right"):
            arm_plots = plot_paths.get(arm_name, {})
            for key, value in arm_plots.items():
                print(f"{arm_name}_{key}: {value}")

        print(f"left_lag_estimated_s: {metrics.get('left', {}).get('lag_estimated_s')}")
        print(f"right_lag_estimated_s: {metrics.get('right', {}).get('lag_estimated_s')}")

        timing_metrics = metrics.get("timing", {})
        print(f"delta_t_mean_ms: {timing_metrics.get('delta_t_mean_ms')}")
        print(f"delta_t_std_ms: {timing_metrics.get('delta_t_std_ms')}")
        print(f"delta_t_max_ms: {timing_metrics.get('delta_t_max_ms')}")
        print(f"delta_t_p95_ms: {timing_metrics.get('delta_t_p95_ms')}")
        print(f"jitter_mean_ms: {timing_metrics.get('jitter_mean_ms')}")
        print(f"jitter_std_ms: {timing_metrics.get('jitter_std_ms')}")
        print(f"jitter_max_ms: {timing_metrics.get('jitter_max_ms')}")
        print(f"jitter_p95_ms: {timing_metrics.get('jitter_p95_ms')}")

    finally:
        # 下使能 + 释放资源
        try:
            robot.clear_set()
            robot.set_state(arm="A", state=0)
            robot.set_state(arm="B", state=0)
            robot.send_cmd()
        except Exception as exc:
            logger.warning(f"failed to disable arms: {exc}")

        try:
            robot.release_robot()
        except Exception as exc:
            logger.warning(f"failed to release robot: {exc}")


if __name__ == "__main__":
    main()
