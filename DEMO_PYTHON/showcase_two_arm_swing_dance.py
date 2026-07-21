import argparse
import math
import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot


M6_LIMITS = [
    (-170.0, 170.0),
    (-120.0, 120.0),
    (-170.0, 170.0),
    (-145.0, 60.0),
    (-170.0, 170.0),
    (-60.0, 60.0),
    (-90.0, 90.0),
]


def clamp(value, low, high):
    return max(low, min(high, value))


def clamp_joints(joints):
    return [clamp(v, low, high) for v, (low, high) in zip(joints, M6_LIMITS)]


def get_current_joints(robot, dcss):
    for _ in range(20):
        sub_data = robot.subscribe(dcss)
        out_a = sub_data["outputs"][0]
        out_b = sub_data["outputs"][1]
        joints_a = [float(x) for x in out_a["fb_joint_pos"]]
        joints_b = [float(x) for x in out_b["fb_joint_pos"]]
        if len(joints_a) == 7 and len(joints_b) == 7:
            return joints_a, joints_b
        time.sleep(0.05)
    raise RuntimeError("failed to read current joint feedback")


def build_dance_targets(center_a, center_b, t, total_time, period, scale):
    phase = 2.0 * math.pi * t / period
    envelope = math.sin(math.pi * min(t, total_time) / total_time)

    shoulder = 7.0 * scale * math.sin(phase)
    elbow = 5.0 * scale * math.sin(phase + math.pi / 2.0)
    wrist_roll = 10.0 * scale * math.sin(phase + math.pi)
    wrist_sway = 7.0 * scale * math.sin(2.0 * phase)
    hand_wave = 12.0 * scale * math.sin(phase + math.pi / 3.0)

    # Small mirror-like joint offsets around the current pose.
    offset_a = [
        0.0,
        shoulder,
        -elbow,
        0.0,
        wrist_roll,
        wrist_sway,
        hand_wave,
    ]
    offset_b = [
        0.0,
        -shoulder,
        elbow,
        0.0,
        -wrist_roll,
        -wrist_sway,
        -hand_wave,
    ]

    joints_a = [c + envelope * o for c, o in zip(center_a, offset_a)]
    joints_b = [c + envelope * o for c, o in zip(center_b, offset_b)]
    return clamp_joints(joints_a), clamp_joints(joints_b)


def main():
    parser = argparse.ArgumentParser(description="Two-arm swing dance demo for Marvin robot.")
    parser.add_argument("--ip", default="192.168.1.190", help="robot IP")
    parser.add_argument("--cycles", type=float, default=3.0, help="dance cycles")
    parser.add_argument("--period", type=float, default=4.0, help="seconds per cycle")
    parser.add_argument("--dt", type=float, default=0.05, help="command period in seconds")
    parser.add_argument("--vel", type=int, default=15, help="position mode velocity ratio")
    parser.add_argument("--acc", type=int, default=15, help="position mode acceleration ratio")
    parser.add_argument("--scale", type=float, default=1.0, help="motion amplitude scale, suggested 0.3~1.0")
    parser.add_argument("--run", action="store_true", help="actually send dance commands")
    parser.add_argument("--keep-enabled", action="store_true", help="do not disable arms after finishing")
    args = parser.parse_args()

    args.scale = clamp(args.scale, 0.1, 1.5)
    total_time = max(args.period * args.cycles, args.dt)

    dcss = DCSS()
    robot = Marvin_Robot()

    if not robot.connect(args.ip):
        print("failed to connect to robot")
        return 1

    try:
        robot.check_error_and_clear(dcss)
        center_a, center_b = get_current_joints(robot, dcss)

        print("Current A center:", [round(x, 2) for x in center_a])
        print("Current B center:", [round(x, 2) for x in center_b])
        print(f"Dance duration: {total_time:.2f}s, period: {args.period:.2f}s, dt: {args.dt:.3f}s, scale: {args.scale:.2f}")

        sample_a, sample_b = build_dance_targets(center_a, center_b, args.period / 4.0, total_time, args.period, args.scale)
        print("Sample A target:", [round(x, 2) for x in sample_a])
        print("Sample B target:", [round(x, 2) for x in sample_b])

        if not args.run:
            print("Preview only. Add --run to execute the dance.")
            return 0

        robot.clear_set()
        robot.set_state(arm="A", state=1)
        robot.set_vel_acc(arm="A", velRatio=args.vel, AccRatio=args.acc)
        robot.set_state(arm="B", state=1)
        robot.set_vel_acc(arm="B", velRatio=args.vel, AccRatio=args.acc)
        robot.send_cmd()
        time.sleep(0.5)

        start = time.time()
        while True:
            elapsed = time.time() - start
            if elapsed > total_time:
                break
            joints_a, joints_b = build_dance_targets(center_a, center_b, elapsed, total_time, args.period, args.scale)
            robot.clear_set()
            robot.set_joint_cmd_pose(arm="A", joints=joints_a)
            robot.set_joint_cmd_pose(arm="B", joints=joints_b)
            robot.send_cmd()
            time.sleep(args.dt)

        robot.clear_set()
        robot.set_joint_cmd_pose(arm="A", joints=center_a)
        robot.set_joint_cmd_pose(arm="B", joints=center_b)
        robot.send_cmd()
        time.sleep(1.0)

        if not args.keep_enabled:
            robot.clear_set()
            robot.set_state(arm="A", state=0)
            robot.set_state(arm="B", state=0)
            robot.send_cmd()

        print("Dance finished.")
        return 0
    except KeyboardInterrupt:
        print("Interrupted, releasing robot.")
        return 130
    finally:
        robot.release_robot()


if __name__ == "__main__":
    raise SystemExit(main())
