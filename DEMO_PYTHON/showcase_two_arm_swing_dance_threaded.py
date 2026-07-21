import argparse
import math
import os
import sys
import threading
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
        joints_a = [float(x) for x in sub_data["outputs"][0]["fb_joint_pos"]]
        joints_b = [float(x) for x in sub_data["outputs"][1]["fb_joint_pos"]]
        if len(joints_a) == 7 and len(joints_b) == 7:
            return joints_a, joints_b
        time.sleep(0.05)
    raise RuntimeError("failed to read current joint feedback")


def build_arm_target(center, arm, t, total_time, period, scale):
    phase = 2.0 * math.pi * t / period
    envelope = math.sin(math.pi * min(t, total_time) / total_time)
    sign = 1.0 if arm == "A" else -1.0

    offsets = [
        0.0,
        sign * 7.0 * scale * math.sin(phase),
        -sign * 5.0 * scale * math.sin(phase + math.pi / 2.0),
        0.0,
        sign * 10.0 * scale * math.sin(phase + math.pi),
        sign * 7.0 * scale * math.sin(2.0 * phase),
        sign * 12.0 * scale * math.sin(phase + math.pi / 3.0),
    ]
    return clamp_joints([c + envelope * o for c, o in zip(center, offsets)])


def arm_worker(robot, sdk_lock, stop_event, arm, center, args, start_time):
    total_time = max(args.period * args.cycles, args.dt)
    next_tick = time.time()

    while not stop_event.is_set():
        elapsed = time.time() - start_time
        if elapsed > total_time:
            break

        joints = build_arm_target(center, arm, elapsed, total_time, args.period, args.scale)
        with sdk_lock:
            robot.clear_set()
            robot.set_joint_cmd_pose(arm=arm, joints=joints)
            robot.send_cmd()

        next_tick += args.dt
        time.sleep(max(0.0, next_tick - time.time()))


def main():
    parser = argparse.ArgumentParser(description="Threaded two-arm swing dance demo for Marvin robot.")
    parser.add_argument("--ip", default="192.168.1.190", help="robot IP")
    parser.add_argument("--cycles", type=float, default=3.0, help="dance cycles")
    parser.add_argument("--period", type=float, default=4.0, help="seconds per cycle")
    parser.add_argument("--dt", type=float, default=0.05, help="command period per arm thread")
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
    sdk_lock = threading.Lock()
    stop_event = threading.Event()

    if not robot.connect(args.ip):
        print("failed to connect to robot")
        return 1

    try:
        robot.check_error_and_clear(dcss)
        center_a, center_b = get_current_joints(robot, dcss)

        print("Current A center:", [round(x, 2) for x in center_a])
        print("Current B center:", [round(x, 2) for x in center_b])
        print(f"Threaded dance duration: {total_time:.2f}s, dt: {args.dt:.3f}s, scale: {args.scale:.2f}")

        if not args.run:
            print("Preview only. Add --run to execute threaded dance.")
            return 0

        with sdk_lock:
            robot.clear_set()
            robot.set_state(arm="A", state=1)
            robot.set_vel_acc(arm="A", velRatio=args.vel, AccRatio=args.acc)
            robot.set_state(arm="B", state=1)
            robot.set_vel_acc(arm="B", velRatio=args.vel, AccRatio=args.acc)
            robot.send_cmd()
        time.sleep(0.5)

        start_time = time.time()
        thread_a = threading.Thread(
            target=arm_worker,
            args=(robot, sdk_lock, stop_event, "A", center_a, args, start_time),
            name="dance-arm-A",
        )
        thread_b = threading.Thread(
            target=arm_worker,
            args=(robot, sdk_lock, stop_event, "B", center_b, args, start_time),
            name="dance-arm-B",
        )

        thread_a.start()
        thread_b.start()
        thread_a.join()
        thread_b.join()

        with sdk_lock:
            robot.clear_set()
            robot.set_joint_cmd_pose(arm="A", joints=center_a)
            robot.set_joint_cmd_pose(arm="B", joints=center_b)
            robot.send_cmd()
        time.sleep(1.0)

        if not args.keep_enabled:
            with sdk_lock:
                robot.clear_set()
                robot.set_state(arm="A", state=0)
                robot.set_state(arm="B", state=0)
                robot.send_cmd()

        print("Threaded dance finished.")
        return 0
    except KeyboardInterrupt:
        stop_event.set()
        print("Interrupted, stopping threads and releasing robot.")
        return 130
    finally:
        robot.release_robot()


if __name__ == "__main__":
    raise SystemExit(main())
