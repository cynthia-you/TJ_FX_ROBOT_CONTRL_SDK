"""
@file example_basic_Position_with_data_collection.py
@brief 在原位置控制示例基础上增加后台实时数据采集功能。
       连接成功后即开始持续刷新数据，主线程可随时获取最新状态。
"""

import sys
import time
import threading
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map, error_dict


class BackgroundDataFetcher:
    def __init__(self, robot, rt_interval=0.02, sg_interval=0.05):
        self.robot = robot
        self._rt_interval = rt_interval
        self._sg_interval = sg_interval
        self._latest_rt = None
        self._latest_sg = None
        self._running = False
        self._lock = threading.Lock()
        self._rt_thread = None
        self._sg_thread = None

    def start(self):
        if self._running:
            return
        self._running = True
        self._rt_thread = threading.Thread(target=self._fetch_rt, daemon=True)
        self._sg_thread = threading.Thread(target=self._fetch_sg, daemon=True)
        self._rt_thread.start()
        self._sg_thread.start()
        print("[DataFetcher] Started background fetching.")

    def _fetch_rt(self):
        while self._running:
            try:
                data = self.robot.get_rt_dict()
                if data is not None:
                    with self._lock:
                        self._latest_rt = data
            except Exception as e:
                print(f"[DataFetcher] RT fetch error: {e}")
            time.sleep(self._rt_interval)

    def _fetch_sg(self):
        while self._running:
            try:
                data = self.robot.get_sg_dict()
                if data is not None:
                    with self._lock:
                        self._latest_sg = data
            except Exception as e:
                print(f"[DataFetcher] SG fetch error: {e}")
            time.sleep(self._sg_interval)

    @property
    def latest_rt(self):
        with self._lock:
            return dict(self._latest_rt) if self._latest_rt else None

    @property
    def latest_sg(self):
        with self._lock:
            return dict(self._latest_sg) if self._latest_sg else None

    def stop(self, timeout=2.0):
        self._running = False
        if self._rt_thread and self._rt_thread.is_alive():
            self._rt_thread.join(timeout=timeout)
        if self._sg_thread and self._sg_thread.is_alive():
            self._sg_thread.join(timeout=timeout)
        print("[DataFetcher] Stopped.")


def main():
    def emergency_stop_thread(robot, fetcher=None):
        input()
        print("\n[Emergency stop] Triggered. Stopping robot...")
        robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
        time.sleep(0.1)
        if fetcher:
            fetcher.stop()
        print("Exiting program.")
        sys.exit(0)

    ctrl_obj = FXObjType.OBJ_ARM0
    target_state = "Position"
    arm_idx = 0
    if ctrl_obj == FXObjType.OBJ_ARM1:
        arm_idx = 1
    log_mask = FXLogMask.FX_LOG_INFO_FLAG
    vel = 10
    acc = 10
    pos1 = [0] * 7
    pos2 = [10, 10, 10, -90, 10, 10, 10]

    robot = GentoRobot()

    print(f"\n### 1/6 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/6. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    data_fetcher = BackgroundDataFetcher(robot, rt_interval=0.02, sg_interval=0.05)
    data_fetcher.start()

    estop_thread = threading.Thread(target=emergency_stop_thread, args=(robot, data_fetcher), daemon=True)
    estop_thread.start()

    print(f"\n### 3/6. Position state switching...")
    arm0_state = robot.current_state(ctrl_obj)
    if state_map[arm0_state] == "Error":
        ret, system_errorcode = robot.reset_error(ctrl_obj, 1000)
        if ret != 0:
            print(f"Reset error failed. Error msg: {error_dict[system_errorcode]}")
            return
    elif state_map[arm0_state] != "IDLE":
        ret = robot.switch_to_idle(ctrl_obj, 1000)
        if ret != 0:
            print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

    ret = robot.switch_to_position_mode(ctrl_obj, 2000, vel, acc)
    if ret != 0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    print(f"Already in {target_state} state.")


    latest = data_fetcher.latest_rt
    if latest:
        print(f"current state (from fetcher)：{latest['arms'][arm_idx]['state']['cur']}")
        print(f"current joints (from fetcher)：{latest['arms'][arm_idx]['fb']['fb_pos']}")
    else:
        rt_dict = robot.get_rt_dict()
        print(f"current state：{rt_dict['arms'][arm_idx]['state']['cur']}")
        print(f"current joints：{rt_dict['arms'][arm_idx]['fb']['fb_pos']}")

    sg = data_fetcher.latest_sg
    if sg:
        print(f"current vel (from fetcher)：{sg['arms'][arm_idx]['set']['vel_ratio']}")
        print(f"current acc (from fetcher)：{sg['arms'][arm_idx]['set']['acc_ratio']}")
    else:
        sg_dict = robot.get_sg_dict()
        print(f"current vel：{sg_dict['arms'][arm_idx]['set']['vel_ratio']}")
        print(f"current acc：{sg_dict['arms'][arm_idx]['set']['acc_ratio']}")

    print(f"\n### 4/6. Run pos1 and pos2...")
    ret = robot.runtime_set_joint_pos_cmd(ctrl_obj, pos1)
    if ret != 0:
        print(f"Set joint command failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    while True:
        latest = data_fetcher.latest_rt
        if latest is None:
            fb_joints = robot.get_rt_dict()["arms"][arm_idx]["fb"]["fb_pos"]
        else:
            fb_joints = latest["arms"][arm_idx]["fb"]["fb_pos"]

        if robot.check_sequences_approx_equal(fb_joints, pos1):
            break
        time.sleep(0.01)

    print(f"Arm0 reached at {fb_joints}")

    time.sleep(0.5)

    ret = robot.runtime_set_joint_pos_cmd(ctrl_obj, pos2)
    if ret != 0:
        print(f"Set joint command failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    while True:
        latest = data_fetcher.latest_rt
        if latest is None:
            fb_joints = robot.get_rt_dict()["arms"][arm_idx]["fb"]["fb_pos"]
        else:
            fb_joints = latest["arms"][arm_idx]["fb"]["fb_pos"]

        if robot.check_sequences_approx_equal(fb_joints, pos2):
            break
        time.sleep(0.01)

    print(f"Arm0 reached at {fb_joints}")

    print(f"\n### 5/6. Switching to IDLE...")
    ret = robot.switch_to_idle(ctrl_obj, 2000)
    if ret != 0:
        print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 6/6. Task finished.")

    data_fetcher.stop()


if __name__ == "__main__":
    main()
