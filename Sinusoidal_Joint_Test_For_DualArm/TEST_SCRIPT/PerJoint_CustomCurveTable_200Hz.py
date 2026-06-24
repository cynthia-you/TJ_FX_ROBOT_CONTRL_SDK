"""
Per-joint custom sine-curve sweep at 200 Hz for Marvin M6.

Purpose
-------
This script is intended for the retest workflow discussed for the Marvin M6
joint tracking report:

1. Reuse the safe robot startup/shutdown and command scheduling style from the
   existing TEST_SCRIPT example.
2. Enter joint impedance / PD mode and keep one fixed K/D setting for the
   whole sweep.
3. Use cross-platform timing setup + sleep_until scheduling for strict
   200 Hz command timing (Windows timeBeginPeriod; Linux best-effort tuning).
4. Use a configurable sine-curve table: q = home + A*sin(2*pi*omega*t).
5. Pre-check every table row against theoretical velocity and acceleration limits.
6. Skip theoretically invalid tests and record the reason instead of moving.
7. Buffer per-trial samples in preallocated NumPy arrays, then save NPZ/CSV and summary timing/tracking metadata after each trial.

Place this file either:
- next to the SDK_PYTHON folder, or
- inside TEST_SCRIPT/ with SDK_PYTHON/ as its sibling folder.

Typical usage on the test PC:
    python PerJoint_CustomCurveTable_200Hz.py --precheck_only
    python PerJoint_CustomCurveTable_200Hz.py --joint_filter J1 --no_confirm
    python PerJoint_CustomCurveTable_200Hz.py --table_json my_curve_table.json --no_confirm
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import json
import math
import os
import platform
import re
import statistics
import sys
import time

import numpy as np

from dataclasses import dataclass, asdict, fields
from datetime import datetime
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# Import SDK_PYTHON from common deployment layouts.
# ---------------------------------------------------------------------------
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(CURRENT_DIR)
for candidate in (CURRENT_DIR, PARENT_DIR):
    if candidate not in sys.path:
        sys.path.insert(0, candidate)

# SDK import is intentionally lazy so that --precheck_only can run on any PC,
# even without loading the robot DLL. The real robot classes are imported in
# load_robot_sdk() before any motion command is executed.
Marvin_Robot = None
DCSS = None


def load_robot_sdk() -> None:
    global Marvin_Robot, DCSS
    if Marvin_Robot is not None and DCSS is not None:
        return
    from SDK_PYTHON.fx_robot import DCSS as _DCSS, Marvin_Robot as _Marvin_Robot  # noqa: E402

    Marvin_Robot = _Marvin_Robot
    DCSS = _DCSS


# ---------------------------------------------------------------------------
# Cross-platform path and timing helpers.
# ---------------------------------------------------------------------------
def normalize_cli_path(path_text: object) -> str:
    """Normalize command-line paths from either Windows or POSIX shells.

    Linux shells treat backslashes in strings like ``.\\results\\run`` as normal
    characters, not separators.  This helper keeps native paths unchanged when
    they already exist, and on non-Windows platforms converts common Windows
    relative separators to POSIX separators.  Windows drive-letter paths such as
    ``C:\\...`` are intentionally not translated on Linux; mount them explicitly
    and pass the mounted POSIX path instead.
    """
    raw = os.path.expanduser(os.path.expandvars(str(path_text).strip()))
    if not raw:
        return raw
    if os.path.exists(raw):
        return raw
    if os.name != "nt" and "\\" in raw:
        # Convert relative Windows-style paths copied from PowerShell/CMD.
        if not re.match(r"^[A-Za-z]:[\\/]", raw):
            return raw.replace("\\", os.sep)
    return raw


def parse_cpu_affinity(text: object) -> Optional[set[int]]:
    """Parse CPU affinity like '2' or '2,3,4-6'. Empty means no change."""
    raw = str(text or "").strip()
    if not raw:
        return None
    cpus: set[int] = set()
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start, end = int(start_s), int(end_s)
            if end < start:
                raise ValueError(f"Invalid CPU range in --cpu_affinity: {part!r}")
            cpus.update(range(start, end + 1))
        else:
            cpus.add(int(part))
    if not cpus:
        return None
    return cpus


def perf_counter_info() -> Dict[str, object]:
    info = time.get_clock_info("perf_counter")
    return {
        "implementation": info.implementation,
        "monotonic": bool(info.monotonic),
        "adjustable": bool(info.adjustable),
        "resolution_s": float(info.resolution),
    }


class PlatformTimingTuner:
    """Best-effort timing setup for the 200 Hz command loop.

    Windows: calls winmm.timeBeginPeriod(1), matching the original script.
    Linux: optionally reduces process timer slack, optionally pins CPU affinity,
    and optionally requests SCHED_FIFO priority.  The Linux real-time scheduler
    request normally requires sudo or CAP_SYS_NICE; failure is non-fatal and is
    recorded in ``summary()``.
    """

    def __init__(
        self,
        period_ms: int = 1,
        linux_timer_slack_ns: int = 50_000,
        linux_rt_priority: int = 0,
        cpu_affinity: object = "",
    ):
        self.period_ms = int(period_ms)
        self.linux_timer_slack_ns = int(linux_timer_slack_ns)
        self.linux_rt_priority = int(linux_rt_priority)
        self.cpu_affinity_text = str(cpu_affinity or "").strip()
        self._winmm = None
        self._orig_affinity = None
        self._orig_sched = None
        self.enabled = False
        self.info: Dict[str, object] = {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "os_name": os.name,
            "system": platform.system(),
            "perf_counter": perf_counter_info(),
            "windows_timeBeginPeriod_enabled": False,
            "linux_timer_slack_ns_requested": None,
            "linux_timer_slack_applied": False,
            "linux_rt_priority_requested": None,
            "linux_rt_priority_applied": False,
            "cpu_affinity_requested": self.cpu_affinity_text or None,
            "cpu_affinity_applied": False,
            "warnings": [],
        }

    def _warn(self, msg: str) -> None:
        self.info.setdefault("warnings", []).append(str(msg))
        print(f"[warn] {msg}")

    def start(self) -> None:
        if os.name == "nt":
            try:
                self._winmm = ctypes.WinDLL("winmm")
                ret = self._winmm.timeBeginPeriod(self.period_ms)
                ok = ret == 0
                self.enabled = bool(ok)
                self.info["windows_timeBeginPeriod_enabled"] = bool(ok)
                if not ok:
                    self._warn(f"timeBeginPeriod({self.period_ms}) failed: ret={ret}")
            except Exception as exc:
                self.enabled = False
                self._warn(f"timeBeginPeriod unavailable: {exc}")
            return

        if sys.platform.startswith("linux"):
            if self.linux_timer_slack_ns >= 0:
                self.info["linux_timer_slack_ns_requested"] = int(self.linux_timer_slack_ns)
                try:
                    PR_SET_TIMERSLACK = 29
                    libc = ctypes.CDLL(None, use_errno=True)
                    ret = libc.prctl(PR_SET_TIMERSLACK, ctypes.c_ulong(int(self.linux_timer_slack_ns)), 0, 0, 0)
                    ok = ret == 0
                    self.info["linux_timer_slack_applied"] = bool(ok)
                    self.enabled = self.enabled or bool(ok)
                    if not ok:
                        errno = ctypes.get_errno()
                        self._warn(f"Linux PR_SET_TIMERSLACK failed: errno={errno}")
                except Exception as exc:
                    self._warn(f"Linux timer slack tuning unavailable: {exc}")

            affinity = parse_cpu_affinity(self.cpu_affinity_text)
            if affinity is not None:
                try:
                    if hasattr(os, "sched_getaffinity") and hasattr(os, "sched_setaffinity"):
                        self._orig_affinity = os.sched_getaffinity(0)
                        os.sched_setaffinity(0, affinity)
                        self.info["cpu_affinity_applied"] = True
                        self.info["cpu_affinity_effective"] = sorted(os.sched_getaffinity(0))
                        self.enabled = True
                    else:
                        self._warn("CPU affinity API is not available on this Python/OS")
                except Exception as exc:
                    self._warn(f"CPU affinity request {sorted(affinity)} failed: {exc}")

            if self.linux_rt_priority > 0:
                self.info["linux_rt_priority_requested"] = int(self.linux_rt_priority)
                try:
                    if hasattr(os, "sched_setscheduler") and hasattr(os, "SCHED_FIFO"):
                        self._orig_sched = (os.sched_getscheduler(0), os.sched_getparam(0))
                        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(int(self.linux_rt_priority)))
                        self.info["linux_rt_priority_applied"] = True
                        self.enabled = True
                    else:
                        self._warn("Linux real-time scheduler API is not available on this Python/OS")
                except Exception as exc:
                    self._warn(
                        "Linux SCHED_FIFO request failed. Run with sudo, set CAP_SYS_NICE, "
                        f"or keep --linux_rt_priority 0. Details: {exc}"
                    )
            return

        self.info.setdefault("warnings", []).append("No OS-specific timing tuner for this platform; using perf_counter + sleep_until only.")

    def stop(self) -> None:
        if os.name == "nt":
            if self.info.get("windows_timeBeginPeriod_enabled") and self._winmm is not None:
                try:
                    self._winmm.timeEndPeriod(self.period_ms)
                except Exception as exc:
                    self._warn(f"timeEndPeriod failed: {exc}")
            return

        if sys.platform.startswith("linux"):
            if self._orig_sched is not None:
                try:
                    policy, param = self._orig_sched
                    os.sched_setscheduler(0, policy, param)
                except Exception as exc:
                    self._warn(f"restore Linux scheduler failed: {exc}")
            if self._orig_affinity is not None:
                try:
                    os.sched_setaffinity(0, self._orig_affinity)
                except Exception as exc:
                    self._warn(f"restore CPU affinity failed: {exc}")

    def summary(self) -> Dict[str, object]:
        self.info["enabled"] = bool(self.enabled)
        return dict(self.info)


def print_platform_timing_info(args: argparse.Namespace) -> None:
    print(
        f"[platform] {platform.system()} {platform.release()} | "
        f"Python {platform.python_version()} | perf_counter resolution "
        f"{perf_counter_info()['resolution_s'] * 1e6:.3f} us"
    )
    if sys.platform.startswith("linux"):
        print(
            "[platform] Linux mode: timeBeginPeriod is unavailable; using "
            "perf_counter + sleep_until, plus best-effort timer slack/affinity/RT options."
        )
        print(
            f"[platform] linux_timer_slack_ns={args.linux_timer_slack_ns}, "
            f"linux_rt_priority={args.linux_rt_priority}, "
            f"cpu_affinity={args.cpu_affinity or 'not set'}, "
            f"spin_threshold_s={args.spin_threshold_s}"
        )


# ---------------------------------------------------------------------------
# User-editable default configuration.
# ---------------------------------------------------------------------------
CONFIG_DEFAULTS = {
    "robot_ip": "192.168.1.190",
    "arm": "A",
    "ctrl_hz": 200,
    "vel_ratio": 100,
    "acc_ratio": 100,
    "home_send_hz": 100.0,
    "home_timeout_s": 20.0,
    "home_tol_deg": 1.0,
    "home_stable_samples": 20,
    "pre_wait_s": 5.0,
    "between_trial_wait_s": 2.0,
    "pre_trial_impedance_hold_s": 1.0,
    "result_dir": "results_per_joint_custom_curve_200hz",
    "tag": "PerJoint_CustomCurveTable_200Hz",
    "min_trial_duration_s": 12.0,
    "max_trial_duration_s": 30.0,
    "cycles_per_trial": 4.0,
    "fixed_duration_s": 0.0,  # 0 means auto duration from period/cycles.
}

# Fixed joint impedance / PD parameters for the whole sweep.
# K unit follows the SDK wrapper definition; D unit follows the SDK wrapper definition.
JOINT_IMP_DEFAULT_K = [6.0, 6.0, 6.0, 5.0, 4.5, 4.5, 3.5]
JOINT_IMP_DEFAULT_D = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

# New custom-curve workflow default home.
# The formal trajectory is q = home + A*sin(2*pi*omega_hz*t).
# By default, home = [0, 0, 0, 0, 0, 0, 0] deg.
# This can still be overridden with --home_joints if needed.
ZERO_HOME_JOINTS_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Previous report home is kept only as an optional compatibility home mode.
REPORT_HOME_JOINTS_DEG = [
    math.degrees(1.70),
    math.degrees(-1.10),
    math.degrees(-1.10),
    math.degrees(-2.00),
    math.degrees(-0.37),
    math.degrees(0.13),
    math.degrees(0.55),
]

# Alternative safe/default positions inherited from the existing TEST_SCRIPT.
def default_joints_for_arm(arm: str) -> List[float]:
    if arm == "A":
        return [90.0, -60.0, -90.0, -90.0, 0.0, 0.0, 0.0]
    if arm == "B":
        return [90.0, 60.0, -90.0, -90.0, 0.0, 0.0, 0.0]
    raise ValueError(f"arm must be A or B, got {arm!r}")


JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]

# ---------------------------------------------------------------------------
# User-editable sine-curve table.
# ---------------------------------------------------------------------------
# Each enabled row defines one formal trial:
#     q_cmd(t) = home + amplitude_deg * sin(2*pi*omega_hz*t)
# where omega_hz is cycles per second (Hz), not rad/s.
#
# The script computes the compatible legacy fields used by the existing
# analysis script:
#     target_speed_deg_s = 2*pi*omega_hz*amplitude_deg
#     period_s           = 1/omega_hz
#     peak_acc_deg_s2    = (2*pi*omega_hz)^2 * amplitude_deg
#
# Fill this table with the amplitude/frequency pairs that you have already
# verified to be inside each joint's velocity and acceleration limits.  You can
# also keep rows disabled while drafting by setting enabled=False.
#
# Alternative external JSON format is also supported with --table_json:
#   [
#     {"enabled": true, "test_id": "J1_A10_w0p5", "joint": "J1", "amplitude_deg": 10.0, "omega_hz": 0.5},
#     {"enabled": true, "test_id": "J2_A20_w0p7", "joint": "J2", "amplitude_deg": 20.0, "omega_hz": 0.7}
#   ]
CUSTOM_SINE_TEST_TABLE = [
    # q_cmd(t) = home + A * sin(2*pi*omega_hz*t)
    # omega_hz = target_speed_deg_s / (2*pi*A)

    # J1, A = 90.0 deg
    {"enabled": True, "test_id": "J1_030dps_A90p0", "joint": "J1", "amplitude_deg": 90.0, "omega_hz": 0.053052},
    {"enabled": True, "test_id": "J1_060dps_A90p0", "joint": "J1", "amplitude_deg": 90.0, "omega_hz": 0.106103},
    {"enabled": True, "test_id": "J1_090dps_A90p0", "joint": "J1", "amplitude_deg": 90.0, "omega_hz": 0.159155},
    {"enabled": True, "test_id": "J1_120dps_A90p0", "joint": "J1", "amplitude_deg": 90.0, "omega_hz": 0.212207},
    {"enabled": True, "test_id": "J1_150dps_A90p0", "joint": "J1", "amplitude_deg": 90.0, "omega_hz": 0.265258},
    {"enabled": True, "test_id": "J1_180dps_A90p0", "joint": "J1", "amplitude_deg": 90.0, "omega_hz": 0.318310},

    # J2, A = 90.0 deg
    {"enabled": True, "test_id": "J2_030dps_A90p0", "joint": "J2", "amplitude_deg": 90.0, "omega_hz": 0.053052},
    {"enabled": True, "test_id": "J2_060dps_A90p0", "joint": "J2", "amplitude_deg": 90.0, "omega_hz": 0.106103},
    {"enabled": True, "test_id": "J2_090dps_A90p0", "joint": "J2", "amplitude_deg": 90.0, "omega_hz": 0.159155},
    {"enabled": True, "test_id": "J2_120dps_A90p0", "joint": "J2", "amplitude_deg": 90.0, "omega_hz": 0.212207},
    {"enabled": True, "test_id": "J2_150dps_A90p0", "joint": "J2", "amplitude_deg": 90.0, "omega_hz": 0.265258},
    {"enabled": True, "test_id": "J2_180dps_A90p0", "joint": "J2", "amplitude_deg": 90.0, "omega_hz": 0.318310},

    # J3, A = 40 deg
    {"enabled": True, "test_id": "J3_030dps_A40p0", "joint": "J3", "amplitude_deg": 40.0, "omega_hz": 0.119366},
    {"enabled": True, "test_id": "J3_060dps_A40p0", "joint": "J3", "amplitude_deg": 40.0, "omega_hz": 0.238732},
    {"enabled": True, "test_id": "J3_090dps_A40p0", "joint": "J3", "amplitude_deg": 40.0, "omega_hz": 0.358099},
    {"enabled": True, "test_id": "J3_120dps_A40p0", "joint": "J3", "amplitude_deg": 40.0, "omega_hz": 0.477465},
    {"enabled": True, "test_id": "J3_150dps_A40p0", "joint": "J3", "amplitude_deg": 40.0, "omega_hz": 0.596831},
    {"enabled": True, "test_id": "J3_180dps_A40p0", "joint": "J3", "amplitude_deg": 40.0, "omega_hz": 0.716197},

    # J4, A = 40 deg
    {"enabled": True, "test_id": "J4_030dps_A40p0", "joint": "J4", "amplitude_deg": 40.0, "omega_hz": 0.119366},
    {"enabled": True, "test_id": "J4_060dps_A40p0", "joint": "J4", "amplitude_deg": 40.0, "omega_hz": 0.238732},
    {"enabled": True, "test_id": "J4_090dps_A40p0", "joint": "J4", "amplitude_deg": 40.0, "omega_hz": 0.358099},
    {"enabled": True, "test_id": "J4_120dps_A40p0", "joint": "J4", "amplitude_deg": 40.0, "omega_hz": 0.477465},
    {"enabled": True, "test_id": "J4_150dps_A40p0", "joint": "J4", "amplitude_deg": 40.0, "omega_hz": 0.596831},
    {"enabled": True, "test_id": "J4_180dps_A40p0", "joint": "J4", "amplitude_deg": 40.0, "omega_hz": 0.716197},

    # J5, A = 40 deg
    {"enabled": True, "test_id": "J5_030dps_A40p0", "joint": "J5", "amplitude_deg": 40.0, "omega_hz": 0.119366},
    {"enabled": True, "test_id": "J5_060dps_A40p0", "joint": "J5", "amplitude_deg": 40.0, "omega_hz": 0.238732},
    {"enabled": True, "test_id": "J5_090dps_A40p0", "joint": "J5", "amplitude_deg": 40.0, "omega_hz": 0.358099},
    {"enabled": True, "test_id": "J5_120dps_A40p0", "joint": "J5", "amplitude_deg": 40.0, "omega_hz": 0.477465},
    {"enabled": True, "test_id": "J5_150dps_A40p0", "joint": "J5", "amplitude_deg": 40.0, "omega_hz": 0.596831},
    {"enabled": True, "test_id": "J5_180dps_A40p0", "joint": "J5", "amplitude_deg": 40.0, "omega_hz": 0.716197},

    # J6, A = 40 deg
    {"enabled": True, "test_id": "J6_030dps_A40p0", "joint": "J6", "amplitude_deg": 40.0, "omega_hz": 0.119366},
    {"enabled": True, "test_id": "J6_060dps_A40p0", "joint": "J6", "amplitude_deg": 40.0, "omega_hz": 0.238732},
    {"enabled": True, "test_id": "J6_090dps_A40p0", "joint": "J6", "amplitude_deg": 40.0, "omega_hz": 0.358099},
    {"enabled": True, "test_id": "J6_120dps_A40p0", "joint": "J6", "amplitude_deg": 40.0, "omega_hz": 0.477465},
    {"enabled": True, "test_id": "J6_150dps_A40p0", "joint": "J6", "amplitude_deg": 40.0, "omega_hz": 0.596831},
    {"enabled": True, "test_id": "J6_180dps_A40p0", "joint": "J6", "amplitude_deg": 40.0, "omega_hz": 0.716197},

    # J7, A = 50 deg
    {"enabled": True, "test_id": "J7_030dps_A50p0", "joint": "J7", "amplitude_deg": 50.0, "omega_hz": 0.095493},
    {"enabled": True, "test_id": "J7_060dps_A50p0", "joint": "J7", "amplitude_deg": 50.0, "omega_hz": 0.190986},
    {"enabled": True, "test_id": "J7_090dps_A50p0", "joint": "J7", "amplitude_deg": 50.0, "omega_hz": 0.286479},
    {"enabled": True, "test_id": "J7_120dps_A50p0", "joint": "J7", "amplitude_deg": 50.0, "omega_hz": 0.381972},
    {"enabled": True, "test_id": "J7_150dps_A50p0", "joint": "J7", "amplitude_deg": 50.0, "omega_hz": 0.477465},
    {"enabled": True, "test_id": "J7_180dps_A50p0", "joint": "J7", "amplitude_deg": 50.0, "omega_hz": 0.572958},
]

# User-provided joint vel acc limits.
VEL_LIMIT_DEG_S = [181.0] * 7
ACC_LIMIT_DEG_S2 = [901.0, 901.0, 1801.0, 1801.0, 1801.0, 1801.0, 1801.0]

# Compatibility fields for the previous analysis/report code.
# For this custom-table workflow, feedback-watch rows are optional per table row.
FEEDBACK_WATCH_GROUPS = set()
HISTORICAL_FEEDBACK_PEAK_VEL_DEG_S = {}

TEST_EQUIPMENT = {
    "robot": "Marvin M6",
    "host_pc": platform.platform(),
    "runtime": f"Python {platform.python_version()} with SDK_PYTHON",
    "control_frequency_hz": 200,
}


# ---------------------------------------------------------------------------
# Utility data classes.
# ---------------------------------------------------------------------------
@dataclass
class TestGroup:
    joint_index: int
    joint_name: str
    amplitude_deg: float
    # omega_hz means the frequency term in q = home + A*sin(2*pi*omega_hz*t).
    omega_hz: float
    test_id: str
    target_speed_deg_s: float
    period_s: float
    theoretical_peak_vel_deg_s: float
    theoretical_peak_acc_deg_s2: float
    vel_limit_deg_s: float
    acc_limit_deg_s2: float
    precheck_result: str
    skip_reason: str
    feedback_watch: bool
    historical_feedback_peak_vel_deg_s: Optional[float]


# ---------------------------------------------------------------------------
# Timing helpers copied/adapted from the existing TEST_SCRIPT.
# ---------------------------------------------------------------------------
class WindowsHighPrecisionTimer:
    def __init__(self, period_ms: int = 1):
        self.period_ms = int(period_ms)
        self._winmm = None
        self.enabled = False

    def start(self) -> None:
        if os.name != "nt":
            return
        try:
            self._winmm = ctypes.WinDLL("winmm")
            ret = self._winmm.timeBeginPeriod(self.period_ms)
            self.enabled = ret == 0
            if not self.enabled:
                print(f"[warn] timeBeginPeriod({self.period_ms}) failed: ret={ret}")
        except Exception as exc:
            self.enabled = False
            print(f"[warn] timeBeginPeriod unavailable: {exc}")

    def stop(self) -> None:
        if os.name != "nt":
            return
        if self.enabled and self._winmm is not None:
            try:
                self._winmm.timeEndPeriod(self.period_ms)
            except Exception as exc:
                print(f"[warn] timeEndPeriod failed: {exc}")
        self.enabled = False


def sleep_until(deadline_s: float, spin_threshold_s: float = 0.0005) -> None:
    """Sleep until an absolute perf_counter deadline, then busy-wait the last few us."""
    while True:
        now = time.perf_counter()
        remain = deadline_s - now
        if remain <= 0.0:
            return
        if remain > spin_threshold_s:
            time.sleep(remain - spin_threshold_s)


def percentile(values: Sequence[float], pct: float) -> Optional[float]:
    if not values:
        return None
    arr = sorted(float(v) for v in values)
    if len(arr) == 1:
        return arr[0]
    k = (len(arr) - 1) * float(pct) / 100.0
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return arr[lo]
    return arr[lo] + (arr[hi] - arr[lo]) * (k - lo)


def stats_ms(samples_s: Sequence[float]) -> Dict[str, Optional[float]]:
    if not samples_s:
        return {
            "count": 0,
            "mean_ms": None,
            "std_ms": None,
            "min_ms": None,
            "max_ms": None,
            "p95_ms": None,
        }
    arr_ms = [float(v) * 1000.0 for v in samples_s]
    return {
        "count": len(arr_ms),
        "mean_ms": float(statistics.fmean(arr_ms)),
        "std_ms": float(statistics.pstdev(arr_ms)) if len(arr_ms) > 1 else 0.0,
        "min_ms": float(min(arr_ms)),
        "max_ms": float(max(arr_ms)),
        "p95_ms": percentile(arr_ms, 95.0),
    }


def safe_float(value: object) -> Optional[float]:
    try:
        f = float(value)
        if math.isfinite(f):
            return f
    except Exception:
        pass
    return None


# ---------------------------------------------------------------------------
# Robot interface helpers copied/adapted from the existing TEST_SCRIPT.
# ---------------------------------------------------------------------------
def arm_to_index(arm: str) -> int:
    if arm == "A":
        return 0
    if arm == "B":
        return 1
    raise ValueError(f"arm must be A or B, got {arm!r}")


def send_joint_command(robot: Marvin_Robot, arm: str, joints_deg: Sequence[float]) -> None:
    if len(joints_deg) != 7:
        raise ValueError("joints_deg must have 7 values")
    robot.clear_set()
    robot.set_joint_cmd_pose(arm=arm, joints=[float(v) for v in joints_deg])
    robot.send_cmd()


def safe_connect_and_clear(robot: Marvin_Robot, dcss: DCSS, robot_ip: str, arm: str) -> None:
    if not robot.connect(robot_ip):
        raise RuntimeError("Robot connect failed: port occupied or network issue")

    time.sleep(0.5)
    robot.clear_set()
    robot.clear_error("A")
    robot.clear_error("B")
    robot.send_cmd()
    time.sleep(0.5)

    try:
        robot.log_switch("0")
    except Exception:
        pass
    try:
        robot.local_log_switch("0")
    except Exception:
        pass

    idx = arm_to_index(arm)
    motion_tag = 0
    frame_update = None
    for _ in range(10):
        sub_data = robot.subscribe(dcss)
        if not sub_data:
            time.sleep(0.1)
            continue
        frame_serial = _optional_int(sub_data["outputs"][idx].get("frame_serial"), 0)
        print(f"connect frames: {frame_serial}")
        if frame_serial != 0 and frame_serial != frame_update:
            motion_tag += 1
            frame_update = frame_serial
        time.sleep(0.1)

    if motion_tag <= 0:
        raise RuntimeError("Connected but no valid frame update received")


def enter_position_mode(robot: Marvin_Robot, arm: str, vel_ratio: int, acc_ratio: int) -> None:
    robot.clear_set()
    robot.set_vel_acc(arm=arm, velRatio=int(vel_ratio), AccRatio=int(acc_ratio))
    robot.send_cmd()
    time.sleep(0.2)

    robot.clear_set()
    robot.set_state(arm=arm, state=1)
    robot.send_cmd()
    time.sleep(0.2)


def enter_joint_impedance_mode(
    robot: Marvin_Robot,
    arm: str,
    vel_ratio: int,
    acc_ratio: int,
    joint_k: Sequence[float],
    joint_d: Sequence[float],
    ctrl_hz: int,
) -> None:
    if len(joint_k) != 7 or len(joint_d) != 7:
        raise ValueError("joint_k and joint_d must each have 7 values")

    robot.clear_set()
    robot.set_vel_acc(arm=arm, velRatio=int(vel_ratio), AccRatio=int(acc_ratio))
    if hasattr(robot, "set_joint_kd_params"):
        ok = robot.set_joint_kd_params(
            arm=arm,
            K=[float(v) for v in joint_k],
            D=[float(v) for v in joint_d],
        )
        print(f"[mode] set_joint_kd_params -> {ok}")
    else:
        print("[warn] set_joint_kd_params not available, continue with state switch only")
    robot.send_cmd()
    time.sleep(0.2)

    robot.clear_set()
    robot.set_state(arm=arm, state=3)  # torque state; required for impedance mode by SDK comments.

    if hasattr(robot, "set_impedance_type"):
        try:
            robot.set_impedance_type(arm=arm, type=1)  # 1 = joint impedance.
        except TypeError:
            robot.set_impedance_type(arm, 1)

    vel_est_step_ms = max(1, int(round(1000.0 / max(float(ctrl_hz), 1.0))))
    if hasattr(robot, "set_vel_est_step"):
        vel_est_ok = True
        try:
            vel_est_ok = bool(robot.set_vel_est_step(arm=arm, time=vel_est_step_ms))
        except TypeError:
            try:
                vel_est_ok = bool(robot.set_vel_est_step(arm, vel_est_step_ms))
            except Exception:
                vel_est_ok = False
        except Exception:
            vel_est_ok = False
        if not vel_est_ok:
            print(f"[warn] set_vel_est_step failed (arm={arm}, time_ms={vel_est_step_ms})")
    else:
        print("[warn] set_vel_est_step not available in current SDK wrapper")

    robot.send_cmd()
    time.sleep(0.2)


def get_current_joints(robot: Marvin_Robot, dcss: DCSS, arm: str) -> List[float]:
    idx = arm_to_index(arm)
    sub_data = robot.subscribe(dcss)
    if not sub_data:
        raise RuntimeError("subscribe failed while reading current joints")
    q = sub_data["outputs"][idx]["fb_joint_pos"]
    if q is None or len(q) != 7:
        raise RuntimeError("invalid fb_joint_pos from subscribe")
    return [float(v) for v in q]


def move_to_joint_target_with_settle(
    robot: Marvin_Robot,
    dcss: DCSS,
    arm: str,
    target_joints_deg: Sequence[float],
    send_hz: float,
    timeout_s: float,
    tol_deg: float,
    stable_samples: int,
) -> None:
    if len(target_joints_deg) != 7:
        raise ValueError("target_joints_deg must have 7 values")

    period_s = 1.0 / max(float(send_hz), 1.0)
    deadline = time.perf_counter() + float(timeout_s)
    stable_count = 0
    last_err = None

    while time.perf_counter() < deadline:
        send_joint_command(robot, arm, target_joints_deg)

        q_fb = get_current_joints(robot, dcss, arm)
        last_err = max(abs(float(a) - float(b)) for a, b in zip(q_fb, target_joints_deg))

        if last_err <= float(tol_deg):
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= int(max(1, stable_samples)):
            print(f"[home] reached target joints, max|error|={last_err:.3f} deg")
            return

        time.sleep(period_s)

    raise RuntimeError(
        f"Move to target joints timeout ({timeout_s:.1f}s), "
        f"last max|error|={float(last_err) if last_err is not None else float('nan'):.3f} deg"
    )


def best_effort_servo_off_and_release(robot: Marvin_Robot, arm: str) -> None:
    try:
        robot.clear_set()
        robot.set_state(arm=arm, state=0)
        robot.send_cmd()
        time.sleep(0.2)
    except Exception as exc:
        print(f"[shutdown] set_state(0) failed: {exc}")

    try:
        if hasattr(robot, "disable"):
            disable_ok = robot.disable(arm)
            print(f"[shutdown] disable({arm}) -> {disable_ok}")
        else:
            print("[shutdown] disable API not available in current SDK wrapper, skip")
    except Exception as exc:
        print(f"[shutdown] disable failed: {exc}")

    try:
        robot.release_robot()
    except Exception as exc:
        print(f"[shutdown] release_robot failed: {exc}")


# ---------------------------------------------------------------------------
# Test planning and theoretical validity checking.
# ---------------------------------------------------------------------------
def _parse_joint_index(value: object) -> int:
    """Parse joint from J1..J7 or 1..7 into zero-based index."""
    if isinstance(value, str):
        text = value.strip().upper()
        if text.startswith("J"):
            text = text[1:]
        idx = int(text) - 1
    else:
        idx = int(value) - 1
    if idx < 0 or idx >= 7:
        raise ValueError(f"Invalid joint value: {value!r}; expected J1..J7 or 1..7")
    return idx


def _table_row_enabled(row: Dict[str, object]) -> bool:
    return bool(row.get("enabled", True))


def load_curve_table(table_json_path: Optional[str]) -> List[Dict[str, object]]:
    """Load custom sine table from JSON or from the in-script table."""
    if table_json_path:
        with open(table_json_path, "r", encoding="utf-8-sig") as f:
            obj = json.load(f)
        if isinstance(obj, dict):
            obj = obj.get("tests", obj.get("table", obj.get("rows")))
        if not isinstance(obj, list):
            raise ValueError("--table_json must contain a list or a dict with tests/table/rows list")
        return [dict(row) for row in obj]
    return [dict(row) for row in CUSTOM_SINE_TEST_TABLE]


def build_test_group_from_row(row: Dict[str, object], row_index: int) -> TestGroup:
    """Build one TestGroup from a custom table row.

    Required row fields:
        joint: "J1".."J7" or 1..7
        amplitude_deg: sine amplitude in deg
        omega_hz: frequency in cycles/s for q = home + A*sin(2*pi*omega_hz*t)

    Compatibility aliases:
        frequency_hz or freq_hz may be used instead of omega_hz.
        If target_speed_deg_s is supplied instead of omega_hz, omega_hz is derived.
    """
    if not _table_row_enabled(row):
        raise ValueError("disabled row should have been filtered before build_test_group_from_row")

    joint_index = _parse_joint_index(row.get("joint", row.get("joint_index", row.get("joint_name"))))
    joint_name = JOINT_NAMES[joint_index]

    amplitude_deg = float(row.get("amplitude_deg", row.get("A_deg")))
    if amplitude_deg <= 0.0:
        raise ValueError(f"amplitude_deg must be > 0 for row {row_index}: {row!r}")

    omega_value = row.get("omega_hz", row.get("frequency_hz", row.get("freq_hz")))
    if omega_value is None:
        # Allow a legacy/derived specification by target peak speed.
        speed_value = row.get("target_speed_deg_s", row.get("speed_deg_s"))
        if speed_value is None:
            raise ValueError(f"row {row_index} needs omega_hz/frequency_hz/freq_hz or target_speed_deg_s")
        target_speed = float(speed_value)
        if target_speed <= 0.0:
            raise ValueError(f"target_speed_deg_s must be > 0 for row {row_index}")
        omega_hz = target_speed / (2.0 * math.pi * amplitude_deg)
    else:
        omega_hz = float(omega_value)
        if omega_hz <= 0.0:
            raise ValueError(f"omega_hz must be > 0 for row {row_index}: {row!r}")
        target_speed = 2.0 * math.pi * omega_hz * amplitude_deg

    period_s = 1.0 / omega_hz
    theoretical_peak_vel = target_speed
    theoretical_peak_acc = (2.0 * math.pi * omega_hz) ** 2 * amplitude_deg
    vel_limit = float(VEL_LIMIT_DEG_S[joint_index])
    acc_limit = float(ACC_LIMIT_DEG_S2[joint_index])

    skip_reasons: List[str] = []
    if theoretical_peak_vel > vel_limit + 1e-9:
        skip_reasons.append(
            f"SKIP_VEL: v_max=2*pi*omega*A=2*pi*{omega_hz:.6f}*{amplitude_deg:.3f}="
            f"{theoretical_peak_vel:.3f} deg/s > vel_limit={vel_limit:.3f} deg/s"
        )
    if theoretical_peak_acc > acc_limit + 1e-9:
        skip_reasons.append(
            f"SKIP_ACC: a_max=(2*pi*omega)^2*A=(2*pi*{omega_hz:.6f})^2*{amplitude_deg:.3f}="
            f"{theoretical_peak_acc:.3f} deg/s^2 > acc_limit={acc_limit:.3f} deg/s^2"
        )

    feedback_watch = bool(row.get("feedback_watch", False))
    historical_peak = safe_float(row.get("historical_feedback_peak_vel_deg_s"))

    if skip_reasons:
        precheck = "+".join(reason.split(":", 1)[0] for reason in skip_reasons)
    elif feedback_watch:
        precheck = "PASS_FEEDBACK_WATCH"
    else:
        precheck = "PASS"

    test_id = str(row.get("test_id", f"{joint_name}_A{amplitude_deg:g}_w{omega_hz:g}"))

    return TestGroup(
        joint_index=joint_index,
        joint_name=joint_name,
        amplitude_deg=amplitude_deg,
        omega_hz=omega_hz,
        test_id=test_id,
        target_speed_deg_s=theoretical_peak_vel,
        period_s=period_s,
        theoretical_peak_vel_deg_s=theoretical_peak_vel,
        theoretical_peak_acc_deg_s2=theoretical_peak_acc,
        vel_limit_deg_s=vel_limit,
        acc_limit_deg_s2=acc_limit,
        precheck_result=precheck,
        skip_reason=" | ".join(skip_reasons),
        feedback_watch=feedback_watch,
        historical_feedback_peak_vel_deg_s=historical_peak,
    )


def normalize_joint_filter(joint_filter: Optional[Sequence[str]]) -> Optional[set[int]]:
    if not joint_filter:
        return None
    return {_parse_joint_index(item) for item in joint_filter}


def normalize_speed_filter(speed_filter: Optional[Sequence[float]]) -> Optional[set[float]]:
    if not speed_filter:
        return None
    return {float(v) for v in speed_filter}


def build_test_plan(
    joint_filter: Optional[Sequence[str]],
    speed_filter: Optional[Sequence[float]],
    table_json_path: Optional[str] = None,
) -> List[TestGroup]:
    joints = normalize_joint_filter(joint_filter)
    speeds = normalize_speed_filter(speed_filter)
    groups: List[TestGroup] = []
    table = load_curve_table(table_json_path)
    for row_index, row in enumerate(table, start=1):
        if not _table_row_enabled(row):
            continue
        group = build_test_group_from_row(row, row_index)
        if joints is not None and group.joint_index not in joints:
            continue
        if speeds is not None and not any(abs(group.target_speed_deg_s - speed) < 1e-6 for speed in speeds):
            continue
        groups.append(group)
    return groups


def trial_duration_s(group: TestGroup, args: argparse.Namespace) -> float:
    min_duration_by_cycles = 3.0 * float(group.period_s)
    fixed = float(args.fixed_duration_s)
    if fixed > 0.0:
        return max(fixed, min_duration_by_cycles)
    desired = float(args.cycles_per_trial) * float(group.period_s)
    desired = max(float(args.min_trial_duration_s), desired)
    desired = min(float(args.max_trial_duration_s), desired)
    desired = max(desired, min_duration_by_cycles)
    return desired


def build_sine_command(
    home_joints_deg: Sequence[float],
    group: TestGroup,
    t_s: float,
    fade_in_s: float = 0.0,
) -> Tuple[List[float], List[float], List[float]]:
    q = [float(v) for v in home_joints_deg]
    dq = [0.0] * 7
    ddq = [0.0] * 7

    j = int(group.joint_index)
    amp = float(group.amplitude_deg)
    omega = 2.0 * math.pi / float(group.period_s)
    phase = omega * float(t_s)

    gain = 1.0
    gain_dot = 0.0
    gain_ddot = 0.0
    if fade_in_s > 0.0 and t_s < fade_in_s:
        # Smoothstep envelope g=3x^2-2x^3.  This is optional and disabled by default
        # because it changes the exact theoretical sine at the beginning of the trial.
        x = max(0.0, min(1.0, float(t_s) / float(fade_in_s)))
        gain = 3.0 * x * x - 2.0 * x * x * x
        gain_dot = (6.0 * x - 6.0 * x * x) / float(fade_in_s)
        gain_ddot = (6.0 - 12.0 * x) / (float(fade_in_s) ** 2)

    sin_p = math.sin(phase)
    cos_p = math.cos(phase)

    # q = home + g*A*sin(wt)
    # dq = A*(g_dot*sin(wt) + g*w*cos(wt))
    # ddq = A*(g_ddot*sin(wt) + 2*g_dot*w*cos(wt) - g*w^2*sin(wt))
    q[j] += gain * amp * sin_p
    dq[j] = amp * (gain_dot * sin_p + gain * omega * cos_p)
    ddq[j] = amp * (gain_ddot * sin_p + 2.0 * gain_dot * omega * cos_p - gain * omega * omega * sin_p)
    return q, dq, ddq


def hold_joint_target_in_impedance_mode(
    robot: Marvin_Robot,
    dcss: DCSS,
    arm: str,
    target_joints_deg: Sequence[float],
    hold_s: float,
    ctrl_hz: int,
    spin_threshold_s: float = 0.0005,
) -> None:
    """Hold the impedance-mode joint target at home before starting t=0.

    This makes the first formal sine sample unambiguous: after entering joint
    impedance mode, the controller receives the home target for a short period,
    then the logged trajectory starts with q_cmd(t=0) = home + A*sin(0) = home.
    Samples during this hold are intentionally not written to the trial CSV/NPZ.
    """
    hold_s = float(hold_s)
    if hold_s <= 0.0:
        return
    if len(target_joints_deg) != 7:
        raise ValueError("target_joints_deg must have 7 values")

    hz = max(1, int(ctrl_hz))
    period_s = 1.0 / float(hz)
    points = max(1, int(round(hold_s * float(hz))))

    print(f"[trial] impedance home hold: {points} samples @ {hz} Hz ({points / float(hz):.3f}s)")
    next_deadline = time.perf_counter()
    for _ in range(points):
        now = time.perf_counter()
        if now < next_deadline:
            sleep_until(next_deadline, spin_threshold_s=float(spin_threshold_s))
        send_joint_command(robot, arm, target_joints_deg)
        try:
            robot.subscribe(dcss)
        except Exception:
            pass
        next_deadline += period_s


# ---------------------------------------------------------------------------
# Data logging helpers.
# ---------------------------------------------------------------------------
def vector_columns(prefix: str, unit: str = "") -> List[str]:
    suffix = f"_{unit}" if unit else ""
    return [f"{prefix}_{name}{suffix}" for name in JOINT_NAMES]


def sample_csv_header() -> List[str]:
    return (
        [
            "sample_index",
            "joint_name",
            "target_speed_deg_s",
            "amplitude_deg",
            "period_s",
            "t_plan_s",
            "t_send_s",
            "fb_read_time_s",
            "command_dt_ms",
            "command_jitter_ms",
            "overrun_flag",
            "frame_serial",
            "traj_state",
            "cur_state",
            "cmd_state",
            "err_code",
        ]
        + vector_columns("q_cmd", "deg")
        + vector_columns("dq_cmd", "deg_s")
        + vector_columns("ddq_cmd", "deg_s2")
        + vector_columns("q_fb", "deg")
        + vector_columns("dq_fb", "deg_s")
        + vector_columns("q_fb_cmd", "deg")
        + vector_columns("torque_cToq", "Nm")
        + vector_columns("torque_sToq", "Nm")
        + vector_columns("torque_them", "Nm")
    )


def empty_joint_vector() -> List[Optional[float]]:
    return [None] * 7


def get_output_vector(output: Dict[str, object], key: str) -> List[Optional[float]]:
    raw = output.get(key) if isinstance(output, dict) else None
    if raw is None or len(raw) != 7:
        return empty_joint_vector()
    return [safe_float(v) for v in raw]


def _optional_int(value: object, default: int = -1) -> int:
    """Best-effort integer conversion for SDK/DCSS status fields.

    Some SDK fields can arrive as normal Python ints, numeric strings, or
    one-byte values such as b"\x00". Direct int(b"\x00") raises ValueError,
    so convert those compact byte status fields safely.
    """
    if value is None:
        return int(default)
    try:
        if isinstance(value, (bytes, bytearray)):
            raw = bytes(value)
            if len(raw) == 0:
                return int(default)
            if len(raw) == 1:
                return int(raw[0])
            try:
                text = raw.decode("ascii", errors="ignore").strip("\x00 \t\r\n")
                if text:
                    return int(text)
            except Exception:
                pass
            return int.from_bytes(raw, byteorder="little", signed=False)
        return int(value)
    except Exception:
        return int(default)


@dataclass
class TrialBuffer:
    """Preallocated in-memory sample buffer.

    The real-time command loop writes only to these NumPy arrays.  Disk output
    is deferred until the trial finishes, avoiding per-frame CSV I/O jitter.
    """

    max_samples: int
    dof: int = 7

    def __post_init__(self) -> None:
        n = int(self.max_samples)
        d = int(self.dof)
        if n <= 0:
            raise ValueError("max_samples must be > 0")
        if d <= 0:
            raise ValueError("dof must be > 0")

        self.count = 0
        self.sample_index = np.full(n, -1, dtype=np.int32)

        self.t_plan_s = np.full(n, np.nan, dtype=np.float64)
        self.t_send_s = np.full(n, np.nan, dtype=np.float64)
        self.fb_read_time_s = np.full(n, np.nan, dtype=np.float64)
        self.command_dt_ms = np.full(n, np.nan, dtype=np.float64)
        self.command_jitter_ms = np.full(n, np.nan, dtype=np.float64)
        self.overrun_flag = np.zeros(n, dtype=np.int8)

        self.frame_serial = np.full(n, -1, dtype=np.int64)
        self.traj_state = np.full(n, -1, dtype=np.int32)
        self.cur_state = np.full(n, -1, dtype=np.int32)
        self.cmd_state = np.full(n, -1, dtype=np.int32)
        self.err_code = np.full(n, -1, dtype=np.int32)

        self.q_cmd = np.full((n, d), np.nan, dtype=np.float64)
        self.dq_cmd = np.full((n, d), np.nan, dtype=np.float64)
        self.ddq_cmd = np.full((n, d), np.nan, dtype=np.float64)
        self.q_fb = np.full((n, d), np.nan, dtype=np.float64)
        self.dq_fb = np.full((n, d), np.nan, dtype=np.float64)
        self.q_fb_cmd = np.full((n, d), np.nan, dtype=np.float64)
        self.torque_cToq = np.full((n, d), np.nan, dtype=np.float64)
        self.torque_sToq = np.full((n, d), np.nan, dtype=np.float64)
        self.torque_them = np.full((n, d), np.nan, dtype=np.float64)

    @staticmethod
    def _vec(values: Sequence[object], dof: int) -> np.ndarray:
        out = np.full(dof, np.nan, dtype=np.float64)
        for idx, value in enumerate(values[:dof]):
            f = safe_float(value)
            if f is not None:
                out[idx] = f
        return out

    def add(
        self,
        sample_index: int,
        t_plan_s: float,
        t_send_s: float,
        fb_read_time_s: Optional[float],
        command_dt_ms: Optional[float],
        command_jitter_ms: Optional[float],
        overrun_flag: int,
        frame_serial: Optional[int],
        traj_state: Optional[int],
        cur_state: Optional[int],
        cmd_state: Optional[int],
        err_code: Optional[int],
        q_cmd: Sequence[object],
        dq_cmd: Sequence[object],
        ddq_cmd: Sequence[object],
        q_fb: Sequence[object],
        dq_fb: Sequence[object],
        q_fb_cmd: Sequence[object],
        ctoq: Sequence[object],
        stoq: Sequence[object],
        them: Sequence[object],
    ) -> None:
        if self.count >= self.max_samples:
            raise RuntimeError("TrialBuffer overflow")

        i = self.count
        d = self.dof

        self.sample_index[i] = int(sample_index)
        self.t_plan_s[i] = float(t_plan_s)
        self.t_send_s[i] = float(t_send_s)
        self.fb_read_time_s[i] = np.nan if fb_read_time_s is None else float(fb_read_time_s)
        self.command_dt_ms[i] = np.nan if command_dt_ms is None else float(command_dt_ms)
        self.command_jitter_ms[i] = np.nan if command_jitter_ms is None else float(command_jitter_ms)
        self.overrun_flag[i] = int(overrun_flag)

        self.frame_serial[i] = _optional_int(frame_serial, -1)
        self.traj_state[i] = _optional_int(traj_state, -1)
        self.cur_state[i] = _optional_int(cur_state, -1)
        self.cmd_state[i] = _optional_int(cmd_state, -1)
        self.err_code[i] = _optional_int(err_code, -1)

        self.q_cmd[i, :] = self._vec(q_cmd, d)
        self.dq_cmd[i, :] = self._vec(dq_cmd, d)
        self.ddq_cmd[i, :] = self._vec(ddq_cmd, d)
        self.q_fb[i, :] = self._vec(q_fb, d)
        self.dq_fb[i, :] = self._vec(dq_fb, d)
        self.q_fb_cmd[i, :] = self._vec(q_fb_cmd, d)
        self.torque_cToq[i, :] = self._vec(ctoq, d)
        self.torque_sToq[i, :] = self._vec(stoq, d)
        self.torque_them[i, :] = self._vec(them, d)

        self.count += 1

    def view(self) -> Dict[str, np.ndarray]:
        n = int(self.count)
        return {
            "sample_index": self.sample_index[:n],
            "t_plan_s": self.t_plan_s[:n],
            "t_send_s": self.t_send_s[:n],
            "fb_read_time_s": self.fb_read_time_s[:n],
            "command_dt_ms": self.command_dt_ms[:n],
            "command_jitter_ms": self.command_jitter_ms[:n],
            "overrun_flag": self.overrun_flag[:n],
            "frame_serial": self.frame_serial[:n],
            "traj_state": self.traj_state[:n],
            "cur_state": self.cur_state[:n],
            "cmd_state": self.cmd_state[:n],
            "err_code": self.err_code[:n],
            "q_cmd": self.q_cmd[:n],
            "dq_cmd": self.dq_cmd[:n],
            "ddq_cmd": self.ddq_cmd[:n],
            "q_fb": self.q_fb[:n],
            "dq_fb": self.dq_fb[:n],
            "q_fb_cmd": self.q_fb_cmd[:n],
            "torque_cToq": self.torque_cToq[:n],
            "torque_sToq": self.torque_sToq[:n],
            "torque_them": self.torque_them[:n],
        }


def save_trial_buffer(
    buffer: TrialBuffer,
    sample_npz_path: str,
    sample_csv_path: str,
    group: TestGroup,
) -> None:
    """Persist one completed trial.

    The NPZ file is the primary numeric record.  The CSV is generated after the
    real-time loop for easy inspection and report tooling.
    """

    data = buffer.view()

    np.savez_compressed(
        sample_npz_path,
        **data,
        joint_index=np.array(group.joint_index, dtype=np.int32),
        joint_name=np.array(group.joint_name),
        target_speed_deg_s=np.array(group.target_speed_deg_s, dtype=np.float64),
        amplitude_deg=np.array(group.amplitude_deg, dtype=np.float64),
        omega_hz=np.array(group.omega_hz, dtype=np.float64),
        test_id=np.array(group.test_id),
        period_s=np.array(group.period_s, dtype=np.float64),
    )

    with open(sample_csv_path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(sample_csv_header())

        n = int(buffer.count)
        for i in range(n):
            row = [
                int(data["sample_index"][i]),
                group.joint_name,
                float(group.target_speed_deg_s),
                float(group.amplitude_deg),
                float(group.period_s),
                float(data["t_plan_s"][i]),
                float(data["t_send_s"][i]),
                float(data["fb_read_time_s"][i]) if np.isfinite(data["fb_read_time_s"][i]) else None,
                float(data["command_dt_ms"][i]) if np.isfinite(data["command_dt_ms"][i]) else None,
                float(data["command_jitter_ms"][i]) if np.isfinite(data["command_jitter_ms"][i]) else None,
                int(data["overrun_flag"][i]),
                None if int(data["frame_serial"][i]) < 0 else int(data["frame_serial"][i]),
                None if int(data["traj_state"][i]) < 0 else int(data["traj_state"][i]),
                None if int(data["cur_state"][i]) < 0 else int(data["cur_state"][i]),
                None if int(data["cmd_state"][i]) < 0 else int(data["cmd_state"][i]),
                None if int(data["err_code"][i]) < 0 else int(data["err_code"][i]),
            ]

            for key in (
                "q_cmd",
                "dq_cmd",
                "ddq_cmd",
                "q_fb",
                "dq_fb",
                "q_fb_cmd",
                "torque_cToq",
                "torque_sToq",
                "torque_them",
            ):
                row.extend(
                    None if not np.isfinite(value) else float(value)
                    for value in data[key][i, :]
                )

            writer.writerow(row)


def write_precheck_csv(path: str, groups: Sequence[TestGroup]) -> None:
    fieldnames = [f.name for f in fields(TestGroup)]
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for g in groups:
            writer.writerow(asdict(g))


def write_summary_csv(path: str, rows: Sequence[Dict[str, object]]) -> None:
    if not rows:
        return
    fieldnames: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def json_dump(path: str, obj: object) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)


# ---------------------------------------------------------------------------
# Trial execution.
# ---------------------------------------------------------------------------
def execute_one_valid_group(
    robot: Marvin_Robot,
    dcss: DCSS,
    args: argparse.Namespace,
    group: TestGroup,
    trial_index: int,
    total_trials: int,
    home_joints_deg: Sequence[float],
    result_dir: str,
) -> Dict[str, object]:
    arm_idx = arm_to_index(args.arm)
    ctrl_hz = int(args.ctrl_hz)
    period_s = 1.0 / float(ctrl_hz)
    duration = trial_duration_s(group, args)
    total_points = max(1, int(round(duration * float(ctrl_hz))))
    duration = total_points / float(ctrl_hz)

    speed_tag = f"{group.target_speed_deg_s:.1f}".replace(".", "p")
    amp_tag = f"{group.amplitude_deg:.1f}".replace(".", "p")
    trial_tag = f"{trial_index:03d}_{group.joint_name}_{speed_tag}dps_A{amp_tag}deg"
    sample_csv_path = os.path.join(result_dir, f"{trial_tag}_samples.csv")
    sample_npz_path = os.path.join(result_dir, f"{trial_tag}_samples.npz")
    trial_json_path = os.path.join(result_dir, f"{trial_tag}_meta.json")

    print(
        f"[trial {trial_index}/{total_trials}] execute {group.joint_name} "
        f"speed={group.target_speed_deg_s:.1f} deg/s, A={group.amplitude_deg:.1f} deg, "
        f"omega={group.omega_hz:.6f} Hz, T={group.period_s:.3f}s, duration={duration:.3f}s"
    )
    if group.feedback_watch:
        print(
            f"[trial {trial_index}] feedback-watch group: historical peak fb velocity "
            f"{group.historical_feedback_peak_vel_deg_s} deg/s > {group.vel_limit_deg_s} deg/s"
        )

    # Always re-enter joint impedance mode before a formal trial, so each trial starts from
    # a known control state and fixed K/D values.
    enter_joint_impedance_mode(
        robot=robot,
        arm=args.arm,
        vel_ratio=int(args.vel_ratio),
        acc_ratio=int(args.acc_ratio),
        joint_k=[float(v) for v in args.joint_k],
        joint_d=[float(v) for v in args.joint_d],
        ctrl_hz=ctrl_hz,
    )

    timing_tuner = PlatformTimingTuner(
        period_ms=1,
        linux_timer_slack_ns=int(args.linux_timer_slack_ns),
        linux_rt_priority=int(args.linux_rt_priority),
        cpu_affinity=args.cpu_affinity,
    )
    timing_tuner.start()
    high_precision_timer_enabled = bool(timing_tuner.enabled)
    platform_timing = timing_tuner.summary()

    # After switching into joint impedance mode, explicitly hold the PD target
    # at home before starting the logged sine trajectory.  Without this, the
    # first logged sample is still home mathematically, but the controller may
    # have only just switched modes and may not have settled to a home target
    # inside impedance mode.
    hold_joint_target_in_impedance_mode(
        robot=robot,
        dcss=dcss,
        arm=args.arm,
        target_joints_deg=home_joints_deg,
        hold_s=float(args.pre_trial_impedance_hold_s),
        ctrl_hz=ctrl_hz,
        spin_threshold_s=float(args.spin_threshold_s),
    )

    cmd_dt_samples_s: List[float] = []
    jitter_samples_s: List[float] = []
    prev_send_ts: Optional[float] = None
    overrun_count = 0
    negative_sleep_count = 0
    peak_fb_vel_abs = [0.0] * 7
    peak_fb_torque_abs = [0.0] * 7
    peak_cmd_vel_abs = [0.0] * 7
    peak_cmd_acc_abs = [0.0] * 7
    feedback_sample_count = 0
    first_subscribe_warning_printed = False
    start_wall = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    t0 = time.perf_counter()
    next_deadline = t0
    trial_buffer = TrialBuffer(max_samples=total_points, dof=7)

    try:
        for i in range(total_points):
            t_plan = float(i) / float(ctrl_hz)
            q_cmd, dq_cmd, ddq_cmd = build_sine_command(
                home_joints_deg=home_joints_deg,
                group=group,
                t_s=t_plan,
                fade_in_s=float(args.fade_in_s),
            )

            now = time.perf_counter()
            overrun_flag = 0
            if now < next_deadline:
                sleep_until(next_deadline, spin_threshold_s=float(args.spin_threshold_s))
            else:
                negative_sleep_count += 1
                if (now - next_deadline) >= period_s:
                    overrun_count += 1
                    overrun_flag = 1

            send_joint_command(robot, args.arm, q_cmd)
            send_ts = time.perf_counter()
            if prev_send_ts is not None:
                cmd_dt_samples_s.append(send_ts - prev_send_ts)
            prev_send_ts = send_ts
            jitter_samples_s.append(send_ts - next_deadline)

            for j in range(7):
                peak_cmd_vel_abs[j] = max(peak_cmd_vel_abs[j], abs(float(dq_cmd[j])))
                peak_cmd_acc_abs[j] = max(peak_cmd_acc_abs[j], abs(float(ddq_cmd[j])))

            fb_read_time_s: Optional[float] = None
            frame_serial: Optional[int] = None
            traj_state: Optional[int] = None
            cur_state: Optional[int] = None
            cmd_state: Optional[int] = None
            err_code: Optional[int] = None
            q_fb = empty_joint_vector()
            dq_fb = empty_joint_vector()
            q_fb_cmd = empty_joint_vector()
            ctoq = empty_joint_vector()
            stoq = empty_joint_vector()
            them = empty_joint_vector()

            try:
                sub_data = robot.subscribe(dcss)
                fb_read_time_s = time.perf_counter() - t0
                if sub_data:
                    output = sub_data["outputs"][arm_idx]
                    state = sub_data["states"][arm_idx]
                    frame_serial = _optional_int(output.get("frame_serial"), -1)
                    traj_state = _optional_int(output.get("traj_state"), -1)
                    cur_state = _optional_int(state.get("cur_state"), -1)
                    cmd_state = _optional_int(state.get("cmd_state"), -1)
                    err_code = _optional_int(state.get("err_code"), -1)
                    q_fb = get_output_vector(output, "fb_joint_pos")
                    dq_fb = get_output_vector(output, "fb_joint_vel")
                    q_fb_cmd = get_output_vector(output, "fb_joint_cmd")
                    ctoq = get_output_vector(output, "fb_joint_cToq")
                    stoq = get_output_vector(output, "fb_joint_sToq")
                    them = get_output_vector(output, "fb_joint_them")
                    feedback_sample_count += 1

                    for j in range(7):
                        if dq_fb[j] is not None:
                            peak_fb_vel_abs[j] = max(peak_fb_vel_abs[j], abs(float(dq_fb[j])))
                        torque_candidates = [ctoq[j], stoq[j], them[j]]
                        for tq in torque_candidates:
                            if tq is not None:
                                peak_fb_torque_abs[j] = max(peak_fb_torque_abs[j], abs(float(tq)))
            except Exception as exc:
                if not first_subscribe_warning_printed:
                    print(f"[warn] subscribe during trial failed, continuing with NaN feedback for this sample: {exc}")
                    first_subscribe_warning_printed = True

            command_dt_ms = None if len(cmd_dt_samples_s) == 0 else cmd_dt_samples_s[-1] * 1000.0
            command_jitter_ms = jitter_samples_s[-1] * 1000.0 if jitter_samples_s else None

            # No disk I/O in the real-time loop.  All sample data is buffered
            # into preallocated NumPy arrays and written to NPZ/CSV after the trial.
            trial_buffer.add(
                sample_index=i,
                t_plan_s=t_plan,
                t_send_s=send_ts - t0,
                fb_read_time_s=fb_read_time_s,
                command_dt_ms=command_dt_ms,
                command_jitter_ms=command_jitter_ms,
                overrun_flag=overrun_flag,
                frame_serial=frame_serial,
                traj_state=traj_state,
                cur_state=cur_state,
                cmd_state=cmd_state,
                err_code=err_code,
                q_cmd=q_cmd,
                dq_cmd=dq_cmd,
                ddq_cmd=ddq_cmd,
                q_fb=q_fb,
                dq_fb=dq_fb,
                q_fb_cmd=q_fb_cmd,
                ctoq=ctoq,
                stoq=stoq,
                them=them,
            )

            next_deadline += period_s
    finally:
        timing_tuner.stop()
        platform_timing = timing_tuner.summary()

    save_trial_buffer(
        buffer=trial_buffer,
        sample_npz_path=sample_npz_path,
        sample_csv_path=sample_csv_path,
        group=group,
    )

    elapsed_s = time.perf_counter() - t0
    end_wall = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    command_dt_stats = stats_ms(cmd_dt_samples_s)
    command_jitter_stats = stats_ms(jitter_samples_s)
    moving_j = group.joint_index

    result = {
        "trial_index": trial_index,
        "trial_tag": trial_tag,
        "status": "executed",
        "start_time": start_wall,
        "end_time": end_wall,
        "elapsed_s": elapsed_s,
        "expected_duration_s": duration,
        "pre_trial_impedance_hold_s": float(args.pre_trial_impedance_hold_s),
        "sample_csv": sample_csv_path,
        "sample_npz": sample_npz_path,
        "high_precision_timer_enabled": high_precision_timer_enabled,
        "platform_timing": platform_timing,
        "spin_threshold_s": float(args.spin_threshold_s),
        "negative_sleep_count": negative_sleep_count,
        "overrun_count": overrun_count,
        "feedback_sample_count": feedback_sample_count,
        "command_dt": command_dt_stats,
        "command_jitter": command_jitter_stats,
        "test_group": asdict(group),
        "joint_k": [float(v) for v in args.joint_k],
        "joint_d": [float(v) for v in args.joint_d],
        "home_joints_deg": [float(v) for v in home_joints_deg],
        "duration_policy": {
            "fixed_duration_s": float(args.fixed_duration_s),
            "cycles_per_trial": float(args.cycles_per_trial),
            "min_trial_duration_s": float(args.min_trial_duration_s),
            "max_trial_duration_s": float(args.max_trial_duration_s),
            "actual_duration_s": float(duration),
            "actual_cycles": float(duration / group.period_s),
        },
        "peaks": {
            "moving_joint_peak_cmd_vel_deg_s": peak_cmd_vel_abs[moving_j],
            "moving_joint_peak_cmd_acc_deg_s2": peak_cmd_acc_abs[moving_j],
            "moving_joint_peak_fb_vel_deg_s": peak_fb_vel_abs[moving_j],
            "moving_joint_peak_fb_torque_abs": peak_fb_torque_abs[moving_j],
            "all_joint_peak_cmd_vel_deg_s": peak_cmd_vel_abs,
            "all_joint_peak_cmd_acc_deg_s2": peak_cmd_acc_abs,
            "all_joint_peak_fb_vel_deg_s": peak_fb_vel_abs,
            "all_joint_peak_fb_torque_abs": peak_fb_torque_abs,
        },
        "limit_check_after_run": {
            "moving_joint_fb_vel_over_limit": peak_fb_vel_abs[moving_j] > group.vel_limit_deg_s + 1e-9,
            "moving_joint_cmd_vel_over_limit": peak_cmd_vel_abs[moving_j] > group.vel_limit_deg_s + 1e-9,
            "moving_joint_cmd_acc_over_limit": peak_cmd_acc_abs[moving_j] > group.acc_limit_deg_s2 + 1e-9,
        },
    }
    json_dump(trial_json_path, result)
    result["trial_json"] = trial_json_path
    return result


def build_summary_row_from_executed(result: Dict[str, object]) -> Dict[str, object]:
    group = result.get("test_group", {}) if isinstance(result, dict) else {}
    cmd_dt = result.get("command_dt", {}) if isinstance(result, dict) else {}
    jitter = result.get("command_jitter", {}) if isinstance(result, dict) else {}
    peaks = result.get("peaks", {}) if isinstance(result, dict) else {}
    limit_after = result.get("limit_check_after_run", {}) if isinstance(result, dict) else {}
    duration_policy = result.get("duration_policy", {}) if isinstance(result, dict) else {}
    return {
        "trial_index": result.get("trial_index"),
        "status": result.get("status"),
        "joint_name": group.get("joint_name"),
        "target_speed_deg_s": group.get("target_speed_deg_s"),
        "amplitude_deg": group.get("amplitude_deg"),
        "omega_hz": group.get("omega_hz"),
        "test_id": group.get("test_id"),
        "period_s": group.get("period_s"),
        "theoretical_peak_vel_deg_s": group.get("theoretical_peak_vel_deg_s"),
        "theoretical_peak_acc_deg_s2": group.get("theoretical_peak_acc_deg_s2"),
        "vel_limit_deg_s": group.get("vel_limit_deg_s"),
        "acc_limit_deg_s2": group.get("acc_limit_deg_s2"),
        "precheck_result": group.get("precheck_result"),
        "skip_reason": group.get("skip_reason"),
        "feedback_watch": group.get("feedback_watch"),
        "historical_feedback_peak_vel_deg_s": group.get("historical_feedback_peak_vel_deg_s"),
        "actual_duration_s": duration_policy.get("actual_duration_s"),
        "actual_cycles": duration_policy.get("actual_cycles"),
        "high_precision_timer_enabled": result.get("high_precision_timer_enabled"),
        "spin_threshold_s": result.get("spin_threshold_s"),
        "command_dt_mean_ms": cmd_dt.get("mean_ms"),
        "command_dt_max_ms": cmd_dt.get("max_ms"),
        "command_dt_p95_ms": cmd_dt.get("p95_ms"),
        "command_jitter_mean_ms": jitter.get("mean_ms"),
        "command_jitter_max_ms": jitter.get("max_ms"),
        "command_jitter_p95_ms": jitter.get("p95_ms"),
        "negative_sleep_count": result.get("negative_sleep_count"),
        "overrun_count": result.get("overrun_count"),
        "feedback_sample_count": result.get("feedback_sample_count"),
        "moving_joint_peak_cmd_vel_deg_s": peaks.get("moving_joint_peak_cmd_vel_deg_s"),
        "moving_joint_peak_cmd_acc_deg_s2": peaks.get("moving_joint_peak_cmd_acc_deg_s2"),
        "moving_joint_peak_fb_vel_deg_s": peaks.get("moving_joint_peak_fb_vel_deg_s"),
        "moving_joint_peak_fb_torque_abs": peaks.get("moving_joint_peak_fb_torque_abs"),
        "moving_joint_fb_vel_over_limit": limit_after.get("moving_joint_fb_vel_over_limit"),
        "sample_csv": result.get("sample_csv"),
        "sample_npz": result.get("sample_npz"),
        "trial_json": result.get("trial_json"),
        "start_time": result.get("start_time"),
        "end_time": result.get("end_time"),
        "elapsed_s": result.get("elapsed_s"),
    }


def build_summary_row_from_skipped(group: TestGroup, trial_index: int) -> Dict[str, object]:
    return {
        "trial_index": trial_index,
        "status": "skipped",
        "joint_name": group.joint_name,
        "target_speed_deg_s": group.target_speed_deg_s,
        "amplitude_deg": group.amplitude_deg,
        "omega_hz": group.omega_hz,
        "test_id": group.test_id,
        "period_s": group.period_s,
        "theoretical_peak_vel_deg_s": group.theoretical_peak_vel_deg_s,
        "theoretical_peak_acc_deg_s2": group.theoretical_peak_acc_deg_s2,
        "vel_limit_deg_s": group.vel_limit_deg_s,
        "acc_limit_deg_s2": group.acc_limit_deg_s2,
        "precheck_result": group.precheck_result,
        "skip_reason": group.skip_reason,
        "feedback_watch": group.feedback_watch,
        "historical_feedback_peak_vel_deg_s": group.historical_feedback_peak_vel_deg_s,
    }


# ---------------------------------------------------------------------------
# CLI and main.
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Per-joint Marvin M6 sine sweep at 200 Hz in joint impedance mode")
    parser.add_argument("--robot_ip", type=str, default=str(CONFIG_DEFAULTS["robot_ip"]), help="Robot IP")
    parser.add_argument("--arm", choices=["A", "B"], default=str(CONFIG_DEFAULTS["arm"]), help="Arm selector")
    parser.add_argument("--ctrl_hz", type=int, default=int(CONFIG_DEFAULTS["ctrl_hz"]), help="Control frequency in Hz")
    parser.add_argument("--vel_ratio", type=int, default=int(CONFIG_DEFAULTS["vel_ratio"]), help="SDK velocity ratio")
    parser.add_argument("--acc_ratio", type=int, default=int(CONFIG_DEFAULTS["acc_ratio"]), help="SDK acceleration ratio")
    parser.add_argument("--home_mode", choices=["zero", "report", "sdk_default"], default="zero", help="Default home source if --home_joints is not set")
    parser.add_argument("--home_joints", nargs=7, type=float, default=None, help="Explicit home joints in deg")
    parser.add_argument("--home_send_hz", type=float, default=float(CONFIG_DEFAULTS["home_send_hz"]), help="Send rate for home move")
    parser.add_argument("--home_timeout_s", type=float, default=float(CONFIG_DEFAULTS["home_timeout_s"]), help="Timeout for home move")
    parser.add_argument("--home_tol_deg", type=float, default=float(CONFIG_DEFAULTS["home_tol_deg"]), help="Home tolerance in deg")
    parser.add_argument("--home_stable_samples", type=int, default=int(CONFIG_DEFAULTS["home_stable_samples"]), help="Stable samples for home reached")
    parser.add_argument("--pre_wait_s", type=float, default=float(CONFIG_DEFAULTS["pre_wait_s"]), help="Wait after reaching home")
    parser.add_argument("--between_trial_wait_s", type=float, default=float(CONFIG_DEFAULTS["between_trial_wait_s"]), help="Wait between trials")
    parser.add_argument("--joint_k", nargs=7, type=float, default=JOINT_IMP_DEFAULT_K, help="Fixed joint impedance K values")
    parser.add_argument("--joint_d", nargs=7, type=float, default=JOINT_IMP_DEFAULT_D, help="Fixed joint impedance D values")
    parser.add_argument("--cycles_per_trial", type=float, default=float(CONFIG_DEFAULTS["cycles_per_trial"]), help="Desired cycles per trial before duration clamp")
    parser.add_argument("--min_trial_duration_s", type=float, default=float(CONFIG_DEFAULTS["min_trial_duration_s"]), help="Minimum trial duration")
    parser.add_argument("--max_trial_duration_s", type=float, default=float(CONFIG_DEFAULTS["max_trial_duration_s"]), help="Maximum trial duration")
    parser.add_argument("--fixed_duration_s", type=float, default=float(CONFIG_DEFAULTS["fixed_duration_s"]), help="Fixed duration; 0 means auto")
    parser.add_argument("--fade_in_s", type=float, default=0.0, help="Optional smooth envelope at trial start; default 0 keeps original sine formula")
    parser.add_argument("--pre_trial_impedance_hold_s", type=float, default=float(CONFIG_DEFAULTS["pre_trial_impedance_hold_s"]), help="Hold home target in joint impedance mode before each logged sine trial")
    parser.add_argument("--spin_threshold_s", type=float, default=0.0005, help="Busy-wait threshold used by sleep_until; larger values reduce sleep jitter but use more CPU")
    parser.add_argument("--linux_timer_slack_ns", type=int, default=50000, help="Linux only: best-effort PR_SET_TIMERSLACK value in ns; use -1 to disable")
    parser.add_argument("--linux_rt_priority", type=int, default=0, help="Linux only: request SCHED_FIFO priority 1-99; 0 disables")
    parser.add_argument("--cpu_affinity", type=str, default="", help="Linux only: optional CPU affinity, e.g. '2' or '2,3' or '2-3'")
    parser.add_argument("--result_dir", type=str, default=str(CONFIG_DEFAULTS["result_dir"]), help="Result directory")
    parser.add_argument("--tag", type=str, default=str(CONFIG_DEFAULTS["tag"]), help="Run tag")
    parser.add_argument("--joint_filter", nargs="*", default=None, help="Optional joints to run, e.g. J1 J5 7")
    parser.add_argument("--speed_filter", nargs="*", type=float, default=None, help="Optional calculated target peak speeds to run, e.g. 30 60 180")
    parser.add_argument("--table_json", type=str, default=None, help="Optional external JSON table for custom sine tests")
    parser.add_argument("--precheck_only", action="store_true", help="Only write precheck table and exit without connecting robot")
    parser.add_argument("--force_execute_skipped", action="store_true", help="Dangerous: execute theoretically skipped tests. Not recommended.")
    parser.add_argument("--no_confirm", action="store_true", help="Do not ask interactive confirmation before robot motion")
    parser.add_argument("--continue_on_trial_failure", action="store_true", help="Continue next trial after a trial failure")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if int(args.ctrl_hz) <= 0:
        raise ValueError("ctrl_hz must be > 0")
    if len(args.joint_k) != 7 or len(args.joint_d) != 7:
        raise ValueError("joint_k and joint_d must each contain 7 numbers")
    if float(args.cycles_per_trial) <= 0.0:
        raise ValueError("cycles_per_trial must be > 0")
    if float(args.min_trial_duration_s) <= 0.0:
        raise ValueError("min_trial_duration_s must be > 0")
    if float(args.max_trial_duration_s) <= 0.0:
        raise ValueError("max_trial_duration_s must be > 0")
    if float(args.max_trial_duration_s) < float(args.min_trial_duration_s):
        raise ValueError("max_trial_duration_s must be >= min_trial_duration_s")
    if float(args.fixed_duration_s) < 0.0:
        raise ValueError("fixed_duration_s must be >= 0")
    if float(args.fade_in_s) < 0.0:
        raise ValueError("fade_in_s must be >= 0")
    if float(args.pre_trial_impedance_hold_s) < 0.0:
        raise ValueError("pre_trial_impedance_hold_s must be >= 0")
    if float(args.spin_threshold_s) < 0.0:
        raise ValueError("spin_threshold_s must be >= 0")
    if int(args.linux_timer_slack_ns) < -1:
        raise ValueError("linux_timer_slack_ns must be >= -1")
    if int(args.linux_rt_priority) < 0 or int(args.linux_rt_priority) > 99:
        raise ValueError("linux_rt_priority must be in [0, 99]")
    parse_cpu_affinity(args.cpu_affinity)
    if args.table_json and not os.path.isfile(args.table_json):
        raise ValueError(f"table_json does not exist: {args.table_json}")


def resolve_home_joints(args: argparse.Namespace) -> List[float]:
    if args.home_joints is not None:
        return [float(v) for v in args.home_joints]
    if args.home_mode == "sdk_default":
        return default_joints_for_arm(args.arm)
    if args.home_mode == "report":
        return [float(v) for v in REPORT_HOME_JOINTS_DEG]
    return [float(v) for v in ZERO_HOME_JOINTS_DEG]


def confirm_or_abort(args: argparse.Namespace, groups: Sequence[TestGroup], home_joints: Sequence[float]) -> None:
    if args.no_confirm:
        return
    executable = [g for g in groups if g.precheck_result.startswith("PASS")]
    skipped = [g for g in groups if not g.precheck_result.startswith("PASS")]
    print("\n[confirm] About to run robot motion with the following configuration:")
    print(f"  robot_ip: {args.robot_ip}")
    print(f"  arm: {args.arm}")
    print(f"  ctrl_hz: {args.ctrl_hz}")
    print(f"  home_joints_deg: {[round(float(v), 4) for v in home_joints]}")
    print(f"  joint_k: {[float(v) for v in args.joint_k]}")
    print(f"  joint_d: {[float(v) for v in args.joint_d]}")
    print(f"  executable groups: {len(executable)}")
    print(f"  skipped by precheck: {len(skipped)}")
    print("Type RUN to continue:")
    text = input("> ").strip()
    if text != "RUN":
        raise RuntimeError("User confirmation not received; abort")


def main() -> int:
    args = parse_args()
    validate_args(args)
    print_platform_timing_info(args)

    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_result_dir = normalize_cli_path(args.result_dir)
    result_dir = os.path.abspath(os.path.join(str(base_result_dir), f"{args.tag}_{run_stamp}"))
    os.makedirs(result_dir, exist_ok=True)

    home_joints_deg = resolve_home_joints(args)
    groups = build_test_plan(args.joint_filter, args.speed_filter, args.table_json)

    precheck_csv = os.path.join(result_dir, "precheck_all_groups.csv")
    write_precheck_csv(precheck_csv, groups)
    print(f"[precheck] wrote: {precheck_csv}")

    config_json = os.path.join(result_dir, "run_config.json")
    json_dump(
        config_json,
        {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "equipment": TEST_EQUIPMENT,
            "platform_timing_defaults": {
                "perf_counter": perf_counter_info(),
                "spin_threshold_s": float(args.spin_threshold_s),
                "linux_timer_slack_ns": int(args.linux_timer_slack_ns),
                "linux_rt_priority": int(args.linux_rt_priority),
                "cpu_affinity": str(args.cpu_affinity),
            },
            "args": vars(args),
            "home_joints_deg": home_joints_deg,
            "joint_k": [float(v) for v in args.joint_k],
            "joint_d": [float(v) for v in args.joint_d],
            "custom_sine_table_source": args.table_json if args.table_json else "CUSTOM_SINE_TEST_TABLE",
            "custom_sine_table": load_curve_table(args.table_json),
            "test_amplitude_deg": [float(g.amplitude_deg) for g in groups],
            "test_speeds_deg_s": [float(g.target_speed_deg_s) for g in groups],
            "test_omega_hz": [float(g.omega_hz) for g in groups],
            "vel_limit_deg_s": VEL_LIMIT_DEG_S,
            "acc_limit_deg_s2": ACC_LIMIT_DEG_S2,
            "feedback_watch_groups": [
                {
                    "joint_name": JOINT_NAMES[j],
                    "target_speed_deg_s": speed,
                    "historical_feedback_peak_vel_deg_s": HISTORICAL_FEEDBACK_PEAK_VEL_DEG_S.get((j, speed)),
                }
                for j, speed in sorted(FEEDBACK_WATCH_GROUPS)
            ],
        },
    )
    print(f"[config] wrote: {config_json}")

    skipped_count = sum(1 for g in groups if not g.precheck_result.startswith("PASS"))
    feedback_watch_count = sum(1 for g in groups if g.feedback_watch and g.precheck_result.startswith("PASS"))
    executable_count = sum(1 for g in groups if g.precheck_result.startswith("PASS"))
    print(
        f"[plan] total={len(groups)}, executable={executable_count}, "
        f"feedback_watch={feedback_watch_count}, skipped={skipped_count}"
    )
    if not groups:
        print("[warn] No enabled tests in the custom sine table. Edit CUSTOM_SINE_TEST_TABLE or pass --table_json.")

    if args.precheck_only:
        print("[done] precheck_only; no robot connection made")
        return 0

    confirm_or_abort(args, groups, home_joints_deg)

    load_robot_sdk()
    robot = Marvin_Robot()
    dcss = DCSS()
    connected = False
    summary_rows: List[Dict[str, object]] = []

    try:
        print("[step1-2] safe connect + clear errors")
        safe_connect_and_clear(robot=robot, dcss=dcss, robot_ip=args.robot_ip, arm=args.arm)
        connected = True

        print("[step3] enter position mode")
        enter_position_mode(robot, args.arm, args.vel_ratio, args.acc_ratio)

        print("[step4] move to home joints")
        print(f"[info] home_joints_deg={home_joints_deg}")
        move_to_joint_target_with_settle(
            robot=robot,
            dcss=dcss,
            arm=args.arm,
            target_joints_deg=home_joints_deg,
            send_hz=float(args.home_send_hz),
            timeout_s=float(args.home_timeout_s),
            tol_deg=float(args.home_tol_deg),
            stable_samples=int(args.home_stable_samples),
        )

        print(f"[step5] wait {float(args.pre_wait_s):.1f}s")
        time.sleep(float(args.pre_wait_s))

        total_trials = len(groups)
        for trial_index, group in enumerate(groups, start=1):
            if (not group.precheck_result.startswith("PASS")) and (not args.force_execute_skipped):
                print(f"[trial {trial_index}/{total_trials}] skip {group.joint_name} {group.target_speed_deg_s:g} deg/s: {group.skip_reason}")
                summary_rows.append(build_summary_row_from_skipped(group, trial_index))
                continue

            try:
                # Return to home before every formal trial.
                enter_position_mode(robot, args.arm, args.vel_ratio, args.acc_ratio)
                move_to_joint_target_with_settle(
                    robot=robot,
                    dcss=dcss,
                    arm=args.arm,
                    target_joints_deg=home_joints_deg,
                    send_hz=float(args.home_send_hz),
                    timeout_s=float(args.home_timeout_s),
                    tol_deg=float(args.home_tol_deg),
                    stable_samples=int(args.home_stable_samples),
                )
                if float(args.between_trial_wait_s) > 0.0:
                    time.sleep(float(args.between_trial_wait_s))

                result = execute_one_valid_group(
                    robot=robot,
                    dcss=dcss,
                    args=args,
                    group=group,
                    trial_index=trial_index,
                    total_trials=total_trials,
                    home_joints_deg=home_joints_deg,
                    result_dir=result_dir,
                )
                summary_rows.append(build_summary_row_from_executed(result))

            except KeyboardInterrupt:
                raise
            except Exception as exc:
                print(f"[trial {trial_index}] failed: {exc}")
                row = build_summary_row_from_skipped(group, trial_index)
                row["status"] = "failed"
                row["error_message"] = str(exc)
                summary_rows.append(row)
                if not args.continue_on_trial_failure:
                    raise
            finally:
                # Always return to position mode + home after each trial.
                try:
                    enter_position_mode(robot, args.arm, args.vel_ratio, args.acc_ratio)
                    move_to_joint_target_with_settle(
                        robot=robot,
                        dcss=dcss,
                        arm=args.arm,
                        target_joints_deg=home_joints_deg,
                        send_hz=float(args.home_send_hz),
                        timeout_s=min(float(args.home_timeout_s), 10.0),
                        tol_deg=float(args.home_tol_deg),
                        stable_samples=int(args.home_stable_samples),
                    )
                except Exception as recover_exc:
                    print(f"[recover] return home after trial failed: {recover_exc}")

        summary_csv = os.path.join(result_dir, "summary.csv")
        summary_json = os.path.join(result_dir, "summary.json")
        write_summary_csv(summary_csv, summary_rows)
        json_dump(summary_json, summary_rows)
        print(f"[done] summary csv: {summary_csv}")
        print(f"[done] summary json: {summary_json}")
        return 0

    except KeyboardInterrupt:
        print("\n[abort] interrupted by user")
        return 130
    except Exception as exc:
        print(f"[error] {exc}")
        if summary_rows:
            try:
                summary_csv = os.path.join(result_dir, "summary_partial.csv")
                summary_json = os.path.join(result_dir, "summary_partial.json")
                write_summary_csv(summary_csv, summary_rows)
                json_dump(summary_json, summary_rows)
                print(f"[partial] summary csv: {summary_csv}")
                print(f"[partial] summary json: {summary_json}")
            except Exception as write_exc:
                print(f"[partial] failed to write partial summary: {write_exc}")
        return 1
    finally:
        if connected:
            try:
                print("[shutdown] best-effort return home")
                enter_position_mode(robot, args.arm, args.vel_ratio, args.acc_ratio)
                move_to_joint_target_with_settle(
                    robot=robot,
                    dcss=dcss,
                    arm=args.arm,
                    target_joints_deg=home_joints_deg,
                    send_hz=float(args.home_send_hz),
                    timeout_s=min(float(args.home_timeout_s), 10.0),
                    tol_deg=float(args.home_tol_deg),
                    stable_samples=int(args.home_stable_samples),
                )
            except Exception as exc:
                print(f"[shutdown] best-effort return home failed: {exc}")
        best_effort_servo_off_and_release(robot, args.arm)


if __name__ == "__main__":
    raise SystemExit(main())
