"""
Cartesian circle test with SDK IK and formal joint-impedance tracking.

Requested workflow implemented by this script:
1) Safe connect
2) Clear errors
3) Enter position mode
4) Move to default joint position
5) Wait 5 seconds
6) Use high precision timer + sleep-until + timing stats
7) Build Cartesian circle around fixed center (xyz only, abc fixed)
8) Solve IK for points and execute at 200Hz, 18s(default), 4s per lap (joint-impedance mode)
9) MOVLA to first circle point, execute circle, MOVLA back to default
10) Finally servo-off and release robot connection
"""

import argparse
import ctypes
import json
import logging
import math
import os
import statistics
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from SDK_PYTHON.fx_kine import FX_InvKineSolvePara, Marvin_Kine
from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset

logging.getLogger("debug_printer").setLevel(logging.ERROR)


# Unified debug/tuning configuration section.
CONFIG_DEFAULTS = {
    "robot_ip": "192.168.1.190",
    "arm": "B",
    "kine_cfg": "ccs_m6_40.MvKDCfg",
    "vel_ratio": 60,
    "acc_ratio": 60,
    "home_send_hz": 100.0,
    "home_timeout_s": 20.0,
    "home_tol_deg": 1.0,
    "home_stable_samples": 20,
    "pre_wait_s": 5.0,
    "radius_m": 0.075,
    "plane": "xy",
    "ctrl_hz": 200,
    "duration_s": 20.0,
    "cycle_s": 2.0,
    "movla_vel_mm_s": 80.0,
    "movla_acc_mm_s2": 200.0,
    "movla_plan_hz": 500,
    "movla_send_hz": 100.0,
    "result_dir": "results_circle_200hz",
    "tag": "circle_posmode_ik",
    "xcorr_max_lag_s": 0.5,
    "j4_fundamental_hz": None,
    "fade_in_s": 2.0,
    "analysis_start_s": 4.0,
    "j4_inset_center_s": 10.0,
    "j4_inset_half_window_s": 2.0,
    "j4_vel_smooth_window_s": 0.08,
}

JOINT_IMP_DEFAULT_K = [22, 10, 10, 15, 9, 7, 15]
DD = 0.3
JOINT_IMP_DEFAULT_D = [DD, DD, DD, DD, DD, DD, DD]


def arm_to_index(arm: str) -> int:
    if arm == "A":
        return 0
    if arm == "B":
        return 1
    raise ValueError(f"arm must be 'A' or 'B', got {arm}")


def get_default_joints_for_arm(arm: str) -> List[float]:
    if arm == "A":
        return [90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0]
    return [90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0]


def stats_ms(samples_s: List[float]) -> Dict[str, Optional[float]]:
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
    arr_sorted = sorted(arr_ms)
    p95_idx = min(len(arr_sorted) - 1, max(0, int(math.ceil(0.95 * len(arr_sorted))) - 1))

    return {
        "count": len(arr_ms),
        "mean_ms": float(statistics.fmean(arr_ms)),
        "std_ms": float(statistics.pstdev(arr_ms)) if len(arr_ms) > 1 else 0.0,
        "min_ms": float(arr_sorted[0]),
        "max_ms": float(arr_sorted[-1]),
        "p95_ms": float(arr_sorted[p95_idx]),
    }


def _to_float_array_2d(data: object, cols: int) -> np.ndarray:
    arr = np.asarray(data if data is not None else [], dtype=np.float64)
    if arr.size == 0:
        return np.empty((0, cols), dtype=np.float64)
    if arr.ndim == 1:
        if (arr.size % cols) != 0:
            raise ValueError(f"Cannot reshape size {arr.size} into N x {cols}")
        arr = arr.reshape((-1, cols))
    if arr.ndim != 2 or arr.shape[1] != cols:
        raise ValueError(f"Expected shape (N, {cols}), got {arr.shape}")
    return arr


def _sanitize_time_and_values(t_s: np.ndarray, values: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if values.ndim == 1:
        values_2d = values.reshape(-1, 1)
        was_1d = True
    else:
        values_2d = values
        was_1d = False

    n = min(len(t_s), len(values_2d))
    if n <= 0:
        if was_1d:
            return np.empty((0,), dtype=np.float64), np.empty((0,), dtype=np.float64)
        return np.empty((0,), dtype=np.float64), np.empty((0, values_2d.shape[1]), dtype=np.float64)

    t_local = np.asarray(t_s[:n], dtype=np.float64)
    v_local = np.asarray(values_2d[:n], dtype=np.float64)

    finite_mask = np.isfinite(t_local) & np.all(np.isfinite(v_local), axis=1)
    t_local = t_local[finite_mask]
    v_local = v_local[finite_mask]
    if len(t_local) == 0:
        if was_1d:
            return np.empty((0,), dtype=np.float64), np.empty((0,), dtype=np.float64)
        return np.empty((0,), dtype=np.float64), np.empty((0, values_2d.shape[1]), dtype=np.float64)

    increasing_mask = np.insert(np.diff(t_local) > 0.0, 0, True)
    t_local = t_local[increasing_mask]
    v_local = v_local[increasing_mask]

    if was_1d:
        return t_local, v_local[:, 0]
    return t_local, v_local


def _build_common_time_grid(t_cmd_s: np.ndarray, t_fb_s: np.ndarray) -> Tuple[np.ndarray, Optional[float]]:
    if len(t_cmd_s) < 2 or len(t_fb_s) < 2:
        return np.empty((0,), dtype=np.float64), None

    dt_cmd = np.diff(t_cmd_s)
    dt_fb = np.diff(t_fb_s)
    dt_cmd = dt_cmd[dt_cmd > 0.0]
    dt_fb = dt_fb[dt_fb > 0.0]
    if len(dt_cmd) == 0 and len(dt_fb) == 0:
        return np.empty((0,), dtype=np.float64), None

    dt_pool = np.concatenate([dt_cmd, dt_fb]) if len(dt_cmd) and len(dt_fb) else (dt_cmd if len(dt_cmd) else dt_fb)
    dt_s = float(np.median(dt_pool))
    if not np.isfinite(dt_s) or dt_s <= 0.0:
        return np.empty((0,), dtype=np.float64), None

    start_s = float(max(t_cmd_s[0], t_fb_s[0]))
    end_s = float(min(t_cmd_s[-1], t_fb_s[-1]))
    if end_s <= start_s + dt_s:
        return np.empty((0,), dtype=np.float64), None

    t_grid_s = np.arange(start_s, end_s, dt_s, dtype=np.float64)
    if len(t_grid_s) < 5:
        return np.empty((0,), dtype=np.float64), None
    return t_grid_s, dt_s


def _interp_to_grid(t_src_s: np.ndarray, values: np.ndarray, t_grid_s: np.ndarray) -> np.ndarray:
    if values.ndim == 1:
        return np.interp(t_grid_s, t_src_s, values)

    out = np.empty((len(t_grid_s), values.shape[1]), dtype=np.float64)
    for i in range(values.shape[1]):
        out[:, i] = np.interp(t_grid_s, t_src_s, values[:, i])
    return out


def _estimate_lag_samples_xcorr(cmd_1d: np.ndarray, fb_1d: np.ndarray, dt_s: float, max_lag_s: float) -> int:
    cmd_center = cmd_1d - float(np.mean(cmd_1d))
    fb_center = fb_1d - float(np.mean(fb_1d))

    cmd_std = float(np.std(cmd_center))
    fb_std = float(np.std(fb_center))
    if cmd_std < 1e-12 or fb_std < 1e-12:
        return 0

    cmd_norm = cmd_center / cmd_std
    fb_norm = fb_center / fb_std

    corr = np.correlate(fb_norm, cmd_norm, mode="full")
    lags = np.arange(-len(cmd_norm) + 1, len(fb_norm))

    max_lag_samples = max(1, int(round(float(max_lag_s) / float(dt_s))))
    mask = np.abs(lags) <= max_lag_samples
    if not np.any(mask):
        return 0

    corr_valid = corr[mask]
    lags_valid = lags[mask]
    best_lag_samples = int(lags_valid[int(np.argmax(corr_valid))])
    return best_lag_samples


def estimate_lag_s_1d(
    t_cmd_s: np.ndarray,
    cmd_1d: np.ndarray,
    t_fb_s: np.ndarray,
    fb_1d: np.ndarray,
    max_lag_s: float,
) -> Optional[float]:
    t_cmd_s, cmd_1d = _sanitize_time_and_values(t_cmd_s, cmd_1d)
    t_fb_s, fb_1d = _sanitize_time_and_values(t_fb_s, fb_1d)
    if len(t_cmd_s) < 5 or len(t_fb_s) < 5:
        return None

    t_grid_s, dt_s = _build_common_time_grid(t_cmd_s, t_fb_s)
    if dt_s is None:
        return None

    cmd_i = _interp_to_grid(t_cmd_s, cmd_1d, t_grid_s)
    fb_i = _interp_to_grid(t_fb_s, fb_1d, t_grid_s)
    best_lag_samples = _estimate_lag_samples_xcorr(cmd_i, fb_i, dt_s, max_lag_s=max_lag_s)
    return float(best_lag_samples * dt_s)


def estimate_lag_s_cartesian(
    t_cmd_s: np.ndarray,
    cmd_xyz_mm: np.ndarray,
    t_fb_s: np.ndarray,
    fb_xyz_mm: np.ndarray,
    max_lag_s: float,
) -> Optional[float]:
    t_cmd_s, cmd_xyz_mm = _sanitize_time_and_values(t_cmd_s, cmd_xyz_mm)
    t_fb_s, fb_xyz_mm = _sanitize_time_and_values(t_fb_s, fb_xyz_mm)
    if len(t_cmd_s) < 5 or len(t_fb_s) < 5:
        return None

    t_grid_s, dt_s = _build_common_time_grid(t_cmd_s, t_fb_s)
    if dt_s is None:
        return None

    cmd_i = _interp_to_grid(t_cmd_s, cmd_xyz_mm, t_grid_s)
    fb_i = _interp_to_grid(t_fb_s, fb_xyz_mm, t_grid_s)
    if cmd_i.ndim != 2 or cmd_i.shape[1] != 3 or fb_i.ndim != 2 or fb_i.shape[1] != 3:
        return None

    cmd_centered = cmd_i - np.mean(cmd_i, axis=0, keepdims=True)
    _, singular_vals, vh = np.linalg.svd(cmd_centered, full_matrices=False)
    if len(singular_vals) == 0 or float(singular_vals[0]) < 1e-12:
        return None

    principal_axis = vh[0]
    if float(np.linalg.norm(principal_axis)) < 1e-12:
        return None

    cmd_proj = cmd_centered @ principal_axis
    fb_proj = (fb_i - np.mean(cmd_i, axis=0, keepdims=True)) @ principal_axis

    best_lag_samples = _estimate_lag_samples_xcorr(cmd_proj, fb_proj, dt_s, max_lag_s=max_lag_s)
    return float(best_lag_samples * dt_s)


def estimate_lag_s_phase_fundamental(
    t_cmd_s: np.ndarray,
    cmd_1d: np.ndarray,
    t_fb_s: np.ndarray,
    fb_1d: np.ndarray,
    fundamental_hz: Optional[float] = None,
    max_lag_s: Optional[float] = None,
) -> Tuple[Optional[float], Optional[float]]:
    t_cmd_s, cmd_1d = _sanitize_time_and_values(t_cmd_s, cmd_1d)
    t_fb_s, fb_1d = _sanitize_time_and_values(t_fb_s, fb_1d)
    if len(t_cmd_s) < 5 or len(t_fb_s) < 5:
        return None, None

    t_grid_s, dt_s = _build_common_time_grid(t_cmd_s, t_fb_s)
    if dt_s is None:
        return None, None

    cmd_i = _interp_to_grid(t_cmd_s, cmd_1d, t_grid_s)
    fb_i = _interp_to_grid(t_fb_s, fb_1d, t_grid_s)

    cmd_center = cmd_i - float(np.mean(cmd_i))
    fb_center = fb_i - float(np.mean(fb_i))
    if float(np.std(cmd_center)) < 1e-12 or float(np.std(fb_center)) < 1e-12:
        return None, None

    freq_hz = float(fundamental_hz) if (fundamental_hz is not None and float(fundamental_hz) > 0.0) else None
    if freq_hz is None:
        spec = np.abs(np.fft.rfft(cmd_center))
        freqs = np.fft.rfftfreq(len(cmd_center), d=float(dt_s))
        if len(spec) < 2 or len(freqs) < 2:
            return None, None

        spec[0] = 0.0
        peak_idx = int(np.argmax(spec))
        if peak_idx <= 0:
            return None, None
        freq_hz = float(freqs[peak_idx])
        if not np.isfinite(freq_hz) or freq_hz <= 0.0:
            return None, None

    w = 2.0 * math.pi * float(freq_hz)
    if abs(w) < 1e-12:
        return None, None

    t_local = t_grid_s - float(t_grid_s[0])
    design = np.column_stack([np.ones_like(t_local), np.cos(w * t_local), np.sin(w * t_local)])

    coef_cmd, *_ = np.linalg.lstsq(design, cmd_i, rcond=None)
    coef_fb, *_ = np.linalg.lstsq(design, fb_i, rcond=None)

    a_cmd = float(coef_cmd[1])
    b_cmd = float(coef_cmd[2])
    a_fb = float(coef_fb[1])
    b_fb = float(coef_fb[2])

    amp_cmd = float(math.hypot(a_cmd, b_cmd))
    amp_fb = float(math.hypot(a_fb, b_fb))
    if amp_cmd < 1e-12 or amp_fb < 1e-12:
        return None, None

    phi_cmd = float(np.arctan2(-b_cmd, a_cmd))
    phi_fb = float(np.arctan2(-b_fb, a_fb))
    dphi = float(np.arctan2(np.sin(phi_fb - phi_cmd), np.cos(phi_fb - phi_cmd)))

    # Sign convention: positive lag means feedback lags command.
    tau_s = float(-dphi / w)

    if max_lag_s is not None and float(max_lag_s) > 0.0 and freq_hz > 1e-12:
        period_s = 1.0 / float(freq_hz)
        k_min = int(math.floor((-float(max_lag_s) - tau_s) / period_s))
        k_max = int(math.ceil((float(max_lag_s) - tau_s) / period_s))
        candidates = [tau_s + k * period_s for k in range(k_min, k_max + 1)]
        if candidates:
            tau_s = float(min(candidates, key=lambda v: abs(v)))
        else:
            tau_s = float(tau_s - round(tau_s / period_s) * period_s)

    return tau_s, float(freq_hz)


def find_delay_by_reference_level(
    t_s: np.ndarray,
    q_cmd_deg: np.ndarray,
    q_fb_deg: np.ndarray,
    ref_deg: float = -90.0,
    window_s: Optional[Tuple[float, float]] = None,
    max_ref_error_deg: float = 2.0,
) -> Optional[Dict[str, float]]:
    t_s = np.asarray(t_s, dtype=np.float64).reshape(-1)
    q_cmd_deg = np.asarray(q_cmd_deg, dtype=np.float64).reshape(-1)
    q_fb_deg = np.asarray(q_fb_deg, dtype=np.float64).reshape(-1)

    n = min(len(t_s), len(q_cmd_deg), len(q_fb_deg))
    if n < 2:
        return None

    t_local = t_s[:n]
    q_cmd_local = q_cmd_deg[:n]
    q_fb_local = q_fb_deg[:n]

    finite_mask = np.isfinite(t_local) & np.isfinite(q_cmd_local) & np.isfinite(q_fb_local)
    if not np.any(finite_mask):
        return None

    t_local = t_local[finite_mask]
    q_cmd_local = q_cmd_local[finite_mask]
    q_fb_local = q_fb_local[finite_mask]

    if window_s is not None:
        w0 = float(window_s[0])
        w1 = float(window_s[1])
        if w1 < w0:
            w0, w1 = w1, w0
        mask_w = (t_local >= w0) & (t_local <= w1)
        if not np.any(mask_w):
            return None
        t_local = t_local[mask_w]
        q_cmd_local = q_cmd_local[mask_w]
        q_fb_local = q_fb_local[mask_w]

    idx_cmd = int(np.argmin(np.abs(q_cmd_local - float(ref_deg))))
    idx_fb = int(np.argmin(np.abs(q_fb_local - float(ref_deg))))

    cmd_err = abs(float(q_cmd_local[idx_cmd]) - float(ref_deg))
    fb_err = abs(float(q_fb_local[idx_fb]) - float(ref_deg))
    if cmd_err > float(max_ref_error_deg) or fb_err > float(max_ref_error_deg):
        return None

    t_cmd = float(t_local[idx_cmd])
    t_fb = float(t_local[idx_fb])
    delay_s = float(t_fb - t_cmd)

    return {
        "ref_deg": float(ref_deg),
        "t_cmd_s": t_cmd,
        "t_fb_s": t_fb,
        "q_cmd_deg": float(q_cmd_local[idx_cmd]),
        "q_fb_deg": float(q_fb_local[idx_fb]),
        "cmd_ref_error_deg": float(cmd_err),
        "fb_ref_error_deg": float(fb_err),
        "delay_s": delay_s,
        "delay_ms": float(delay_s * 1000.0),
    }


def annotate_delay_on_axis(
    ax: plt.Axes,
    delay_info: Optional[Dict[str, float]],
    y_arrow: Optional[float] = None,
    text_offset: Tuple[float, float] = (0.0, 0.05),
) -> None:
    if not delay_info:
        return

    t_cmd = float(delay_info["t_cmd_s"])
    t_fb = float(delay_info["t_fb_s"])
    delay_ms = float(delay_info["delay_ms"])

    ax.axvline(t_cmd, color="red", linestyle="--", linewidth=1.0, alpha=0.85)
    ax.axvline(t_fb, color="red", linestyle="--", linewidth=1.0, alpha=0.85)

    x_min, x_max = ax.get_xlim()
    y_min, y_max = ax.get_ylim()
    x_span = max(1e-9, float(x_max - x_min))
    y_span = max(1e-9, float(y_max - y_min))

    if y_arrow is None:
        y_arrow = float(y_min + 0.86 * y_span)

    ax.annotate(
        "",
        xy=(t_cmd, y_arrow),
        xytext=(t_fb, y_arrow),
        arrowprops={"arrowstyle": "<->", "color": "red", "lw": 1.2},
    )

    x_mid = 0.5 * (t_cmd + t_fb)
    text_x = float(x_mid + float(text_offset[0]) * x_span)
    text_y = float(y_arrow + float(text_offset[1]) * y_span)
    ax.text(
        text_x,
        text_y,
        f"{delay_ms:.2f} ms",
        color="red",
        fontsize=9,
        ha="center",
        va="bottom",
        bbox={"facecolor": "white", "edgecolor": "red", "alpha": 0.75, "boxstyle": "round,pad=0.2"},
    )


def _set_axes_equal_3d(ax: plt.Axes, xyz_a: np.ndarray, xyz_b: np.ndarray) -> None:
    clouds: List[np.ndarray] = []
    if len(xyz_a) > 0:
        clouds.append(np.asarray(xyz_a, dtype=np.float64))
    if len(xyz_b) > 0:
        clouds.append(np.asarray(xyz_b, dtype=np.float64))
    if not clouds:
        return

    pts = np.vstack(clouds)
    mins = np.min(pts, axis=0)
    maxs = np.max(pts, axis=0)
    center = 0.5 * (mins + maxs)
    radius = 0.5 * float(np.max(maxs - mins))
    if not np.isfinite(radius) or radius <= 0.0:
        radius = 1e-3

    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def _plot_trajectory_on_axis(ax: plt.Axes, cmd_xyz_m: np.ndarray, fb_xyz_m: np.ndarray) -> None:
    if len(cmd_xyz_m) > 0:
        ax.plot(cmd_xyz_m[:, 0], cmd_xyz_m[:, 1], cmd_xyz_m[:, 2], linewidth=1.8, label="Command XYZ")
    if len(fb_xyz_m) > 0:
        ax.plot(fb_xyz_m[:, 0], fb_xyz_m[:, 1], fb_xyz_m[:, 2], linewidth=1.4, alpha=0.9, label="Feedback XYZ")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.grid(True)
    _set_axes_equal_3d(ax, cmd_xyz_m, fb_xyz_m)


def save_trajectory_3d_plot(
    cmd_xyz_m: np.ndarray,
    fb_xyz_m: np.ndarray,
    output_path: str,
    cart_delay_ms: Optional[float] = None,
) -> None:
    fig = plt.figure(figsize=(24, 7), dpi=160)

    ax_1 = fig.add_subplot(131, projection="3d")
    ax_2 = fig.add_subplot(132, projection="3d")
    ax_3 = fig.add_subplot(133, projection="3d")

    _plot_trajectory_on_axis(ax_1, cmd_xyz_m, fb_xyz_m)
    _plot_trajectory_on_axis(ax_2, cmd_xyz_m, fb_xyz_m)
    _plot_trajectory_on_axis(ax_3, cmd_xyz_m, fb_xyz_m)

    # 1) 保留原来的斜视图
    ax_1.view_init(elev=25.0, azim=45.0)

    # 2) 俯视图：从上往下看
    ax_2.view_init(elev=90.0, azim=0.0)

    # 3) 侧视图：更容易看Z方向偏差
    ax_3.view_init(elev=0.0, azim=45.0)

    delay_text = f"PCA correlation delay: {float(cart_delay_ms):.1f} ms" if cart_delay_ms is not None else None

    ax_1.set_title(
        f"Cartesian 3D Trajectory (View A{', ' + delay_text if delay_text is not None else ''})"
    )
    ax_2.set_title("Cartesian 3D Trajectory (Top View)")
    ax_3.set_title("Cartesian 3D Trajectory (Side View for Z Error)")

    ax_1.legend(loc="best")
    ax_2.legend(loc="best")
    ax_3.legend(loc="best")

    if delay_text is not None:
        fig.suptitle(f"Cartesian 3D Cmd vs Feedback | {delay_text}")
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    else:
        fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def save_cartesian_error_plot(t_s: np.ndarray, e_mm: np.ndarray, output_path: str) -> None:
    t_s = np.asarray(t_s, dtype=np.float64).reshape(-1)
    e_mm = np.asarray(e_mm, dtype=np.float64).reshape(-1)

    n = min(len(t_s), len(e_mm))
    if n <= 0:
        raise ValueError("No samples for Cartesian error plot")

    t_s = t_s[:n]
    e_mm = e_mm[:n]

    mean_mm = float(np.mean(e_mm))
    max_mm = float(np.max(e_mm))
    p95_mm = float(np.percentile(e_mm, 95))

    fig, ax = plt.subplots(1, 1, figsize=(12, 4.8), dpi=160)

    ax.plot(t_s, e_mm, linewidth=1.5, color="#d62728", label="|p_fb - p_cmd|")
    ax.axhline(mean_mm, linestyle="--", linewidth=1.2, color="#1f77b4", label=f"mean = {mean_mm:.2f} mm")
    ax.axhline(max_mm, linestyle="--", linewidth=1.2, color="#9467bd", label=f"max = {max_mm:.2f} mm")
    ax.axhline(p95_mm, linestyle="--", linewidth=1.2, color="#2ca02c", label=f"p95 = {p95_mm:.2f} mm")
    ax.set_title("Cartesian Tracking Error vs Time")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("e (mm)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _first_order_diff(t_s: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if len(t_s) < 2 or len(y) < 2:
        return np.empty((0,), dtype=np.float64), np.empty((0,), dtype=np.float64)

    dt = np.diff(t_s)
    dy = np.diff(y)
    valid = dt > 0.0
    if not np.any(valid):
        return np.empty((0,), dtype=np.float64), np.empty((0,), dtype=np.float64)

    t_mid = t_s[:-1][valid] + 0.5 * dt[valid]
    d = dy[valid] / dt[valid]
    return t_mid, d


def _moving_average_1d(y: np.ndarray, window_samples: int) -> np.ndarray:
    y = np.asarray(y, dtype=np.float64).reshape(-1)
    if len(y) == 0:
        return np.empty((0,), dtype=np.float64)

    w = int(max(1, window_samples))
    if (w % 2) == 0:
        w += 1

    max_w = len(y) if (len(y) % 2) == 1 else (len(y) - 1)
    max_w = max(1, max_w)
    w = min(w, max_w)
    if w <= 1:
        return y.copy()

    pad = w // 2
    y_pad = np.pad(y, (pad, pad), mode="edge")
    kernel = np.ones(w, dtype=np.float64) / float(w)
    return np.convolve(y_pad, kernel, mode="valid")


def _align_velocity_traces(
    t_cmd_s: np.ndarray,
    cmd_1d: np.ndarray,
    t_fb_s: np.ndarray,
    fb_1d: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    t_cmd_s, cmd_1d = _sanitize_time_and_values(t_cmd_s, cmd_1d)
    t_fb_s, fb_1d = _sanitize_time_and_values(t_fb_s, fb_1d)
    if len(t_cmd_s) < 2 or len(t_fb_s) < 2:
        empty = np.empty((0,), dtype=np.float64)
        return empty, empty, empty

    t_grid_s, _ = _build_common_time_grid(t_cmd_s, t_fb_s)
    if len(t_grid_s) == 0:
        empty = np.empty((0,), dtype=np.float64)
        return empty, empty, empty

    cmd_i = _interp_to_grid(t_cmd_s, cmd_1d, t_grid_s)
    fb_i = _interp_to_grid(t_fb_s, fb_1d, t_grid_s)
    return t_grid_s, cmd_i, fb_i


def save_j4_error_triplet_plot(
    t_s: np.ndarray,
    q_cmd_j4_deg: np.ndarray,
    q_fb_j4_deg: np.ndarray,
    vel_smooth_window_s: float,
    output_path: str,
) -> None:
    t_s = np.asarray(t_s, dtype=np.float64).reshape(-1)
    q_cmd_j4_deg = np.asarray(q_cmd_j4_deg, dtype=np.float64).reshape(-1)
    q_fb_j4_deg = np.asarray(q_fb_j4_deg, dtype=np.float64).reshape(-1)

    n = min(len(t_s), len(q_cmd_j4_deg), len(q_fb_j4_deg))
    if n < 2:
        raise ValueError("Insufficient samples for J4 error plot")
    t_s = t_s[:n]
    q_cmd_j4_deg = q_cmd_j4_deg[:n]
    q_fb_j4_deg = q_fb_j4_deg[:n]

    pos_err_deg = q_fb_j4_deg - q_cmd_j4_deg

    v_t_cmd, v_cmd = _first_order_diff(t_s, q_cmd_j4_deg)
    v_t_fb, v_fb = _first_order_diff(t_s, q_fb_j4_deg)
    v_t, v_cmd_i, v_fb_i = _align_velocity_traces(v_t_cmd, v_cmd, v_t_fb, v_fb)

    vel_err = np.empty((0,), dtype=np.float64)
    vel_err_smooth = np.empty((0,), dtype=np.float64)
    window_samples_used = 1
    if len(v_t) > 0:
        vel_err = v_fb_i - v_cmd_i

        dt_v = np.diff(v_t)
        dt_v = dt_v[dt_v > 0.0]
        dt_median = float(np.median(dt_v)) if len(dt_v) > 0 else 0.0

        if dt_median > 0.0 and float(vel_smooth_window_s) > 0.0:
            window_samples_used = int(round(float(vel_smooth_window_s) / dt_median))
        v_fb_smooth = _moving_average_1d(v_fb_i, window_samples=window_samples_used)
        vel_err_smooth = v_fb_smooth - v_cmd_i

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), dpi=160, sharex=True)

    axes[0].plot(t_s, pos_err_deg, linewidth=1.5, label="J4 Position Error (fb - cmd)")
    axes[0].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    axes[0].set_title("J4 Position Error")
    axes[0].set_ylabel("deg")
    axes[0].grid(True, alpha=0.35)
    axes[0].legend(loc="best")

    if len(v_t) > 0:
        axes[1].plot(v_t, vel_err, linewidth=1.4, color="#d62728", label="J4 Velocity Error (fb - cmd)")
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    axes[1].set_title("J4 Velocity Error (1st Difference)")
    axes[1].set_ylabel("deg/s")
    axes[1].grid(True, alpha=0.35)
    axes[1].legend(loc="best")

    if len(v_t) > 0:
        axes[2].plot(
            v_t,
            vel_err_smooth,
            linewidth=1.4,
            color="#2ca02c",
            label=f"Smoothed Vel Error (window={window_samples_used} samples)",
        )
    axes[2].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    axes[2].set_title("J4 Smoothed Velocity Error")
    axes[2].set_ylabel("deg/s")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(True, alpha=0.35)
    axes[2].legend(loc="best")

    axes[2].set_xlim(float(t_s[0]), float(t_s[-1]))

    fig.tight_layout(h_pad=1.2)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def save_j4_pos_vel_acc_plot(t_s: np.ndarray, q_cmd_j4_deg: np.ndarray, q_fb_j4_deg: np.ndarray, output_path: str) -> None:
    t_s = np.asarray(t_s, dtype=np.float64)
    q_cmd_j4_deg = np.asarray(q_cmd_j4_deg, dtype=np.float64)
    q_fb_j4_deg = np.asarray(q_fb_j4_deg, dtype=np.float64)

    v_t_cmd, v_cmd = _first_order_diff(t_s, q_cmd_j4_deg)
    v_t_fb, v_fb = _first_order_diff(t_s, q_fb_j4_deg)

    a_t_cmd, a_cmd = _first_order_diff(v_t_cmd, v_cmd)
    a_t_fb, a_fb = _first_order_diff(v_t_fb, v_fb)

    fig, axes = plt.subplots(3, 1, figsize=(14, 11), dpi=160, sharex=False)

    axes[0].plot(t_s, q_cmd_j4_deg, linewidth=1.8, label="J4 Command")
    axes[0].plot(t_s, q_fb_j4_deg, linewidth=1.4, label="J4 Feedback")
    axes[0].set_title("J4 Position")
    axes[0].set_ylabel("deg")
    axes[0].grid(True, alpha=0.35)
    axes[0].legend(loc="best")

    # Inset zoom for J4 position with horizontally stretched window.
    axins = inset_axes(
        axes[0],
        width="55%",
        height="38%",
        loc="center right",
        borderpad=2.0,
    )
    axins.plot(t_s, q_cmd_j4_deg, linewidth=1.4)
    axins.plot(t_s, q_fb_j4_deg, linewidth=1.2)

    t_center = float(CONFIG_DEFAULTS.get("j4_inset_center_s", 4.0))
    half_window = max(1e-6, float(CONFIG_DEFAULTS.get("j4_inset_half_window_s", 0.5)))
    x1 = t_center - half_window
    x2 = t_center + half_window
    axins.set_xlim(x1, x2)

    mask = (t_s >= x1) & (t_s <= x2)
    y_local = np.concatenate([q_cmd_j4_deg[mask], q_fb_j4_deg[mask]]) if np.any(mask) else np.array([])
    if y_local.size > 0:
        y_min = float(np.min(y_local))
        y_max = float(np.max(y_local))
        pad = max(0.2, 0.08 * (y_max - y_min + 1e-9))
        axins.set_ylim(y_min - pad, y_max + pad)

    axins.grid(True, alpha=0.4)
    axins.set_title(f"Zoom near t={t_center:.2f}s", fontsize=10)
    mark_inset(axes[0], axins, loc1=2, loc2=4, fc="none", ec="0.5")

    axes[1].plot(v_t_cmd, v_cmd, linewidth=1.6, label="J4 Command Vel")
    axes[1].plot(v_t_fb, v_fb, linewidth=1.3, label="J4 Feedback Vel")
    axes[1].set_title("J4 Velocity (1st Difference)")
    axes[1].set_ylabel("deg/s")
    axes[1].grid(True, alpha=0.35)
    axes[1].legend(loc="best")

    axes[2].plot(a_t_cmd, a_cmd, linewidth=1.6, label="J4 Command Acc")
    axes[2].plot(a_t_fb, a_fb, linewidth=1.3, label="J4 Feedback Acc")
    axes[2].set_title("J4 Acceleration (2nd Difference)")
    axes[2].set_xlabel("time (s)")
    axes[2].set_ylabel("deg/s^2")
    axes[2].grid(True, alpha=0.35)
    axes[2].legend(loc="best")

    fig.tight_layout(h_pad=1.8)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def save_j4_position_zoom_delay_plot(
    t_s: np.ndarray,
    q_cmd_j4_deg: np.ndarray,
    q_fb_j4_deg: np.ndarray,
    output_path: str,
    zoom_window_s: Tuple[float, float] = (9.5, 10.5),
    ref_deg: float = -90.0,
    max_ref_error_deg: float = 2.0,
) -> None:
    t_s = np.asarray(t_s, dtype=np.float64).reshape(-1)
    q_cmd_j4_deg = np.asarray(q_cmd_j4_deg, dtype=np.float64).reshape(-1)
    q_fb_j4_deg = np.asarray(q_fb_j4_deg, dtype=np.float64).reshape(-1)

    n = min(len(t_s), len(q_cmd_j4_deg), len(q_fb_j4_deg))
    if n < 2:
        raise ValueError("Insufficient samples for J4 zoom delay plot")

    t_s = t_s[:n]
    q_cmd_j4_deg = q_cmd_j4_deg[:n]
    q_fb_j4_deg = q_fb_j4_deg[:n]

    x1 = float(zoom_window_s[0])
    x2 = float(zoom_window_s[1])
    if x2 < x1:
        x1, x2 = x2, x1

    fig, ax = plt.subplots(1, 1, figsize=(12, 5), dpi=180)

    ax.plot(t_s, q_cmd_j4_deg, linewidth=1.8, label="J4 Command")
    ax.plot(t_s, q_fb_j4_deg, linewidth=1.4, label="J4 Feedback")
    ax.set_title("J4 Position with Zoom Delay Annotation")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("deg")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best")

    axins = inset_axes(
        ax,
        width="55%",
        height="45%",
        loc="center right",
        borderpad=2.0,
    )
    axins.plot(t_s, q_cmd_j4_deg, linewidth=1.4)
    axins.plot(t_s, q_fb_j4_deg, linewidth=1.2)
    axins.set_xlim(x1, x2)

    mask = (t_s >= x1) & (t_s <= x2)
    y_local = np.concatenate([q_cmd_j4_deg[mask], q_fb_j4_deg[mask]]) if np.any(mask) else np.array([])
    if y_local.size > 0:
        y_min = float(np.min(y_local))
        y_max = float(np.max(y_local))
        pad = max(0.2, 0.08 * (y_max - y_min + 1e-9))
        axins.set_ylim(y_min - pad, y_max + pad)
    axins.grid(True, alpha=0.4)

    delay_info = find_delay_by_reference_level(
        t_s=t_s,
        q_cmd_deg=q_cmd_j4_deg,
        q_fb_deg=q_fb_j4_deg,
        ref_deg=float(ref_deg),
        window_s=(x1, x2),
        max_ref_error_deg=float(max_ref_error_deg),
    )
    annotate_delay_on_axis(axins, delay_info, text_offset=(0.0, 0.06))

    delay_text = f"{float(delay_info['delay_ms']):.2f} ms" if delay_info is not None else "N/A"
    axins.set_title(f"Zoom {x1:.1f}-{x2:.1f}s | delay@{float(ref_deg):.0f}deg = {delay_text}", fontsize=10)
    mark_inset(ax, axins, loc1=2, loc2=4, fc="none", ec="0.5")

    fig.tight_layout()
    fig.savefig(output_path, dpi=240)
    plt.close(fig)


def save_j1_j7_pos_cmd_fb_plot(
    t_s: np.ndarray,
    q_cmd_deg: np.ndarray,
    q_fb_deg: np.ndarray,
    output_path: str,
    annotate_delay: bool = False,
    delay_window_s: Tuple[float, float] = (9.5, 10.5),
    delay_ref_deg: float = -90.0,
    max_ref_error_deg: float = 2.0,
) -> None:
    t_s = np.asarray(t_s, dtype=np.float64).reshape(-1)
    q_cmd_deg = np.asarray(q_cmd_deg, dtype=np.float64)
    q_fb_deg = np.asarray(q_fb_deg, dtype=np.float64)

    if q_cmd_deg.ndim != 2 or q_cmd_deg.shape[1] != 7:
        raise ValueError(f"q_cmd_deg must be (N, 7), got {q_cmd_deg.shape}")
    if q_fb_deg.ndim != 2 or q_fb_deg.shape[1] != 7:
        raise ValueError(f"q_fb_deg must be (N, 7), got {q_fb_deg.shape}")

    n = min(len(t_s), len(q_cmd_deg), len(q_fb_deg))
    if n < 2:
        raise ValueError("Insufficient samples for J1-J7 position plot")

    t_s = t_s[:n]
    q_cmd_deg = q_cmd_deg[:n, :]
    q_fb_deg = q_fb_deg[:n, :]

    fig, axes = plt.subplots(7, 1, figsize=(15, 22), dpi=180, sharex=True)

    for j in range(7):
        ax = axes[j]
        ax.plot(t_s, q_cmd_deg[:, j], linewidth=1.8, label=f"J{j + 1} Command")
        ax.plot(t_s, q_fb_deg[:, j], linewidth=1.4, label=f"J{j + 1} Feedback")
        if annotate_delay:
            delay_info = find_delay_by_reference_level(
                t_s=t_s,
                q_cmd_deg=q_cmd_deg[:, j],
                q_fb_deg=q_fb_deg[:, j],
                ref_deg=float(delay_ref_deg),
                window_s=(float(delay_window_s[0]), float(delay_window_s[1])),
                max_ref_error_deg=float(max_ref_error_deg),
            )
            annotate_delay_on_axis(ax, delay_info)
            delay_text = f"{float(delay_info['delay_ms']):.2f} ms" if delay_info is not None else "N/A"
            ax.set_title(f"J{j + 1} Position | delay@{float(delay_ref_deg):.0f}deg: {delay_text}")
        else:
            ax.set_title(f"J{j + 1} Position")
        ax.set_ylabel("deg")
        ax.grid(True, alpha=0.35)
        ax.legend(loc="best")

    axes[-1].set_xlabel("time (s)")
    axes[-1].set_xlim(float(t_s[0]), float(t_s[-1]))

    fig.tight_layout(h_pad=1.4)
    fig.savefig(output_path, dpi=260)
    plt.close(fig)


def run_postprocess_plots(
    kine: Marvin_Kine,
    tracking_data: Dict[str, object],
    result_dir: str,
    artifact_prefix: str,
    max_lag_s: float,
    j4_fundamental_hz: Optional[float],
    analysis_start_s: float,
    j4_vel_smooth_window_s: float,
) -> Dict[str, object]:
    try:
        cmd_t_s = np.asarray(tracking_data.get("cmd_time_s", []), dtype=np.float64).reshape(-1)
        cmd_q_deg = _to_float_array_2d(tracking_data.get("cmd_joint_deg", []), cols=7)
        cmd_xyz_mm = _to_float_array_2d(tracking_data.get("cmd_xyz_mm", []), cols=3)

        fb_t_s = np.asarray(tracking_data.get("fb_time_s", []), dtype=np.float64).reshape(-1)
        fb_q_deg = _to_float_array_2d(tracking_data.get("fb_joint_deg", []), cols=7)
    except Exception as exc:
        return {
            "ok": False,
            "reason": f"tracking_data parse failed: {exc}",
        }

    n_cmd_raw = min(len(cmd_t_s), len(cmd_q_deg), len(cmd_xyz_mm))
    n_fb_raw = min(len(fb_t_s), len(fb_q_deg))
    if n_cmd_raw < 10 or n_fb_raw < 10:
        return {
            "ok": False,
            "reason": "insufficient tracking samples",
            "samples": {
                "command_raw": int(n_cmd_raw),
                "feedback_joint_raw": int(n_fb_raw),
            },
        }

    cmd_t_s = cmd_t_s[:n_cmd_raw]
    cmd_q_deg = cmd_q_deg[:n_cmd_raw, :]
    cmd_xyz_mm = cmd_xyz_mm[:n_cmd_raw, :]

    fb_t_s = fb_t_s[:n_fb_raw]
    fb_q_deg = fb_q_deg[:n_fb_raw, :]

    if analysis_start_s > 0.0:
        cmd_mask = cmd_t_s >= float(analysis_start_s)
        fb_mask = fb_t_s >= float(analysis_start_s)
        cmd_t_s = cmd_t_s[cmd_mask]
        cmd_q_deg = cmd_q_deg[cmd_mask, :]
        cmd_xyz_mm = cmd_xyz_mm[cmd_mask, :]
        fb_t_s = fb_t_s[fb_mask]
        fb_q_deg = fb_q_deg[fb_mask, :]

    n_cmd = int(len(cmd_t_s))
    n_fb = int(len(fb_t_s))
    if n_cmd < 10 or n_fb < 10:
        return {
            "ok": False,
            "reason": "insufficient samples after analysis_start_s filter",
            "samples": {
                "command_raw": int(n_cmd_raw),
                "feedback_joint_raw": int(n_fb_raw),
                "command_post_start": int(n_cmd),
                "feedback_joint_post_start": int(n_fb),
                "analysis_start_s": float(analysis_start_s),
            },
        }

    fb_xyz_t_s_list: List[float] = []
    fb_xyz_mm_list: List[List[float]] = []
    for ts, q in zip(fb_t_s, fb_q_deg):
        fk_mat = kine.fk([float(v) for v in q.tolist()])
        if not fk_mat:
            continue
        pose_mm_deg = kine.mat4x4_to_xyzabc(fk_mat)
        if not pose_mm_deg:
            continue
        xyz_mm = [float(pose_mm_deg[0]), float(pose_mm_deg[1]), float(pose_mm_deg[2])]
        if not np.all(np.isfinite(xyz_mm)):
            continue
        fb_xyz_t_s_list.append(float(ts))
        fb_xyz_mm_list.append(xyz_mm)

    fb_xyz_t_s = np.asarray(fb_xyz_t_s_list, dtype=np.float64)
    fb_xyz_mm = _to_float_array_2d(fb_xyz_mm_list, cols=3)

    cart_plot_path: Optional[str] = None
    cart_err_plot_path: Optional[str] = None
    cart_lag_s: Optional[float] = None
    cart_err_stats: Dict[str, Optional[float]] = {
        "mean_mm": None,
        "max_mm": None,
        "p95_mm": None,
    }
    if len(cmd_xyz_mm) >= 10 and len(fb_xyz_mm) >= 10:
        cart_lag_s = estimate_lag_s_cartesian(
            t_cmd_s=cmd_t_s,
            cmd_xyz_mm=cmd_xyz_mm,
            t_fb_s=fb_xyz_t_s,
            fb_xyz_mm=fb_xyz_mm,
            max_lag_s=float(max_lag_s),
        )
        cart_plot_path = os.path.join(result_dir, f"{artifact_prefix}_traj3d_cmd_vs_fb.png")
        save_trajectory_3d_plot(
            cmd_xyz_m=cmd_xyz_mm / 1000.0,
            fb_xyz_m=fb_xyz_mm / 1000.0,
            output_path=cart_plot_path,
            cart_delay_ms=(float(cart_lag_s) * 1000.0) if cart_lag_s is not None else None,
        )

        t_cmd_xyz, cmd_xyz_clean = _sanitize_time_and_values(cmd_t_s, cmd_xyz_mm)
        t_fb_xyz, fb_xyz_clean = _sanitize_time_and_values(fb_xyz_t_s, fb_xyz_mm)
        t_grid_xyz, _ = _build_common_time_grid(t_cmd_xyz, t_fb_xyz)
        if len(t_grid_xyz) >= 10:
            cmd_xyz_i = _interp_to_grid(t_cmd_xyz, cmd_xyz_clean, t_grid_xyz)
            fb_xyz_i = _interp_to_grid(t_fb_xyz, fb_xyz_clean, t_grid_xyz)
            err_mm = np.linalg.norm(fb_xyz_i - cmd_xyz_i, axis=1)

            cart_err_plot_path = os.path.join(result_dir, f"{artifact_prefix}_cart_error_vs_time.png")
            save_cartesian_error_plot(t_grid_xyz, err_mm, cart_err_plot_path)

            if len(err_mm) > 0:
                cart_err_stats = {
                    "mean_mm": float(np.mean(err_mm)),
                    "max_mm": float(np.max(err_mm)),
                    "p95_mm": float(np.percentile(err_mm, 95)),
                }

    j4_plot_path: Optional[str] = None
    j4_err_plot_path: Optional[str] = None
    j1_j7_plot_path: Optional[str] = None
    j4_zoom_delay_plot_path: Optional[str] = None
    j1_j7_delay_plot_path: Optional[str] = None
    j4_lag_s, j4_f0_used_hz = estimate_lag_s_phase_fundamental(
        t_cmd_s=cmd_t_s,
        cmd_1d=cmd_q_deg[:, 3],
        t_fb_s=fb_t_s,
        fb_1d=fb_q_deg[:, 3],
        fundamental_hz=j4_fundamental_hz,
        max_lag_s=float(max_lag_s),
    )

    cart_delay_ms = round(float(cart_lag_s) * 1000.0, 4) if cart_lag_s is not None else None
    j4_delay_ms = round(float(j4_lag_s) * 1000.0, 4) if j4_lag_s is not None else None

    t_cmd_clean, cmd_q_clean = _sanitize_time_and_values(cmd_t_s, cmd_q_deg)
    t_fb_clean, fb_q_clean = _sanitize_time_and_values(fb_t_s, fb_q_deg)
    t_grid_s, _ = _build_common_time_grid(t_cmd_clean, t_fb_clean)
    if len(t_grid_s) >= 10:
        cmd_q_i = _interp_to_grid(t_cmd_clean, cmd_q_clean, t_grid_s)
        fb_q_i = _interp_to_grid(t_fb_clean, fb_q_clean, t_grid_s)
        j4_cmd_i = cmd_q_i[:, 3]
        j4_fb_i = fb_q_i[:, 3]

        j1_j7_plot_path = os.path.join(result_dir, f"{artifact_prefix}_j1_j7_pos_cmd_vs_fb.png")
        save_j1_j7_pos_cmd_fb_plot(
            t_s=t_grid_s,
            q_cmd_deg=cmd_q_i,
            q_fb_deg=fb_q_i,
            output_path=j1_j7_plot_path,
            annotate_delay=False,
        )

        j1_j7_delay_plot_path = os.path.join(result_dir, f"{artifact_prefix}_j1_j7_pos_cmd_vs_fb_delay.png")
        save_j1_j7_pos_cmd_fb_plot(
            t_s=t_grid_s,
            q_cmd_deg=cmd_q_i,
            q_fb_deg=fb_q_i,
            output_path=j1_j7_delay_plot_path,
            annotate_delay=True,
            delay_window_s=(9.5, 10.5),
            delay_ref_deg=-90.0,
            max_ref_error_deg=2.0,
        )

        j4_plot_path = os.path.join(result_dir, f"{artifact_prefix}_j4_pos_vel_acc.png")
        save_j4_pos_vel_acc_plot(t_grid_s, j4_cmd_i, j4_fb_i, j4_plot_path)

        j4_zoom_delay_plot_path = os.path.join(result_dir, f"{artifact_prefix}_j4_position_zoom_delay.png")
        save_j4_position_zoom_delay_plot(
            t_s=t_grid_s,
            q_cmd_j4_deg=j4_cmd_i,
            q_fb_j4_deg=j4_fb_i,
            output_path=j4_zoom_delay_plot_path,
            zoom_window_s=(9.5, 10.5),
            ref_deg=-90.0,
            max_ref_error_deg=2.0,
        )

        j4_err_plot_path = os.path.join(result_dir, f"{artifact_prefix}_j4_error_triplet.png")
        save_j4_error_triplet_plot(
            t_s=t_grid_s,
            q_cmd_j4_deg=j4_cmd_i,
            q_fb_j4_deg=j4_fb_i,
            vel_smooth_window_s=float(j4_vel_smooth_window_s),
            output_path=j4_err_plot_path,
        )

    return {
        "ok": True,
        "samples": {
            "command_raw": int(n_cmd_raw),
            "feedback_joint_raw": int(n_fb_raw),
            "command_post_start": int(n_cmd),
            "feedback_joint_post_start": int(n_fb),
            "feedback_cartesian_valid": int(len(fb_xyz_mm)),
            "analysis_start_s": float(analysis_start_s),
        },
        "xcorr_max_lag_s": float(max_lag_s),
        "delay_methods": {
            "cartesian_xyz": "xcorr_pca_projection",
            "j4_position": "fundamental_phase_alignment",
        },
        "j4_fundamental_hz_used": round(float(j4_f0_used_hz), 6) if j4_f0_used_hz is not None else None,
        "j4_vel_smooth_window_s": float(j4_vel_smooth_window_s),
        "delays": {
            "cartesian_xyz_xcorr_ms": cart_delay_ms,
            "j4_position_fundamental_phase_ms": j4_delay_ms,
        },
        "cartesian_tracking_error_mm": cart_err_stats,
        "plots": {
            "cartesian_3d_cmd_vs_feedback": cart_plot_path,
            "cartesian_error_vs_time": cart_err_plot_path,
            "j1_j7_pos_cmd_vs_feedback": j1_j7_plot_path,
            "j1_j7_pos_cmd_vs_feedback_delay": j1_j7_delay_plot_path,
            "j4_pos_vel_acc": j4_plot_path,
            "j4_position_zoom_delay": j4_zoom_delay_plot_path,
            "j4_error_triplet": j4_err_plot_path,
        },
    }


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
            self.enabled = (ret == 0)
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
    while True:
        now = time.perf_counter()
        remain = deadline_s - now
        if remain <= 0.0:
            return
        if remain > spin_threshold_s:
            time.sleep(remain - spin_threshold_s)


def resolve_path(path_str: str) -> str:
    p = os.path.expanduser(path_str)
    if os.path.isabs(p):
        return p
    return os.path.join(current_dir, p)


def send_joint_command(robot: Marvin_Robot, arm: str, joints_deg: List[float]) -> None:
    robot.clear_set()
    robot.set_joint_cmd_pose(arm=arm, joints=joints_deg)
    robot.send_cmd()


def safe_connect_and_clear(robot: Marvin_Robot, dcss: DCSS, robot_ip: str, arm: str) -> None:
    if robot.connect(robot_ip) == 0:
        raise RuntimeError("Robot connect failed: port occupied or network issue")

    time.sleep(0.5)
    robot.clear_set()
    robot.clear_error("A")
    robot.clear_error("B")
    robot.send_cmd()
    time.sleep(0.5)

    robot.log_switch("0")
    robot.local_log_switch("0")

    idx = arm_to_index(arm)
    motion_tag = 0
    frame_update = None
    for _ in range(5):
        sub_data = robot.subscribe(dcss)
        frame_serial = int(sub_data["outputs"][idx]["frame_serial"])
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
    joint_k: List[float],
    joint_d: List[float],
    ctrl_hz: int,
) -> None:
    robot.clear_set()
    robot.set_vel_acc(arm=arm, velRatio=int(vel_ratio), AccRatio=int(acc_ratio))
    if hasattr(robot, "set_joint_kd_params"):
        robot.set_joint_kd_params(arm=arm, K=[float(v) for v in joint_k], D=[float(v) for v in joint_d])
    else:
        print("[warn] set_joint_kd_params not available, continue with state switch only")
    robot.send_cmd()
    time.sleep(0.2)

    robot.clear_set()
    robot.set_state(arm=arm, state=3)
    if hasattr(robot, "set_impedance_type"):
        try:
            robot.set_impedance_type(arm=arm, type=1)
        except TypeError:
            try:
                robot.set_impedance_type(arm, 1)
            except Exception:
                pass

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
    q = sub_data["outputs"][idx]["fb_joint_pos"]
    return [float(v) for v in q]


def get_current_pose_mm_deg(kine: Marvin_Kine, robot: Marvin_Robot, dcss: DCSS, arm: str) -> List[float]:
    q_now = get_current_joints(robot, dcss, arm)
    fk_mat = kine.fk(q_now)
    if not fk_mat:
        raise RuntimeError("FK failed while reading current pose")
    pose_mm_deg = kine.mat4x4_to_xyzabc(fk_mat)
    if not pose_mm_deg:
        raise RuntimeError("mat4x4_to_xyzabc failed while reading current pose")
    return [float(v) for v in pose_mm_deg]


def move_to_joint_target_with_settle(
    robot: Marvin_Robot,
    dcss: DCSS,
    arm: str,
    target_joints_deg: List[float],
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
            print(f"[home] reached default joints, max|error|={last_err:.3f} deg")
            return

        time.sleep(period_s)

    raise RuntimeError(
        f"Move to default joints timeout ({timeout_s:.1f}s), "
        f"last max|error|={float(last_err) if last_err is not None else float('nan'):.3f} deg"
    )


def solve_ik_pose_mm_deg(kine: Marvin_Kine, pose_mm_deg: List[float], ref_joints_deg: List[float]) -> Optional[List[float]]:
    sp = FX_InvKineSolvePara()
    pose_mat = kine.xyzabc_to_mat4x4(pose_mm_deg)
    if not pose_mat:
        return None

    pose_mat_1x16 = kine.mat4x4_to_mat1x16(pose_mat)
    sp.set_input_ik_target_tcp(pose_mat_1x16)
    sp.set_input_ik_ref_joint(ref_joints_deg)

    ik_result = kine.ik(structure_data=sp)
    if not ik_result:
        return None

    return [float(v) for v in ik_result.m_Output_RetJoint.to_list()]


def plan_movla_points_compat(
    kine: Marvin_Kine,
    start_xyzabc_mm_deg: List[float],
    end_xyzabc_mm_deg: List[float],
    ref_joints_deg: List[float],
    vel_mm_s: float,
    acc_mm_s2: float,
    plan_hz: int,
) -> List[List[float]]:
    movla_pset = None
    try:
        try:
            movla_ret = kine.movLA(
                start_xyzabc=start_xyzabc_mm_deg,
                end_xyzabc=end_xyzabc_mm_deg,
                ref_joints=ref_joints_deg,
                vel=float(vel_mm_s),
                acc=float(acc_mm_s2),
                freq_hz=int(plan_hz),
            )
        except TypeError:
            movla_ret = kine.movLA(
                start_xyzabc=start_xyzabc_mm_deg,
                end_xyzabc=end_xyzabc_mm_deg,
                ref_joints=ref_joints_deg,
                vel=float(vel_mm_s),
                acc=float(acc_mm_s2),
            )

        points_raw = movla_ret
        if isinstance(movla_ret, tuple):
            if len(movla_ret) >= 1:
                points_raw = movla_ret[0]
            if len(movla_ret) >= 2:
                movla_pset = movla_ret[1]

        if not points_raw:
            return []

        points: List[List[float]] = []
        for row in points_raw:
            if len(row) != 7:
                raise RuntimeError(f"MOVLA point length must be 7, got {len(row)}")
            points.append([float(v) for v in row])
        return points
    finally:
        if movla_pset is not None and hasattr(kine, "destroy_point_set"):
            try:
                kine.destroy_point_set(movla_pset)
            except Exception:
                pass


def execute_joint_points(robot: Marvin_Robot, arm: str, points: List[List[float]], send_hz: float) -> None:
    if not points:
        raise RuntimeError("Point list is empty")
    period_s = 1.0 / max(float(send_hz), 1.0)
    next_deadline = time.perf_counter()
    for q in points:
        sleep_until(next_deadline)
        send_joint_command(robot, arm, q)
        next_deadline += period_s


def build_circle_poses_mm_deg(
    center_xyz_m: List[float],
    abc_deg: List[float],
    radius_m: float,
    plane: str,
    total_points: int,
    points_per_cycle: int,
    circle_direction: int = 1,
) -> List[List[float]]:
    if len(center_xyz_m) != 3:
        raise ValueError("center_xyz_m must have 3 values")
    if len(abc_deg) != 3:
        raise ValueError("abc_deg must have 3 values")
    if total_points <= 0 or points_per_cycle <= 0:
        raise ValueError("total_points and points_per_cycle must be positive")

    cx, cy, cz = [float(v) for v in center_xyz_m]
    a_deg, b_deg, c_deg = [float(v) for v in abc_deg]
    r = float(radius_m)

    poses: List[List[float]] = []
    for i in range(total_points):
        direction = 1.0 if int(circle_direction) >= 0 else -1.0
        theta = direction * 2.0 * math.pi * (float(i % points_per_cycle) / float(points_per_cycle))

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        x, y, z = cx, cy, cz
        if plane == "xy":
            x = cx + r * cos_t
            y = cy + r * sin_t
        elif plane == "xz":
            x = cx + r * cos_t
            z = cz + r * sin_t
        elif plane == "yz":
            y = cy + r * cos_t
            z = cz + r * sin_t
        else:
            raise ValueError(f"Unsupported plane: {plane}")

        poses.append([x * 1000.0, y * 1000.0, z * 1000.0, a_deg, b_deg, c_deg])

    return poses


def apply_fade_in_to_trajectory(
    joint_points_deg: List[List[float]],
    cart_points_mm_deg: List[List[float]],
    ctrl_hz: int,
    cycle_s: float,
    fade_in_s: float,
) -> Tuple[List[List[float]], List[List[float]]]:
    if not joint_points_deg or not cart_points_mm_deg:
        return joint_points_deg, cart_points_mm_deg

    if fade_in_s <= 0.0:
        return joint_points_deg, cart_points_mm_deg

    n = min(len(joint_points_deg), len(cart_points_mm_deg))
    q0 = np.asarray(joint_points_deg[0], dtype=np.float64)
    p0 = np.asarray(cart_points_mm_deg[0], dtype=np.float64)
    hz = max(int(ctrl_hz), 1)
    fade_den = max(float(fade_in_s), 1e-9)

    q_faded: List[List[float]] = []
    p_faded: List[List[float]] = []

    for i in range(n):
        t_elapsed = float(i) / float(hz)

        if t_elapsed <= float(fade_in_s):
            gain = max(0.0, min(1.0, t_elapsed / fade_den))
        else:
            gain = 1.0

        qi = np.asarray(joint_points_deg[i], dtype=np.float64)
        pi = np.asarray(cart_points_mm_deg[i], dtype=np.float64)
        q_faded.append((q0 + gain * (qi - q0)).tolist())
        p_faded.append((p0 + gain * (pi - p0)).tolist())

    return q_faded, p_faded


def execute_timed_joint_trajectory(
    robot: Marvin_Robot,
    dcss: DCSS,
    arm: str,
    joint_points_deg: List[List[float]],
    cart_points_mm_deg: List[List[float]],
    ctrl_hz: int,
) -> Dict[str, object]:
    if not joint_points_deg:
        raise RuntimeError("joint_points_deg is empty")

    period_s = 1.0 / float(ctrl_hz)
    dt_samples: List[float] = []
    jitter_samples: List[float] = []
    overrun_count = 0
    negative_sleep_count = 0
    prev_send_ts: Optional[float] = None

    cmd_time_s: List[float] = []
    cmd_joint_deg: List[List[float]] = []
    cmd_xyz_mm: List[List[float]] = []
    fb_time_s: List[float] = []
    fb_joint_deg: List[List[float]] = []
    arm_idx = arm_to_index(arm)

    hp_timer = WindowsHighPrecisionTimer(period_ms=1)
    hp_timer.start()
    hp_timer_started = bool(hp_timer.enabled)

    t0 = time.perf_counter()
    next_deadline = t0

    try:
        for i, q_cmd in enumerate(joint_points_deg):
            now = time.perf_counter()
            if now < next_deadline:
                sleep_until(next_deadline)
            else:
                negative_sleep_count += 1
                if (now - next_deadline) >= period_s:
                    overrun_count += 1

            send_joint_command(robot, arm, q_cmd)
            send_ts = time.perf_counter()

            jitter_samples.append(send_ts - next_deadline)
            if prev_send_ts is not None:
                dt_samples.append(send_ts - prev_send_ts)
            prev_send_ts = send_ts

            cmd_time_s.append(float(send_ts - t0))
            cmd_joint_deg.append([float(v) for v in q_cmd])
            if i < len(cart_points_mm_deg) and len(cart_points_mm_deg[i]) >= 3:
                cmd_xyz_mm.append([
                    float(cart_points_mm_deg[i][0]),
                    float(cart_points_mm_deg[i][1]),
                    float(cart_points_mm_deg[i][2]),
                ])
            else:
                cmd_xyz_mm.append([float("nan"), float("nan"), float("nan")])

            try:
                sub_data = robot.subscribe(dcss)
                q_fb = sub_data["outputs"][arm_idx]["fb_joint_pos"]
                if q_fb is not None and len(q_fb) == 7:
                    fb_time_s.append(float(time.perf_counter() - t0))
                    fb_joint_deg.append([float(v) for v in q_fb])
            except Exception:
                pass

            next_deadline += period_s
    finally:
        hp_timer.stop()

    elapsed_s = time.perf_counter() - t0

    return {
        "high_precision_timer_enabled": hp_timer_started,
        "elapsed_s": float(elapsed_s),
        "expected_s": float(len(joint_points_deg) / float(ctrl_hz)),
        "points_sent": int(len(joint_points_deg)),
        "ctrl_hz": int(ctrl_hz),
        "negative_sleep_count": int(negative_sleep_count),
        "overrun_count": int(overrun_count),
        "timing_stats": {
            "command_dt": stats_ms(dt_samples),
            "command_jitter": stats_ms(jitter_samples),
        },
        "tracking_data": {
            "cmd_time_s": cmd_time_s,
            "cmd_joint_deg": cmd_joint_deg,
            "cmd_xyz_mm": cmd_xyz_mm,
            "fb_time_s": fb_time_s,
            "fb_joint_deg": fb_joint_deg,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Cartesian circle test with IK, timing control, and joint-impedance formal run")
    parser.add_argument("--robot_ip", type=str, default=str(CONFIG_DEFAULTS["robot_ip"]), help="Robot IP")
    parser.add_argument("--arm", choices=["A", "B"], default=str(CONFIG_DEFAULTS["arm"]), help="Arm selector")
    parser.add_argument("--kine_cfg", type=str, default=str(CONFIG_DEFAULTS["kine_cfg"]), help="Kinematics config path")

    parser.add_argument("--default_joints", nargs=7, type=float, default=None, help="Default joints in deg")
    parser.add_argument("--vel_ratio", type=int, default=int(CONFIG_DEFAULTS["vel_ratio"]), help="Position mode velocity ratio")
    parser.add_argument("--acc_ratio", type=int, default=int(CONFIG_DEFAULTS["acc_ratio"]), help="Position mode acceleration ratio")
    parser.add_argument("--home_send_hz", type=float, default=float(CONFIG_DEFAULTS["home_send_hz"]), help="Send rate for default move")
    parser.add_argument("--home_timeout_s", type=float, default=float(CONFIG_DEFAULTS["home_timeout_s"]), help="Timeout for default move")
    parser.add_argument("--home_tol_deg", type=float, default=float(CONFIG_DEFAULTS["home_tol_deg"]), help="Joint tolerance for default move")
    parser.add_argument("--home_stable_samples", type=int, default=int(CONFIG_DEFAULTS["home_stable_samples"]), help="Stable samples for default move")
    parser.add_argument("--pre_wait_s", type=float, default=float(CONFIG_DEFAULTS["pre_wait_s"]), help="Wait time after reaching default")

    parser.add_argument("--center", nargs=3, type=float, default=None, help="Circle center xyz in meters")
    parser.add_argument("--abc", nargs=3, type=float, default=None, help="Fixed abc in deg")
    parser.add_argument("--radius_m", type=float, default=float(CONFIG_DEFAULTS["radius_m"]), help="Circle radius in meters")
    parser.add_argument("--plane", choices=["xy", "xz", "yz"], default=str(CONFIG_DEFAULTS["plane"]), help="Circle plane")

    parser.add_argument("--ctrl_hz", type=int, default=int(CONFIG_DEFAULTS["ctrl_hz"]), help="Control frequency in Hz")
    parser.add_argument("--duration_s", type=float, default=float(CONFIG_DEFAULTS["duration_s"]), help="Trajectory duration in seconds")
    parser.add_argument("--cycle_s", type=float, default=float(CONFIG_DEFAULTS["cycle_s"]), help="Seconds per lap")
    parser.add_argument("--ik_ref", nargs=7, type=float, default=None, help="IK initial reference joints in deg")
    parser.add_argument(
        "--joint_k",
        nargs=7,
        type=float,
        default=JOINT_IMP_DEFAULT_K,
        help="Joint stiffness K for joint impedance mode",
    )
    parser.add_argument(
        "--joint_d",
        nargs=7,
        type=float,
        default=JOINT_IMP_DEFAULT_D,
        help="Joint damping D for joint impedance mode",
    )

    parser.add_argument("--movla_vel_mm_s", type=float, default=float(CONFIG_DEFAULTS["movla_vel_mm_s"]), help="MOVLA planning velocity")
    parser.add_argument("--movla_acc_mm_s2", type=float, default=float(CONFIG_DEFAULTS["movla_acc_mm_s2"]), help="MOVLA planning acceleration")
    parser.add_argument("--movla_plan_hz", type=int, default=int(CONFIG_DEFAULTS["movla_plan_hz"]), help="MOVLA planning frequency")
    parser.add_argument("--movla_send_hz", type=float, default=float(CONFIG_DEFAULTS["movla_send_hz"]), help="MOVLA execution send rate")

    parser.add_argument("--result_dir", type=str, default=str(CONFIG_DEFAULTS["result_dir"]), help="Result directory")
    parser.add_argument("--tag", type=str, default=str(CONFIG_DEFAULTS["tag"]), help="Result filename tag")
    parser.add_argument("--xcorr_max_lag_s", type=float, default=float(CONFIG_DEFAULTS["xcorr_max_lag_s"]), help="Max lag window for xcorr delay")
    parser.add_argument(
        "--j4_fundamental_hz",
        type=float,
        default=CONFIG_DEFAULTS["j4_fundamental_hz"],
        help="Fundamental frequency for J4 phase-based delay (default: 1/cycle_s)",
    )
    parser.add_argument(
        "--fade_in_s",
        type=float,
        default=float(CONFIG_DEFAULTS["fade_in_s"]),
        help="Apply fade-in on first window with gain=2*dt/T",
    )
    parser.add_argument(
        "--analysis_start_s",
        type=float,
        default=float(CONFIG_DEFAULTS["analysis_start_s"]),
        help="Postprocess starts from this time (s)",
    )
    parser.add_argument(
        "--j4_vel_smooth_window_s",
        type=float,
        default=float(CONFIG_DEFAULTS["j4_vel_smooth_window_s"]),
        help="Smoothing window (seconds) for differenced J4 velocity before error calculation",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.ctrl_hz <= 0:
        raise ValueError("ctrl_hz must be > 0")
    if args.duration_s <= 0.0:
        raise ValueError("duration_s must be > 0")
    if args.cycle_s <= 0.0:
        raise ValueError("cycle_s must be > 0")
    if args.radius_m <= 0.0:
        raise ValueError("radius_m must be > 0")
    if args.xcorr_max_lag_s <= 0.0:
        raise ValueError("xcorr_max_lag_s must be > 0")
    if args.j4_fundamental_hz is not None and args.j4_fundamental_hz <= 0.0:
        raise ValueError("j4_fundamental_hz must be > 0")
    if args.fade_in_s < 0.0:
        raise ValueError("fade_in_s must be >= 0")
    if args.analysis_start_s < 0.0:
        raise ValueError("analysis_start_s must be >= 0")
    if args.j4_vel_smooth_window_s < 0.0:
        raise ValueError("j4_vel_smooth_window_s must be >= 0")

    points_per_cycle = int(round(float(args.cycle_s) * float(args.ctrl_hz)))
    total_points = int(round(float(args.duration_s) * float(args.ctrl_hz)))
    if points_per_cycle <= 0 or total_points <= 0:
        raise ValueError("computed points_per_cycle/total_points must be > 0")

    if args.default_joints is None:
        default_joints_deg = get_default_joints_for_arm(args.arm)
    else:
        default_joints_deg = [float(v) for v in args.default_joints]

    ik_ref_deg = [float(v) for v in args.ik_ref] if args.ik_ref is not None else default_joints_deg.copy()
    j4_fundamental_hz = float(args.j4_fundamental_hz) if args.j4_fundamental_hz is not None else (1.0 / float(args.cycle_s))

    kine_cfg_path = resolve_path(args.kine_cfg)
    result_dir = resolve_path(args.result_dir)
    os.makedirs(result_dir, exist_ok=True)

    print("[step1] initialize kinematics")
    kine = Marvin_Kine()
    kine.log_switch(0)

    arm_type = arm_to_index(args.arm)
    cfg = kine.load_config(arm_type=arm_type, config_path=kine_cfg_path)
    if not cfg:
        raise RuntimeError(f"Failed to load kinematics config: {kine_cfg_path}")

    ok = kine.initial_kine(
        robot_type=cfg["TYPE"][arm_type],
        dh=cfg["DH"][arm_type],
        pnva=cfg["PNVA"][arm_type],
        j67=cfg["BD"][arm_type],
    )
    if not ok:
        raise RuntimeError("Failed to initialize kinematics")

    robot = Marvin_Robot()
    dcss = DCSS()

    runtime_result: Dict[str, object] = {}

    try:
        print("[step1-2] safe connect + clear errors")
        safe_connect_and_clear(robot=robot, dcss=dcss, robot_ip=args.robot_ip, arm=args.arm)

        print("[step3] enter position mode")
        enter_position_mode(robot, args.arm, args.vel_ratio, args.acc_ratio)

        print("[step4] move to default joints")
        print(f"[info] default_joints_deg={default_joints_deg}")
        move_to_joint_target_with_settle(
            robot=robot,
            dcss=dcss,
            arm=args.arm,
            target_joints_deg=default_joints_deg,
            send_hz=args.home_send_hz,
            timeout_s=args.home_timeout_s,
            tol_deg=args.home_tol_deg,
            stable_samples=args.home_stable_samples,
        )

        print(f"[step5] wait {args.pre_wait_s:.1f}s")
        time.sleep(float(args.pre_wait_s))

        pose_now_mm_deg = get_current_pose_mm_deg(kine, robot, dcss, args.arm)
        center_xyz_m = [float(v) for v in args.center] if args.center is not None else [
            pose_now_mm_deg[0] / 1000.0,
            pose_now_mm_deg[1] / 1000.0,
            pose_now_mm_deg[2] / 1000.0,
        ]
        abc_deg = [float(v) for v in args.abc] if args.abc is not None else [
            float(pose_now_mm_deg[3]),
            float(pose_now_mm_deg[4]),
            float(pose_now_mm_deg[5]),
        ]

        print("[step7] build Cartesian circle points")
        print(
            "[info] "
            f"center_m={center_xyz_m}, radius_m={args.radius_m}, plane={args.plane}, abc_deg={abc_deg}"
        )
        print(
            "[info] "
            f"ctrl_hz={args.ctrl_hz}, duration_s={args.duration_s}, cycle_s={args.cycle_s}, "
            f"points_per_cycle={points_per_cycle}, total_points={total_points}"
        )
        print(
            "[info] "
            f"fade_in_s={args.fade_in_s}, analysis_start_s={args.analysis_start_s}"
        )

        circle_poses_mm_deg = build_circle_poses_mm_deg(
            center_xyz_m=center_xyz_m,
            abc_deg=abc_deg,
            radius_m=float(args.radius_m),
            plane=args.plane,
            total_points=total_points,
            points_per_cycle=points_per_cycle,
        )

        print("[step8] IK solve for full control point list")
        q_circle_deg: List[List[float]] = []
        ref_j = ik_ref_deg.copy()
        for i, pose_mm_deg in enumerate(circle_poses_mm_deg):
            q_sol = solve_ik_pose_mm_deg(kine, pose_mm_deg, ref_j)
            if q_sol is None:
                raise RuntimeError(f"IK failed at point index {i}")
            q_circle_deg.append(q_sol)
            ref_j = q_sol
            if (i + 1) % 200 == 0 or (i + 1) == len(circle_poses_mm_deg):
                print(f"  IK progress: {i + 1}/{len(circle_poses_mm_deg)}")

        print("[step9] MOVLA to first circle point")
        pose_before_circle_mm_deg = get_current_pose_mm_deg(kine, robot, dcss, args.arm)
        q_before_circle_deg = get_current_joints(robot, dcss, args.arm)
        first_pose_mm_deg = circle_poses_mm_deg[0]

        points_to_first = plan_movla_points_compat(
            kine=kine,
            start_xyzabc_mm_deg=pose_before_circle_mm_deg,
            end_xyzabc_mm_deg=first_pose_mm_deg,
            ref_joints_deg=q_before_circle_deg,
            vel_mm_s=float(args.movla_vel_mm_s),
            acc_mm_s2=float(args.movla_acc_mm_s2),
            plan_hz=int(args.movla_plan_hz),
        )
        if not points_to_first:
            raise RuntimeError("MOVLA planning to first circle point returned empty points")
        execute_joint_points(robot, args.arm, points_to_first, send_hz=args.movla_send_hz)

        print("[step6] hot switch to joint impedance mode (formal circle run)")
        enter_joint_impedance_mode(
            robot=robot,
            arm=args.arm,
            vel_ratio=args.vel_ratio,
            acc_ratio=args.acc_ratio,
            joint_k=[float(v) for v in args.joint_k],
            joint_d=[float(v) for v in args.joint_d],
            ctrl_hz=int(args.ctrl_hz),
        )

        print("[step6+8] high precision timer + sleep-until + 200Hz control")
        q_exec_deg, cart_exec_mm_deg = apply_fade_in_to_trajectory(
            joint_points_deg=q_circle_deg,
            cart_points_mm_deg=circle_poses_mm_deg,
            ctrl_hz=int(args.ctrl_hz),
            cycle_s=float(args.cycle_s),
            fade_in_s=float(args.fade_in_s),
        )
        runtime_result = execute_timed_joint_trajectory(
            robot=robot,
            dcss=dcss,
            arm=args.arm,
            joint_points_deg=q_exec_deg,
            cart_points_mm_deg=cart_exec_mm_deg,
            ctrl_hz=int(args.ctrl_hz),
        )

        print("[step8.5] switch back to position mode for MOVLA return")
        enter_position_mode(robot, args.arm, args.vel_ratio, args.acc_ratio)

        run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        artifact_prefix = f"{args.tag}_{run_stamp}"
        tracking_data = runtime_result.pop("tracking_data", {})
        postprocess = run_postprocess_plots(
            kine=kine,
            tracking_data=tracking_data,
            result_dir=result_dir,
            artifact_prefix=artifact_prefix,
            max_lag_s=float(args.xcorr_max_lag_s),
            j4_fundamental_hz=float(j4_fundamental_hz),
            analysis_start_s=float(args.analysis_start_s),
            j4_vel_smooth_window_s=float(args.j4_vel_smooth_window_s),
        )

        print("[step9] MOVLA back to default")
        pose_after_circle_mm_deg = get_current_pose_mm_deg(kine, robot, dcss, args.arm)

        fk_default = kine.fk(default_joints_deg)
        if not fk_default:
            raise RuntimeError("FK failed for default joints")
        default_pose_mm_deg = kine.mat4x4_to_xyzabc(fk_default)
        if not default_pose_mm_deg:
            raise RuntimeError("mat4x4_to_xyzabc failed for default joints")
        default_pose_mm_deg = [float(v) for v in default_pose_mm_deg]

        q_after_circle_deg = get_current_joints(robot, dcss, args.arm)
        points_back_home = plan_movla_points_compat(
            kine=kine,
            start_xyzabc_mm_deg=pose_after_circle_mm_deg,
            end_xyzabc_mm_deg=default_pose_mm_deg,
            ref_joints_deg=q_after_circle_deg,
            vel_mm_s=float(args.movla_vel_mm_s),
            acc_mm_s2=float(args.movla_acc_mm_s2),
            plan_hz=int(args.movla_plan_hz),
        )
        if not points_back_home:
            raise RuntimeError("MOVLA planning back to default returned empty points")
        execute_joint_points(robot, args.arm, points_back_home, send_hz=args.movla_send_hz)

        result = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "workflow": {
                "safe_connect": True,
                "clear_error": True,
                "position_mode": True,
                "home_default": True,
                "wait_5s": float(args.pre_wait_s),
                "high_precision_clock": True,
                "sleep_until": True,
                "cartesian_circle": True,
                "formal_mode_joint_impedance": True,
                "return_mode_position": True,
                "fade_in_applied": float(args.fade_in_s) > 0.0,
                "ik_used": True,
                "movla_to_first": True,
                "movla_back_default": True,
            },
            "config": {
                "robot_ip": args.robot_ip,
                "arm": args.arm,
                "kine_cfg": kine_cfg_path,
                "default_joints_deg": default_joints_deg,
                "center_xyz_m": center_xyz_m,
                "abc_deg": abc_deg,
                "radius_m": float(args.radius_m),
                "plane": args.plane,
                "ctrl_hz": int(args.ctrl_hz),
                "duration_s": float(args.duration_s),
                "cycle_s": float(args.cycle_s),
                "points_per_cycle": int(points_per_cycle),
                "total_points": int(total_points),
                "xcorr_max_lag_s": float(args.xcorr_max_lag_s),
                "j4_fundamental_hz": float(j4_fundamental_hz),
                "joint_k": [float(v) for v in args.joint_k],
                "joint_d": [float(v) for v in args.joint_d],
                "fade_in_s": float(args.fade_in_s),
                "analysis_start_s": float(args.analysis_start_s),
                "j4_vel_smooth_window_s": float(args.j4_vel_smooth_window_s),
            },
            "runtime": runtime_result,
            "postprocess": postprocess,
        }

        out_name = f"{artifact_prefix}.json"
        out_path = os.path.join(result_dir, out_name)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)

        print("[done] test completed")
        print(f"[done] result json: {out_path}")

        timing = runtime_result.get("timing_stats", {})
        dt_stats = timing.get("command_dt", {})
        jit_stats = timing.get("command_jitter", {})
        print(
            "[timing] command_dt(ms): "
            f"mean={dt_stats.get('mean_ms')}, std={dt_stats.get('std_ms')}, max={dt_stats.get('max_ms')}"
        )
        print(
            "[timing] jitter(ms): "
            f"mean={jit_stats.get('mean_ms')}, std={jit_stats.get('std_ms')}, max={jit_stats.get('max_ms')}"
        )

        post_delays = postprocess.get("delays", {}) if isinstance(postprocess, dict) else {}
        cart_delay_ms = post_delays.get("cartesian_xyz_xcorr_ms")
        j4_delay_ms = post_delays.get("j4_position_fundamental_phase_ms")
        print(
            "[analysis] cartesian_xyz_xcorr_delay(ms): "
            f"{float(cart_delay_ms):.4f}" if cart_delay_ms is not None else "[analysis] cartesian_xyz_xcorr_delay(ms): None"
        )
        print(
            "[analysis] j4_position_phase_delay(ms): "
            f"{float(j4_delay_ms):.4f}" if j4_delay_ms is not None else "[analysis] j4_position_phase_delay(ms): None"
        )

        j4_f0_used_hz = postprocess.get("j4_fundamental_hz_used") if isinstance(postprocess, dict) else None
        if j4_f0_used_hz is not None:
            print(f"[analysis] j4_fundamental_hz_used: {float(j4_f0_used_hz):.6f}")

        post_plots = postprocess.get("plots", {}) if isinstance(postprocess, dict) else {}
        if post_plots.get("cartesian_3d_cmd_vs_feedback"):
            print(f"[analysis] plot(3d): {post_plots.get('cartesian_3d_cmd_vs_feedback')}")
        if post_plots.get("cartesian_error_vs_time"):
            print(f"[analysis] plot(cart_error): {post_plots.get('cartesian_error_vs_time')}")
        if post_plots.get("j1_j7_pos_cmd_vs_feedback"):
            print(f"[analysis] plot(j1_j7_pos): {post_plots.get('j1_j7_pos_cmd_vs_feedback')}")
        if post_plots.get("j1_j7_pos_cmd_vs_feedback_delay"):
            print(f"[analysis] plot(j1_j7_pos_delay): {post_plots.get('j1_j7_pos_cmd_vs_feedback_delay')}")
        if post_plots.get("j4_pos_vel_acc"):
            print(f"[analysis] plot(j4): {post_plots.get('j4_pos_vel_acc')}")
        if post_plots.get("j4_position_zoom_delay"):
            print(f"[analysis] plot(j4_zoom_delay): {post_plots.get('j4_position_zoom_delay')}")
        if post_plots.get("j4_error_triplet"):
            print(f"[analysis] plot(j4_error_triplet): {post_plots.get('j4_error_triplet')}")

        return 0

    except KeyboardInterrupt:
        print("\n[abort] interrupted by user")
        return 130
    except Exception as exc:
        print(f"[error] {exc}")
        return 1
    finally:
        # step10: servo-off and release in finally
        try:
            robot.clear_set()
            robot.set_state(arm=args.arm, state=0)
            robot.send_cmd()
            time.sleep(0.2)
        except Exception as exc:
            print(f"[shutdown] set_state(0) failed: {exc}")

        try:
            if hasattr(robot, "disable"):
                disable_ok = robot.disable(args.arm)
                print(f"[shutdown] disable({args.arm}) -> {disable_ok}")
            else:
                print("[shutdown] disable API not available in current SDK wrapper, skip")
        except Exception as exc:
            print(f"[shutdown] disable failed: {exc}")

        try:
            robot.release_robot()
        except Exception as exc:
            print(f"[shutdown] release_robot failed: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
