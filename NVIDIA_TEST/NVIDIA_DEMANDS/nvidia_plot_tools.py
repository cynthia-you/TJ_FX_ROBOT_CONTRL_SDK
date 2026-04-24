import json
import os
import time
from typing import Any, Dict, Optional, Tuple

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


# 直接沿用旧脚本调试参数，保证导数曲线风格一致
JOINT_DEBUG_WIN_POS = 5
JOINT_DEBUG_WIN_VEL = 7
JOINT_DEBUG_WIN_ACC = 11
JOINT_MIN_SPAN_VEL_DEG_S = 5.0
JOINT_MIN_SPAN_ACC_DEG_S2 = 50.0
JOINT_ERROR_THRESHOLD_DEG = 15.0


def np_to_builtin(obj):
    """递归把 numpy 类型转换为 json 可序列化的内建类型。"""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, dict):
        return {k: np_to_builtin(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [np_to_builtin(v) for v in obj]
    return obj


def save_showcase_json(data, output_dir, file_name) -> str:
    """统一保存 json，返回文件绝对路径。"""
    os.makedirs(output_dir, exist_ok=True)
    save_path = os.path.join(output_dir, file_name)
    with open(save_path, "w", encoding="utf-8") as f:
        json.dump(np_to_builtin(data), f, ensure_ascii=False, indent=2)
    return save_path


def _infer_ctrl_hz(t_s: np.ndarray, ctrl_hz: Optional[float]) -> int:
    if ctrl_hz is not None and float(ctrl_hz) > 0:
        return max(1, int(round(float(ctrl_hz))))

    if t_s.size <= 1:
        return 100

    dt = np.diff(t_s)
    dt = dt[np.isfinite(dt) & (dt > 1e-9)]
    if dt.size == 0:
        return 100

    dt_med = float(np.median(dt))
    if dt_med <= 1e-9:
        return 100
    return max(1, int(round(1.0 / dt_med)))


def _moving_average_1d(x: np.ndarray, win: int) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64).reshape(-1)
    if x.size == 0:
        return x.copy()

    w = max(1, int(win))
    if w <= 1:
        return x.copy()
    if w % 2 == 0:
        w += 1
    if x.size < w:
        w = int(x.size)
        if w % 2 == 0:
            w = max(1, w - 1)
    if w <= 1:
        return x.copy()

    kernel = np.ones(w, dtype=np.float64) / float(w)
    pad = w // 2
    x_pad = np.pad(x, (pad, pad), mode="edge")
    y = np.convolve(x_pad, kernel, mode="valid")
    return np.asarray(y, dtype=np.float64)


def _moving_average_cols(arr: np.ndarray, win: int) -> np.ndarray:
    arr = np.asarray(arr, dtype=np.float64)
    if arr.ndim != 2:
        raise ValueError("moving_average_cols 输入需要二维数组")
    if arr.size == 0:
        return arr.copy()

    out = np.empty_like(arr, dtype=np.float64)
    for j in range(arr.shape[1]):
        out[:, j] = _moving_average_1d(arr[:, j], win)
    return out


def _resample_joint_series_uniform(
    t_raw: np.ndarray,
    q_cmd_raw: np.ndarray,
    q_fb_raw: np.ndarray,
    ctrl_hz: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    t_raw = np.asarray(t_raw, dtype=np.float64).reshape(-1)
    q_cmd_raw = np.asarray(q_cmd_raw, dtype=np.float64)
    q_fb_raw = np.asarray(q_fb_raw, dtype=np.float64)

    if q_cmd_raw.ndim != 2 or q_fb_raw.ndim != 2:
        raise ValueError("q_cmd_raw/q_fb_raw 需要二维数组 [N, joint_dim]")
    if q_cmd_raw.shape != q_fb_raw.shape:
        raise ValueError("q_cmd_raw 与 q_fb_raw 形状必须一致")

    joint_dim = int(q_cmd_raw.shape[1])
    n = min(len(t_raw), q_cmd_raw.shape[0], q_fb_raw.shape[0])
    if n <= 0:
        return (
            np.zeros((0,), dtype=np.float64),
            np.zeros((0, joint_dim), dtype=np.float64),
            np.zeros((0, joint_dim), dtype=np.float64),
        )

    t_local = t_raw[:n]
    q_cmd_local = q_cmd_raw[:n, :]
    q_fb_local = q_fb_raw[:n, :]

    keep = np.isfinite(t_local)
    if np.any(~np.isfinite(q_cmd_local)):
        keep &= np.isfinite(q_cmd_local).all(axis=1)
    if np.any(~np.isfinite(q_fb_local)):
        keep &= np.isfinite(q_fb_local).all(axis=1)

    t_local = t_local[keep]
    q_cmd_local = q_cmd_local[keep, :]
    q_fb_local = q_fb_local[keep, :]

    if t_local.size == 0:
        return (
            np.zeros((0,), dtype=np.float64),
            np.zeros((0, joint_dim), dtype=np.float64),
            np.zeros((0, joint_dim), dtype=np.float64),
        )

    if t_local.size == 1:
        return t_local.copy(), q_cmd_local.copy(), q_fb_local.copy()

    dedup = np.concatenate(([True], np.diff(t_local) > 1e-9))
    t_local = t_local[dedup]
    q_cmd_local = q_cmd_local[dedup, :]
    q_fb_local = q_fb_local[dedup, :]

    if t_local.size <= 1:
        return t_local.copy(), q_cmd_local.copy(), q_fb_local.copy()

    dt_nom = 1.0 / float(max(1, int(ctrl_hz)))
    t_uniform = np.arange(
        float(t_local[0]),
        float(t_local[-1]) + 0.5 * dt_nom,
        dt_nom,
        dtype=np.float64,
    )
    if t_uniform.size == 0:
        t_uniform = np.asarray([float(t_local[0])], dtype=np.float64)

    q_cmd_uniform = np.column_stack(
        [np.interp(t_uniform, t_local, q_cmd_local[:, j]) for j in range(joint_dim)]
    )
    q_fb_uniform = np.column_stack(
        [np.interp(t_uniform, t_local, q_fb_local[:, j]) for j in range(joint_dim)]
    )
    return t_uniform, q_cmd_uniform, q_fb_uniform


def _compute_joint_debug_derivatives(
    t_s: np.ndarray,
    q_cmd_deg: np.ndarray,
    q_fb_deg: np.ndarray,
    ctrl_hz: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t_uniform, q_cmd_uniform, q_fb_uniform = _resample_joint_series_uniform(
        t_raw=t_s,
        q_cmd_raw=q_cmd_deg,
        q_fb_raw=q_fb_deg,
        ctrl_hz=ctrl_hz,
    )

    n = t_uniform.size
    joint_dim = q_cmd_uniform.shape[1] if q_cmd_uniform.ndim == 2 else 0
    if n <= 1:
        zeros = np.zeros((n, joint_dim), dtype=np.float64)
        return t_uniform, q_cmd_uniform, q_fb_uniform, zeros, zeros, zeros, zeros

    dt_nom = 1.0 / float(max(1, int(ctrl_hz)))

    # 旧脚本逻辑：先对反馈位姿平滑，再求导，最后再平滑导数
    q_fb_smooth = _moving_average_cols(q_fb_uniform, JOINT_DEBUG_WIN_POS)

    dq_target = np.gradient(q_cmd_uniform, dt_nom, axis=0)
    dq_actual = np.gradient(q_fb_smooth, dt_nom, axis=0)

    dq_target_plot = _moving_average_cols(dq_target, JOINT_DEBUG_WIN_VEL)
    dq_actual_plot = _moving_average_cols(dq_actual, JOINT_DEBUG_WIN_VEL)

    ddq_target = np.gradient(dq_target_plot, dt_nom, axis=0)
    ddq_actual = np.gradient(dq_actual_plot, dt_nom, axis=0)

    ddq_target_plot = _moving_average_cols(ddq_target, JOINT_DEBUG_WIN_ACC)
    ddq_actual_plot = _moving_average_cols(ddq_actual, JOINT_DEBUG_WIN_ACC)

    return (
        t_uniform,
        q_cmd_uniform,
        q_fb_smooth,
        dq_target_plot,
        dq_actual_plot,
        ddq_target_plot,
        ddq_actual_plot,
    )


def _estimate_lag_s_1d(
    t_s: np.ndarray,
    target_1d: np.ndarray,
    actual_1d: np.ndarray,
    max_lag_s: float = 0.5,
) -> float:
    # 直接沿用旧脚本互相关估计逻辑
    t_s = np.asarray(t_s, dtype=np.float64).reshape(-1)
    target_1d = np.asarray(target_1d, dtype=np.float64).reshape(-1)
    actual_1d = np.asarray(actual_1d, dtype=np.float64).reshape(-1)

    n = min(len(t_s), len(target_1d), len(actual_1d))
    if n < 5:
        return 0.0

    t_local = t_s[:n]
    target_local = target_1d[:n] - float(np.mean(target_1d[:n]))
    actual_local = actual_1d[:n] - float(np.mean(actual_1d[:n]))

    if float(np.std(target_local)) < 1e-12 or float(np.std(actual_local)) < 1e-12:
        return 0.0

    dt = np.diff(t_local)
    dt = dt[dt > 0]
    if len(dt) == 0:
        return 0.0
    dt_median = float(np.median(dt))

    corr = np.correlate(actual_local, target_local, mode="full")
    lags = np.arange(-len(target_local) + 1, len(actual_local))

    max_lag_samples = max(1, int(round(float(max_lag_s) / dt_median)))
    mask = np.abs(lags) <= max_lag_samples
    if not np.any(mask):
        return 0.0

    corr_valid = corr[mask]
    lags_valid = lags[mask]
    best_lag_samples = int(lags_valid[int(np.argmax(corr_valid))])
    return float(best_lag_samples * dt_median)


def _estimate_joint_lag_s(
    t_s: np.ndarray,
    q_target_deg: np.ndarray,
    q_actual_deg: np.ndarray,
) -> float:
    if len(t_s) < 5:
        return 0.0

    t_s = np.asarray(t_s, dtype=np.float64).reshape(-1)
    q_target_deg = np.asarray(q_target_deg, dtype=np.float64)
    q_actual_deg = np.asarray(q_actual_deg, dtype=np.float64)

    if q_target_deg.ndim != 2 or q_actual_deg.ndim != 2:
        return 0.0
    if q_target_deg.shape[0] != len(t_s) or q_actual_deg.shape[0] != len(t_s):
        return 0.0

    axis_ptp = np.ptp(q_target_deg, axis=0)
    axis = int(np.argmax(axis_ptp))
    if float(axis_ptp[axis]) < 1e-12:
        return 0.0

    s_target = q_target_deg[:, axis]
    s_actual = q_actual_deg[:, axis]
    return _estimate_lag_s_1d(t_s, s_target, s_actual, max_lag_s=0.5)


def _joint_error_with_lag_comp(
    t_s: np.ndarray,
    q_target_deg: np.ndarray,
    q_actual_deg: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, float]:
    e_sync_deg = np.max(np.abs(q_actual_deg - q_target_deg), axis=1)

    lag_s = _estimate_joint_lag_s(
        t_s=t_s,
        q_target_deg=q_target_deg,
        q_actual_deg=q_actual_deg,
    )

    shifted_target_deg = np.column_stack(
        [
            np.interp(t_s - lag_s, t_s, q_target_deg[:, i], left=np.nan, right=np.nan)
            for i in range(q_target_deg.shape[1])
        ]
    )
    valid_mask = ~np.isnan(shifted_target_deg).any(axis=1)

    e_lag_comp_deg = np.full_like(e_sync_deg, np.nan, dtype=np.float64)
    if np.any(valid_mask):
        e_lag_comp_deg[valid_mask] = np.max(
            np.abs(q_actual_deg[valid_mask] - shifted_target_deg[valid_mask]),
            axis=1,
        )

    return e_sync_deg, e_lag_comp_deg, float(lag_s)


def _safe_time_for_plot(t_s: np.ndarray, n_expect: int) -> np.ndarray:
    t = np.asarray(t_s, dtype=np.float64).reshape(-1)
    if t.size != n_expect:
        t = np.arange(n_expect, dtype=np.float64)
    if t.size == 0:
        return np.asarray([0.0], dtype=np.float64)
    if not np.all(np.isfinite(t)):
        return np.arange(t.size, dtype=np.float64)
    return t


def _finite_min_max(data: np.ndarray, default_min: float = -1.0, default_max: float = 1.0) -> Tuple[float, float]:
    finite = np.asarray(data, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return float(default_min), float(default_max)
    return float(np.min(finite)), float(np.max(finite))


def _plot_joint_pair_series(
    t_s: np.ndarray,
    target_arr: np.ndarray,
    actual_arr: np.ndarray,
    save_path: str,
    mode_title: str,
    pair_title: str,
    target_label: str,
    actual_label: str,
    y_unit: str,
    min_span: float,
    abs_margin: float,
    span_mode: str = "common",
) -> str:
    target_arr = np.asarray(target_arr, dtype=np.float64)
    actual_arr = np.asarray(actual_arr, dtype=np.float64)

    if target_arr.ndim != 2 or actual_arr.ndim != 2:
        raise ValueError("joint pair series 输入需要二维数组 [N, joint_dim]")
    if target_arr.shape != actual_arr.shape:
        raise ValueError("joint pair series 的 target/actual 形状必须一致")

    n, joint_dim = target_arr.shape
    if joint_dim <= 0:
        raise ValueError("joint pair series 关节维度必须大于 0")

    if n == 0:
        # 没有数据时保底画一张空图，避免导出流程中断
        n = 1
        target_arr = np.full((1, joint_dim), np.nan, dtype=np.float64)
        actual_arr = np.full((1, joint_dim), np.nan, dtype=np.float64)

    t_plot = _safe_time_for_plot(t_s=t_s, n_expect=n)

    fig_h = max(7.0, 2.3 * float(joint_dim))
    fig, axes = plt.subplots(joint_dim, 1, figsize=(13, fig_h), sharex=True)
    if joint_dim == 1:
        axes = [axes]

    joint_min = []
    joint_max = []
    for j in range(joint_dim):
        data_j = np.concatenate([target_arr[:, j], actual_arr[:, j]])
        vmin, vmax = _finite_min_max(data_j)
        joint_min.append(vmin)
        joint_max.append(vmax)

    joint_span = np.asarray(joint_max, dtype=np.float64) - np.asarray(joint_min, dtype=np.float64)
    if span_mode not in ("common", "independent"):
        raise ValueError("span_mode 仅支持 common/independent")

    span_common_plot = None
    if span_mode == "common":
        span_common = float(np.max(joint_span)) if joint_span.size > 0 else 0.0
        span_common_plot = max(span_common * 1.05, span_common + float(abs_margin), float(min_span))

    for j in range(joint_dim):
        axes[j].plot(t_plot, target_arr[:, j], "b-", linewidth=1.1, label=target_label)
        axes[j].plot(t_plot, actual_arr[:, j], "g-", linewidth=1.0, label=actual_label)
        axes[j].set_ylabel(f"J{j + 1} ({y_unit})")
        axes[j].grid(True, linestyle="--", alpha=0.4)

        center_j = 0.5 * (joint_min[j] + joint_max[j])
        if span_mode == "common":
            half_span_j = 0.5 * float(span_common_plot)
        else:
            span_j = float(max(joint_span[j], 0.0))
            span_plot_j = max(span_j * 1.05, span_j + float(abs_margin), float(min_span))
            half_span_j = 0.5 * span_plot_j
        axes[j].set_ylim(center_j - half_span_j, center_j + half_span_j)

        if j == 0:
            axes[j].legend(loc="best")

    axes[-1].set_xlabel("t (s)")
    fig.suptitle(f"{mode_title}: {pair_title}")
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(save_path, dpi=140)
    plt.close(fig)
    return save_path


def _plot_joint_error(
    t_s: np.ndarray,
    e_sync_deg: np.ndarray,
    threshold_deg: float,
    save_path: str,
    mode_title: str,
) -> str:
    e_sync = np.asarray(e_sync_deg, dtype=np.float64).reshape(-1)
    if e_sync.size == 0:
        e_sync = np.asarray([np.nan], dtype=np.float64)
    t_plot = _safe_time_for_plot(t_s=t_s, n_expect=e_sync.size)

    fig, ax = plt.subplots(1, 1, figsize=(12, 4))
    ax.plot(t_plot, e_sync, "k-", linewidth=1.2, label="sync error")
    ax.axhline(
        threshold_deg,
        color="r",
        linestyle="--",
        linewidth=1.2,
        label=f"{threshold_deg:.1f} deg threshold",
    )
    ax.set_xlabel("t (s)")
    ax.set_ylabel("error (deg)")
    ax.set_title(f"{mode_title}: Joint Tracking Error")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(save_path, dpi=140)
    plt.close(fig)
    return save_path


def _plot_joint_error_lag_comp(
    t_s: np.ndarray,
    e_sync_deg: np.ndarray,
    e_lag_comp_deg: np.ndarray,
    lag_s: float,
    threshold_deg: float,
    save_path: str,
    mode_title: str,
) -> str:
    e_sync = np.asarray(e_sync_deg, dtype=np.float64).reshape(-1)
    e_lag = np.asarray(e_lag_comp_deg, dtype=np.float64).reshape(-1)
    n = max(e_sync.size, e_lag.size)
    if n == 0:
        n = 1
    if e_sync.size != n:
        tmp = np.full((n,), np.nan, dtype=np.float64)
        copy_n = min(n, e_sync.size)
        tmp[:copy_n] = e_sync[:copy_n]
        e_sync = tmp
    if e_lag.size != n:
        tmp = np.full((n,), np.nan, dtype=np.float64)
        copy_n = min(n, e_lag.size)
        tmp[:copy_n] = e_lag[:copy_n]
        e_lag = tmp

    t_plot = _safe_time_for_plot(t_s=t_s, n_expect=n)

    fig, ax = plt.subplots(1, 1, figsize=(12, 4))
    ax.plot(t_plot, e_sync, "k-", linewidth=1.2, label="sync error")
    ax.plot(t_plot, e_lag, "m-", linewidth=1.2, label="lag-comp error")
    ax.axhline(
        threshold_deg,
        color="r",
        linestyle="--",
        linewidth=1.2,
        label=f"{threshold_deg:.1f} deg threshold",
    )
    ax.set_xlabel("t (s)")
    ax.set_ylabel("error (deg)")
    ax.set_title(f"{mode_title}: Error + Lag Compensation (lag={lag_s:.4f}s)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(save_path, dpi=140)
    plt.close(fig)
    return save_path


def _metric_stats(arr: np.ndarray) -> Dict[str, Optional[float]]:
    v = np.asarray(arr, dtype=np.float64).reshape(-1)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return {"max": None, "mean": None, "p95": None}
    return {
        "max": float(np.max(v)),
        "mean": float(np.mean(v)),
        "p95": float(np.percentile(v, 95)),
    }


def _compute_command_timing_stats(
    t_cmd_s: np.ndarray,
    ctrl_hz: int,
) -> Dict[str, Any]:
    """基于命令时间轴做节拍后处理统计。

    注意：
    当前主循环没有显式记录 next_deadline、sleep_time 和 deadline 误差，
    因此 overrun / negative sleep 只能依据命令时间轴做后验估计。
    """
    t_cmd = np.asarray(t_cmd_s, dtype=np.float64).reshape(-1)
    hz = max(1, int(ctrl_hz))
    nominal_dt_s = 1.0 / float(hz)

    sample_count = int(t_cmd.size)
    interval_count = max(0, sample_count - 1)

    if interval_count <= 0:
        empty = np.asarray([], dtype=np.float64)
        return {
            "nominal_dt_s": float(nominal_dt_s),
            "sample_count": int(sample_count),
            "interval_count": int(interval_count),
            "delta_t_s": empty,
            "delta_t_ms": empty,
            "jitter_s": empty,
            "jitter_ms": empty,
            "delta_t_mean_ms": None,
            "delta_t_std_ms": None,
            "delta_t_max_ms": None,
            "delta_t_p95_ms": None,
            "jitter_mean_ms": None,
            "jitter_std_ms": None,
            "jitter_max_ms": None,
            "jitter_p95_ms": None,
            "estimated_overrun_count": 0,
            "estimated_negative_sleep_count": 0,
            "estimated_overrun_ratio": 0.0,
            "overrun_tolerance_s": float(max(nominal_dt_s * 0.05, 0.0003)),
        }

    delta_t_s = np.diff(t_cmd)
    delta_t_ms = delta_t_s * 1000.0
    jitter_s = delta_t_s - nominal_dt_s
    jitter_ms = jitter_s * 1000.0

    finite_delta = np.isfinite(delta_t_s)
    finite_jitter = np.isfinite(jitter_s)

    delta_valid_ms = delta_t_ms[finite_delta]
    jitter_valid_ms = jitter_ms[finite_jitter]

    if delta_valid_ms.size > 0:
        delta_t_mean_ms = float(np.mean(delta_valid_ms))
        delta_t_std_ms = float(np.std(delta_valid_ms))
        delta_t_max_ms = float(np.max(delta_valid_ms))
        delta_t_p95_ms = float(np.percentile(delta_valid_ms, 95))
    else:
        delta_t_mean_ms = None
        delta_t_std_ms = None
        delta_t_max_ms = None
        delta_t_p95_ms = None

    if jitter_valid_ms.size > 0:
        jitter_mean_ms = float(np.mean(jitter_valid_ms))
        jitter_std_ms = float(np.std(jitter_valid_ms))
        jitter_max_ms = float(np.max(jitter_valid_ms))
        jitter_p95_ms = float(np.percentile(jitter_valid_ms, 95))
    else:
        jitter_mean_ms = None
        jitter_std_ms = None
        jitter_max_ms = None
        jitter_p95_ms = None

    # 由于没有循环内部 sleep/deadline 显式记录，这里是后验推断而非精确计数
    tolerance_s = float(max(nominal_dt_s * 0.05, 0.0003))
    estimated_overrun_mask = finite_delta & (delta_t_s > (nominal_dt_s + tolerance_s))
    estimated_overrun_count = int(np.sum(estimated_overrun_mask))
    estimated_negative_sleep_count = int(estimated_overrun_count)
    estimated_overrun_ratio = float(estimated_overrun_count / float(interval_count)) if interval_count > 0 else 0.0

    return {
        "nominal_dt_s": float(nominal_dt_s),
        "sample_count": int(sample_count),
        "interval_count": int(interval_count),
        "delta_t_s": delta_t_s,
        "delta_t_ms": delta_t_ms,
        "jitter_s": jitter_s,
        "jitter_ms": jitter_ms,
        "delta_t_mean_ms": delta_t_mean_ms,
        "delta_t_std_ms": delta_t_std_ms,
        "delta_t_max_ms": delta_t_max_ms,
        "delta_t_p95_ms": delta_t_p95_ms,
        "jitter_mean_ms": jitter_mean_ms,
        "jitter_std_ms": jitter_std_ms,
        "jitter_max_ms": jitter_max_ms,
        "jitter_p95_ms": jitter_p95_ms,
        "estimated_overrun_count": int(estimated_overrun_count),
        "estimated_negative_sleep_count": int(estimated_negative_sleep_count),
        "estimated_overrun_ratio": float(estimated_overrun_ratio),
        "overrun_tolerance_s": float(tolerance_s),
    }


def _process_one_arm_joint_result(
    arm_name: str,
    arm_file_prefix: str,
    arm_result: Dict[str, Any],
    t_cmd_s: np.ndarray,
    output_dir: str,
    mode_title: str,
    run_tag: str,
    ctrl_hz: int,
) -> Tuple[Dict[str, str], Dict[str, Any], Dict[str, Any]]:
    q_cmd_deg = np.asarray(arm_result.get("q_cmd_deg"), dtype=np.float64)
    q_fb_deg = np.asarray(arm_result.get("q_fb_deg"), dtype=np.float64)

    if q_cmd_deg.ndim != 2 or q_fb_deg.ndim != 2:
        raise ValueError(f"{arm_name} 的 q_cmd_deg/q_fb_deg 必须为二维数组")
    if q_cmd_deg.shape[1] != q_fb_deg.shape[1]:
        raise ValueError(f"{arm_name} 的 q_cmd_deg/q_fb_deg 关节维度不一致")

    n = min(t_cmd_s.size, q_cmd_deg.shape[0], q_fb_deg.shape[0])
    if n <= 0:
        raise ValueError(f"{arm_name} 无有效样本")

    t_axis = np.asarray(t_cmd_s[:n], dtype=np.float64)
    q_cmd_deg = np.asarray(q_cmd_deg[:n, :], dtype=np.float64)
    q_fb_deg = np.asarray(q_fb_deg[:n, :], dtype=np.float64)

    e_q_deg = np.asarray(arm_result.get("e_q_deg"), dtype=np.float64)
    if e_q_deg.shape != q_cmd_deg.shape:
        e_q_deg = q_fb_deg - q_cmd_deg

    fb_joint_s_to_q = np.asarray(arm_result.get("fb_joint_sToq"), dtype=np.float64)
    if fb_joint_s_to_q.shape != q_cmd_deg.shape:
        fb_joint_s_to_q = np.full_like(q_cmd_deg, np.nan, dtype=np.float64)

    est_joint_firc_dot = np.asarray(arm_result.get("est_joint_firc_dot"), dtype=np.float64)
    if est_joint_firc_dot.shape != q_cmd_deg.shape:
        est_joint_firc_dot = np.full_like(q_cmd_deg, np.nan, dtype=np.float64)

    valid_mask = np.isfinite(t_axis) & np.isfinite(q_cmd_deg).all(axis=1) & np.isfinite(q_fb_deg).all(axis=1)
    t_valid = t_axis[valid_mask]
    q_cmd_valid = q_cmd_deg[valid_mask, :]
    q_fb_valid = q_fb_deg[valid_mask, :]

    if t_valid.size >= 2:
        (
            t_uniform_dbg,
            q_cmd_uniform_dbg,
            q_fb_uniform_dbg,
            dq_cmd_deg_s,
            dq_fb_deg_s,
            ddq_cmd_deg_s2,
            ddq_fb_deg_s2,
        ) = _compute_joint_debug_derivatives(
            t_s=t_valid,
            q_cmd_deg=q_cmd_valid,
            q_fb_deg=q_fb_valid,
            ctrl_hz=ctrl_hz,
        )
    else:
        # 有效样本太少时给占位数组，保证导出流程不中断
        joint_dim = q_cmd_deg.shape[1]
        t_uniform_dbg = np.asarray([0.0], dtype=np.float64)
        q_cmd_uniform_dbg = np.full((1, joint_dim), np.nan, dtype=np.float64)
        q_fb_uniform_dbg = np.full((1, joint_dim), np.nan, dtype=np.float64)
        dq_cmd_deg_s = np.full((1, joint_dim), np.nan, dtype=np.float64)
        dq_fb_deg_s = np.full((1, joint_dim), np.nan, dtype=np.float64)
        ddq_cmd_deg_s2 = np.full((1, joint_dim), np.nan, dtype=np.float64)
        ddq_fb_deg_s2 = np.full((1, joint_dim), np.nan, dtype=np.float64)

    e_sync_deg = np.full((n,), np.nan, dtype=np.float64)
    e_lag_comp_deg = np.full((n,), np.nan, dtype=np.float64)
    lag_estimated_s = 0.0

    if t_valid.size >= 5:
        e_sync_valid, e_lag_valid, lag_estimated_s = _joint_error_with_lag_comp(
            t_s=t_valid,
            q_target_deg=q_cmd_valid,
            q_actual_deg=q_fb_valid,
        )
        e_sync_deg[valid_mask] = e_sync_valid
        e_lag_comp_deg[valid_mask] = e_lag_valid
    elif t_valid.size > 0:
        e_sync_deg[valid_mask] = np.max(np.abs(q_fb_valid - q_cmd_valid), axis=1)

    sync_stats = _metric_stats(e_sync_deg)
    lag_stats = _metric_stats(e_lag_comp_deg)

    arm_metrics = {
        "max_error_deg": sync_stats["max"],
        "mean_error_deg": sync_stats["mean"],
        "p95_error_deg": sync_stats["p95"],
        "lag_estimated_s": float(lag_estimated_s),
        "lag_comp_max_error_deg": lag_stats["max"],
        "lag_comp_mean_error_deg": lag_stats["mean"],
    }

    joint_cmd_vs_fb_path = os.path.join(output_dir, f"{arm_file_prefix}_joint_cmd_vs_fb_{run_tag}.png")
    joint_vel_cmd_vs_fb_path = os.path.join(output_dir, f"{arm_file_prefix}_joint_vel_cmd_vs_fb_{run_tag}.png")
    joint_acc_cmd_vs_fb_path = os.path.join(output_dir, f"{arm_file_prefix}_joint_acc_cmd_vs_fb_{run_tag}.png")
    joint_error_path = os.path.join(output_dir, f"{arm_file_prefix}_joint_error_{run_tag}.png")
    joint_error_lag_comp_path = os.path.join(output_dir, f"{arm_file_prefix}_max_joint_error_lag_comp_{run_tag}.png")

    _plot_joint_pair_series(
        t_s=t_axis,
        target_arr=q_cmd_deg,
        actual_arr=q_fb_deg,
        save_path=joint_cmd_vs_fb_path,
        mode_title=f"{mode_title} ({arm_name})",
        pair_title="q_cmd vs q_fb",
        target_label="q_cmd",
        actual_label="q_fb",
        y_unit="deg",
        min_span=0.5,
        abs_margin=0.5,
        span_mode="common",
    )
    _plot_joint_pair_series(
        t_s=t_uniform_dbg,
        target_arr=dq_cmd_deg_s,
        actual_arr=dq_fb_deg_s,
        save_path=joint_vel_cmd_vs_fb_path,
        mode_title=f"{mode_title} ({arm_name})",
        pair_title="dq_cmd vs dq_fb",
        target_label="dq_cmd",
        actual_label="dq_fb",
        y_unit="deg/s",
        min_span=JOINT_MIN_SPAN_VEL_DEG_S,
        abs_margin=1.0,
        span_mode="independent",
    )
    _plot_joint_pair_series(
        t_s=t_uniform_dbg,
        target_arr=ddq_cmd_deg_s2,
        actual_arr=ddq_fb_deg_s2,
        save_path=joint_acc_cmd_vs_fb_path,
        mode_title=f"{mode_title} ({arm_name})",
        pair_title="ddq_cmd vs ddq_fb",
        target_label="ddq_cmd",
        actual_label="ddq_fb",
        y_unit="deg/s^2",
        min_span=JOINT_MIN_SPAN_ACC_DEG_S2,
        abs_margin=10.0,
        span_mode="independent",
    )
    _plot_joint_error(
        t_s=t_axis,
        e_sync_deg=e_sync_deg,
        threshold_deg=JOINT_ERROR_THRESHOLD_DEG,
        save_path=joint_error_path,
        mode_title=f"{mode_title} ({arm_name})",
    )
    _plot_joint_error_lag_comp(
        t_s=t_axis,
        e_sync_deg=e_sync_deg,
        e_lag_comp_deg=e_lag_comp_deg,
        lag_s=lag_estimated_s,
        threshold_deg=JOINT_ERROR_THRESHOLD_DEG,
        save_path=joint_error_lag_comp_path,
        mode_title=f"{mode_title} ({arm_name})",
    )

    arm_plot_paths = {
        "joint_cmd_vs_fb": joint_cmd_vs_fb_path,
        "joint_vel_cmd_vs_fb": joint_vel_cmd_vs_fb_path,
        "joint_acc_cmd_vs_fb": joint_acc_cmd_vs_fb_path,
        "joint_error": joint_error_path,
        "joint_error_lag_comp": joint_error_lag_comp_path,
    }

    arm_timeseries = {
        "t_cmd_s": t_axis,
        "q_cmd_deg": q_cmd_deg,
        "q_fb_deg": q_fb_deg,
        "e_q_deg": e_q_deg,
        "fb_joint_sToq": fb_joint_s_to_q,
        "est_joint_firc_dot": est_joint_firc_dot,
        "e_sync_deg": e_sync_deg,
        "e_lag_comp_deg": e_lag_comp_deg,
        "t_uniform_dbg": t_uniform_dbg,
        "q_cmd_uniform_dbg": q_cmd_uniform_dbg,
        "q_fb_uniform_dbg": q_fb_uniform_dbg,
        "dq_cmd_deg_s": dq_cmd_deg_s,
        "dq_fb_deg_s": dq_fb_deg_s,
        "ddq_cmd_deg_s2": ddq_cmd_deg_s2,
        "ddq_fb_deg_s2": ddq_fb_deg_s2,
    }

    return arm_plot_paths, arm_metrics, arm_timeseries


def _to_1d_with_default(values: Any, n: int, fill_value: float = np.nan, dtype=np.float64) -> np.ndarray:
    arr = np.asarray(values if values is not None else [], dtype=dtype).reshape(-1)
    out = np.full((n,), fill_value, dtype=dtype)
    copy_n = min(n, arr.size)
    if copy_n > 0:
        out[:copy_n] = arr[:copy_n]
    return out


def generate_showcase_plots(
    result,
    output_dir,
    tag=None,
    mode_title="NVIDIA Position Showcase",
    ctrl_hz=None,
    save_json=True,
) -> Tuple[Dict[str, Dict[str, str]], Dict[str, Any], str]:
    """双臂版本统一完成误差分析、lag 估计、绘图导出和 JSON 保存。"""
    os.makedirs(output_dir, exist_ok=True)
    run_tag = str(tag) if tag else time.strftime("%Y%m%d_%H%M%S")

    t_cmd_s = np.asarray(result.get("t_cmd_s"), dtype=np.float64).reshape(-1)
    if t_cmd_s.size == 0:
        raise ValueError("result['t_cmd_s'] 为空，无法绘图")

    sample_count = int(t_cmd_s.size)
    t_fb_s = _to_1d_with_default(result.get("t_fb_s"), n=sample_count, fill_value=np.nan, dtype=np.float64)
    fb_age_s = _to_1d_with_default(result.get("fb_age_s"), n=sample_count, fill_value=np.nan, dtype=np.float64)
    feedback_seq = _to_1d_with_default(result.get("feedback_seq"), n=sample_count, fill_value=-1, dtype=np.int64)
    feedback_stale = _to_1d_with_default(result.get("feedback_stale"), n=sample_count, fill_value=True, dtype=bool)

    effective_ctrl_hz = _infer_ctrl_hz(t_s=t_cmd_s, ctrl_hz=ctrl_hz)
    timing_metrics = _compute_command_timing_stats(
        t_cmd_s=t_cmd_s,
        ctrl_hz=effective_ctrl_hz,
    )

    left_plots, left_metrics, left_timeseries = _process_one_arm_joint_result(
        arm_name="left",
        arm_file_prefix="A_arm",
        arm_result=result.get("left", {}),
        t_cmd_s=t_cmd_s,
        output_dir=output_dir,
        mode_title=mode_title,
        run_tag=run_tag,
        ctrl_hz=effective_ctrl_hz,
    )
    right_plots, right_metrics, right_timeseries = _process_one_arm_joint_result(
        arm_name="right",
        arm_file_prefix="B_arm",
        arm_result=result.get("right", {}),
        t_cmd_s=t_cmd_s,
        output_dir=output_dir,
        mode_title=mode_title,
        run_tag=run_tag,
        ctrl_hz=effective_ctrl_hz,
    )

    stale_ratio = float(np.mean(feedback_stale.astype(np.float64))) if sample_count > 0 else None

    plot_paths = {
        "left": left_plots,
        "right": right_plots,
    }

    metrics = {
        "left": left_metrics,
        "right": right_metrics,
        "sample_count": sample_count,
        "feedback_exception_count": int(result.get("feedback_exception_count", 0)),
        "stale_ratio": stale_ratio,
        "collector_join_timeout": bool(result.get("collector_join_timeout", False)),
        "timing": timing_metrics,
    }

    json_payload = {
        "mode": "showcase_dual_arm",
        "mode_title": mode_title,
        "tag": run_tag,
        "ctrl_hz": int(effective_ctrl_hz),
        "sample_count": sample_count,
        "feedback_exception_count": int(result.get("feedback_exception_count", 0)),
        "collector_join_timeout": bool(result.get("collector_join_timeout", False)),
        "stale_ratio": stale_ratio,
        "metrics": metrics,
        "timing": timing_metrics,
        "plot_paths": plot_paths,
        "timeseries": {
            "t_cmd_s": t_cmd_s,
            "t_fb_s": t_fb_s,
            "fb_age_s": fb_age_s,
            "feedback_seq": feedback_seq,
            "feedback_stale": feedback_stale,
            "left": left_timeseries,
            "right": right_timeseries,
        },
    }

    json_path = ""
    if bool(save_json):
        json_path = save_showcase_json(
            data=json_payload,
            output_dir=output_dir,
            file_name=f"showcase_result_dual_arm_{run_tag}.json",
        )

    return plot_paths, metrics, json_path
