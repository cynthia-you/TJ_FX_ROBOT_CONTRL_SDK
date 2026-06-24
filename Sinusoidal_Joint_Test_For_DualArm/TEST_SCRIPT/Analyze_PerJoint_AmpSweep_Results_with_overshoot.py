#!/usr/bin/env python
# -*- coding: utf-8 -*-
r"""
Analyze and plot Marvin M6 per-joint sine sweep results.

This script is intended to be placed under TEST_SCRIPT/ and run from the
project folder layout:

    KernalM_Test/
        TEST_SCRIPT/
            Analyze_PerJoint_AmpSweep_Results.py
        results_per_joint_amp_sweep_200hz/
            PerJoint_AmpSweep_200Hz_YYYYMMDD_HHMMSS/
                summary.csv
                run_config.json
                *_samples.npz
                *_samples.csv
                *_meta.json

Outputs:
    <run_dir>/analysis_report/
        metrics_summary_all.csv
        realtime_summary.csv
        metrics_tables.md
        delay_summary.csv
        figures/
            J1_tracking_summary.png
            ...
            J7_tracking_summary.png

Default choices:
    - velocity plot/table uses numerical differentiation of feedback position
      because this retest request asks for "反馈速度差分图".
    - torque plot/table uses torque_sToq by default, because it is usually in
      Nm-scale in the saved samples; cToq/them are still available via CLI.
    - delay metrics are computed offline from q_cmd and q_fb using normalized
      cross-correlation and sine least-squares fitting.
    - by default, the first full sine cycle is excluded from metrics and plots
      to avoid startup/settling oscillation bias.

Typical usage:
    python .\TEST_SCRIPT\Analyze_PerJoint_AmpSweep_Results.py

Run a specific result folder:
    python .\TEST_SCRIPT\Analyze_PerJoint_AmpSweep_Results.py ^
        --run_dir .\results_per_joint_amp_sweep_200hz\PerJoint_AmpSweep_200Hz_20260623_113241

Only plot J3:
    python .\TEST_SCRIPT\Analyze_PerJoint_AmpSweep_Results.py --joint_filter J3
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]


# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------
def script_dir() -> Path:
    return Path(__file__).resolve().parent


def normalize_cli_path(path_text: object) -> Path:
    """Accept native paths and common Windows-style relative paths on Linux.

    Example: on Linux, ``.\\results\\run`` is converted to ``./results/run``.
    Windows drive-letter paths are not translated; mount/copy the folder and
    pass its POSIX path instead.
    """
    raw = os.path.expanduser(os.path.expandvars(str(path_text).strip()))
    p = Path(raw)
    if p.exists():
        return p
    if os.name != "nt" and "\\" in raw:
        if not re.match(r"^[A-Za-z]:[\\/]", raw):
            return Path(raw.replace("\\", "/"))
    return p


def find_project_root() -> Path:
    """Find project root from common execution locations."""
    candidates = [
        Path.cwd().resolve(),
        Path.cwd().resolve().parent,
        script_dir(),
        script_dir().parent,
    ]
    seen = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        if (candidate / "results_per_joint_amp_sweep_200hz").exists():
            return candidate
        if (candidate / "results_per_joint_custom_curve_200hz").exists():
            return candidate
    # Fallback: if running inside TEST_SCRIPT, parent is usually project root.
    if script_dir().name.upper() == "TEST_SCRIPT":
        return script_dir().parent
    return Path.cwd().resolve()


def find_latest_run_dir(result_root: Path) -> Path:
    if not result_root.exists():
        raise FileNotFoundError(f"Result root does not exist: {result_root}")
    candidates = [
        p for p in result_root.iterdir()
        if p.is_dir() and p.name.startswith("PerJoint_AmpSweep_200Hz_")
    ]
    if not candidates:
        candidates = [p for p in result_root.iterdir() if p.is_dir()]
    if not candidates:
        raise FileNotFoundError(f"No run directories found under: {result_root}")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def resolve_input_run_dir(args: argparse.Namespace) -> Path:
    if args.run_dir:
        return normalize_cli_path(args.run_dir).resolve()

    project_root = find_project_root()
    result_root = normalize_cli_path(args.result_root)
    if not result_root.is_absolute():
        result_root = project_root / result_root
    return find_latest_run_dir(result_root.resolve())


def as_existing_path(path_text: object, run_dir: Path) -> Optional[Path]:
    """Resolve a path from summary/meta robustly.

    The summary may contain absolute Windows paths from the test PC.  If those
    paths are not valid in the current environment, fall back to the basename
    under run_dir.
    """
    if path_text is None or (isinstance(path_text, float) and np.isnan(path_text)):
        return None
    raw = str(path_text).strip()
    if not raw:
        return None

    p = normalize_cli_path(raw)
    if p.exists():
        return p

    # Windows absolute path on a different environment: use basename in run_dir.
    basename = Path(raw.replace("\\", "/")).name
    fallback = run_dir / basename
    if fallback.exists():
        return fallback

    return None


def infer_trial_tag(row: pd.Series) -> str:
    trial_index = int(row.get("trial_index", -1))
    joint_name = str(row.get("joint_name", "J?"))
    speed = float(row.get("target_speed_deg_s", np.nan))
    amp = float(row.get("amplitude_deg", np.nan))
    amp_text = f"{amp:.1f}".replace(".", "p")
    return f"{trial_index:03d}_{joint_name}_{int(speed)}dps_A{amp_text}deg"


def find_sample_file(row: pd.Series, run_dir: Path, suffix: str) -> Optional[Path]:
    """suffix is '.npz' or '.csv'."""
    key = "sample_npz" if suffix == ".npz" else "sample_csv"
    p = as_existing_path(row.get(key), run_dir)
    if p is not None:
        return p

    tag = infer_trial_tag(row)
    direct = run_dir / f"{tag}_samples{suffix}"
    if direct.exists():
        return direct

    # Last chance: glob by trial index + joint + speed.
    trial_index = int(row.get("trial_index", -1))
    joint = str(row.get("joint_name", ""))
    speed = int(float(row.get("target_speed_deg_s", -1)))
    pattern = f"{trial_index:03d}_{joint}_{speed}dps_*_samples{suffix}"
    matches = sorted(run_dir.glob(pattern))
    return matches[0] if matches else None


# ---------------------------------------------------------------------------
# Loading helpers
# ---------------------------------------------------------------------------
def load_summary(run_dir: Path) -> pd.DataFrame:
    csv_path = run_dir / "summary.csv"
    json_path = run_dir / "summary.json"

    if csv_path.exists():
        return pd.read_csv(csv_path)
    if json_path.exists():
        return pd.read_json(json_path)
    raise FileNotFoundError(f"Neither summary.csv nor summary.json found in {run_dir}")


def load_run_config(run_dir: Path) -> Dict[str, object]:
    path = run_dir / "run_config.json"
    if path.exists():
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}


def load_precheck(run_dir: Path) -> Optional[pd.DataFrame]:
    path = run_dir / "precheck_all_groups.csv"
    if path.exists():
        return pd.read_csv(path)
    return None


def load_trial_meta(row: pd.Series, run_dir: Path) -> Dict[str, object]:
    p = as_existing_path(row.get("trial_json"), run_dir)
    if p is None:
        tag = infer_trial_tag(row)
        candidate = run_dir / f"{tag}_meta.json"
        p = candidate if candidate.exists() else None
    if p is None:
        return {}
    try:
        with open(p, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def load_trial_arrays(row: pd.Series, run_dir: Path) -> Tuple[Optional[Dict[str, np.ndarray]], Optional[Path], str]:
    """Return (data, path, source). Prefer NPZ; fall back to CSV."""
    npz_path = find_sample_file(row, run_dir, ".npz")
    if npz_path is not None:
        with np.load(npz_path, allow_pickle=True) as npz:
            data = {key: npz[key] for key in npz.files}
        return data, npz_path, "npz"

    csv_path = find_sample_file(row, run_dir, ".csv")
    if csv_path is not None:
        df = pd.read_csv(csv_path)
        data = dataframe_to_arrays(df)
        return data, csv_path, "csv"

    return None, None, "missing"


def dataframe_to_arrays(df: pd.DataFrame) -> Dict[str, np.ndarray]:
    data: Dict[str, np.ndarray] = {}
    scalar_cols = [
        "sample_index",
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
    for col in scalar_cols:
        if col in df.columns:
            data[col] = pd.to_numeric(df[col], errors="coerce").to_numpy()

    vector_map = {
        "q_cmd": "q_cmd_{joint}_deg",
        "dq_cmd": "dq_cmd_{joint}_deg_s",
        "ddq_cmd": "ddq_cmd_{joint}_deg_s2",
        "q_fb": "q_fb_{joint}_deg",
        "dq_fb": "dq_fb_{joint}_deg_s",
        "q_fb_cmd": "q_fb_cmd_{joint}_deg",
        "torque_cToq": "torque_cToq_{joint}_Nm",
        "torque_sToq": "torque_sToq_{joint}_Nm",
        "torque_them": "torque_them_{joint}_Nm",
    }
    for key, fmt in vector_map.items():
        arr = np.full((len(df), 7), np.nan, dtype=np.float64)
        for j, joint in enumerate(JOINT_NAMES):
            col = fmt.format(joint=joint)
            if col in df.columns:
                arr[:, j] = pd.to_numeric(df[col], errors="coerce").to_numpy()
        data[key] = arr
    return data


# ---------------------------------------------------------------------------
# Numeric helpers
# ---------------------------------------------------------------------------
def joint_index_from_name(joint_name: object) -> int:
    text = str(joint_name).strip().upper()
    if text.startswith("J"):
        text = text[1:]
    idx = int(text) - 1
    if idx < 0 or idx >= 7:
        raise ValueError(f"Invalid joint name: {joint_name!r}")
    return idx


def finite_nanmax_abs(arr: np.ndarray) -> float:
    arr = np.asarray(arr, dtype=np.float64)
    if not np.isfinite(arr).any():
        return float("nan")
    return float(np.nanmax(np.abs(arr)))


def finite_rmse(err: np.ndarray) -> float:
    err = np.asarray(err, dtype=np.float64)
    err = err[np.isfinite(err)]
    if err.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(err * err)))


def pick_time_axis(data: Dict[str, np.ndarray]) -> np.ndarray:
    """Prefer actual send time; fall back to planned time."""
    t_send = np.asarray(data.get("t_send_s", []), dtype=np.float64)
    if t_send.size >= 2 and np.isfinite(t_send).sum() >= 2:
        # t_send may include NaN only if data is malformed; preserve.
        return t_send
    t_plan = np.asarray(data.get("t_plan_s", []), dtype=np.float64)
    return t_plan


def analysis_time_axis_for_trimming(data: Dict[str, np.ndarray]) -> np.ndarray:
    """Use planned trajectory time for cycle trimming when available.

    The command formula is defined against t_plan_s, so trimming by cycle should
    follow t_plan_s rather than wall-clock send jitter.  If t_plan_s is absent,
    fall back to the chosen plotting/analysis time axis.
    """
    t_plan = np.asarray(data.get("t_plan_s", []), dtype=np.float64)
    if t_plan.size >= 2 and np.isfinite(t_plan).sum() >= 2:
        return t_plan
    return pick_time_axis(data)


def trim_data_after_first_cycles(
    data: Dict[str, np.ndarray],
    period_s: float,
    drop_first_cycles: float,
) -> Tuple[Dict[str, np.ndarray], Dict[str, object]]:
    """Return a view/copy of sample arrays after dropping initial cycles.

    Arrays whose first dimension matches the sample count are trimmed.  Scalars
    or metadata arrays are left unchanged.  The default drop_first_cycles=1.0
    means all metrics, delay estimates and figures start at the second cycle.
    """
    drop_first_cycles = float(drop_first_cycles)
    t_ref = analysis_time_axis_for_trimming(data)
    n = int(len(t_ref))

    meta = {
        "analysis_drop_first_cycles": float(drop_first_cycles),
        "analysis_start_time_s": 0.0,
        "analysis_dropped_samples": 0,
        "analysis_original_sample_count": n,
        "analysis_sample_count": n,
    }

    if n == 0 or drop_first_cycles <= 0.0 or not np.isfinite(period_s) or float(period_s) <= 0.0:
        return data, meta

    t0 = float(np.nanmin(t_ref)) if np.isfinite(t_ref).any() else 0.0
    start_s = t0 + float(drop_first_cycles) * float(period_s)
    mask = np.asarray(t_ref, dtype=np.float64) >= (start_s - 1e-12)

    # If a requested trim would remove almost all data, keep the original data
    # but report the failed trim request. This avoids producing misleading NaN
    # reports for very short debug runs.
    if int(np.sum(mask)) < 8:
        meta["analysis_note"] = "Requested first-cycle trim left fewer than 8 samples; original data kept."
        return data, meta

    trimmed: Dict[str, np.ndarray] = {}
    for key, value in data.items():
        arr = np.asarray(value)
        if arr.ndim >= 1 and arr.shape[0] == n:
            trimmed[key] = arr[mask]
        else:
            trimmed[key] = arr

    meta.update({
        "analysis_start_time_s": float(start_s - t0),
        "analysis_dropped_samples": int(n - int(np.sum(mask))),
        "analysis_original_sample_count": n,
        "analysis_sample_count": int(np.sum(mask)),
    })
    return trimmed, meta


def differentiate_position_deg_s(t: np.ndarray, q: np.ndarray) -> np.ndarray:
    t = np.asarray(t, dtype=np.float64)
    q = np.asarray(q, dtype=np.float64)
    v = np.full_like(q, np.nan, dtype=np.float64)
    mask = np.isfinite(t) & np.isfinite(q)
    if mask.sum() < 2:
        return v
    idx = np.flatnonzero(mask)
    tt = t[idx]
    qq = q[idx]
    # Remove non-increasing duplicate time points if any.
    keep = np.ones_like(tt, dtype=bool)
    keep[1:] = np.diff(tt) > 1e-9
    idx = idx[keep]
    tt = tt[keep]
    qq = qq[keep]
    if len(tt) < 2:
        return v
    vv = np.gradient(qq, tt)
    v[idx] = vv
    return v


def moving_average_nan(arr: np.ndarray, window: int) -> np.ndarray:
    window = int(window)
    if window <= 1:
        return np.asarray(arr, dtype=np.float64)
    if window % 2 == 0:
        window += 1

    x = np.asarray(arr, dtype=np.float64)
    y = np.full_like(x, np.nan)
    finite = np.isfinite(x)
    if finite.sum() == 0:
        return y

    values = np.where(finite, x, 0.0)
    weights = finite.astype(np.float64)
    kernel = np.ones(window, dtype=np.float64)

    num = np.convolve(values, kernel, mode="same")
    den = np.convolve(weights, kernel, mode="same")
    ok = den > 0
    y[ok] = num[ok] / den[ok]
    return y



def _wrap_to_pi(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return float((angle_rad + math.pi) % (2.0 * math.pi) - math.pi)


def normalized_xcorr_delay_ms(
    t: np.ndarray,
    q_cmd: np.ndarray,
    q_fb: np.ndarray,
    max_lag_fraction_of_period: float = 0.45,
    period_s: Optional[float] = None,
) -> Dict[str, float]:
    """Estimate feedback delay using normalized cross-correlation.

    Sign convention: positive delay means feedback lags command.
    The integer-sample result is limited by the sample period.  A parabolic
    interpolation around the correlation peak is also reported as a sub-sample
    estimate, but it should not be over-interpreted below one control cycle.
    """
    t = np.asarray(t, dtype=np.float64)
    cmd = np.asarray(q_cmd, dtype=np.float64)
    fb = np.asarray(q_fb, dtype=np.float64)
    mask = np.isfinite(t) & np.isfinite(cmd) & np.isfinite(fb)
    if mask.sum() < 8:
        return {
            "delay_xcorr_ms": float("nan"),
            "delay_xcorr_refined_ms": float("nan"),
            "delay_xcorr_samples": float("nan"),
            "delay_xcorr_refined_samples": float("nan"),
            "xcorr_peak": float("nan"),
            "sample_dt_ms_for_delay": float("nan"),
        }

    tt = t[mask]
    c = cmd[mask]
    f = fb[mask]

    # Keep monotonic samples only.
    keep = np.ones_like(tt, dtype=bool)
    keep[1:] = np.diff(tt) > 1e-9
    tt = tt[keep]
    c = c[keep]
    f = f[keep]
    if len(tt) < 8:
        return {
            "delay_xcorr_ms": float("nan"),
            "delay_xcorr_refined_ms": float("nan"),
            "delay_xcorr_samples": float("nan"),
            "delay_xcorr_refined_samples": float("nan"),
            "xcorr_peak": float("nan"),
            "sample_dt_ms_for_delay": float("nan"),
        }

    dt = float(np.nanmedian(np.diff(tt)))
    if not np.isfinite(dt) or dt <= 0:
        return {
            "delay_xcorr_ms": float("nan"),
            "delay_xcorr_refined_ms": float("nan"),
            "delay_xcorr_samples": float("nan"),
            "delay_xcorr_refined_samples": float("nan"),
            "xcorr_peak": float("nan"),
            "sample_dt_ms_for_delay": float("nan"),
        }

    # If t is not perfectly uniform, interpolate to a uniform grid so each lag
    # corresponds to one stable dt.
    uniform_t = np.arange(tt[0], tt[-1] + 0.5 * dt, dt)
    if uniform_t.size < 8:
        uniform_t = tt
    c = np.interp(uniform_t, tt, c)
    f = np.interp(uniform_t, tt, f)
    n = len(uniform_t)

    c = c - np.mean(c)
    f = f - np.mean(f)
    c_std = float(np.std(c))
    f_std = float(np.std(f))
    if c_std <= 0 or f_std <= 0:
        return {
            "delay_xcorr_ms": float("nan"),
            "delay_xcorr_refined_ms": float("nan"),
            "delay_xcorr_samples": float("nan"),
            "delay_xcorr_refined_samples": float("nan"),
            "xcorr_peak": float("nan"),
            "sample_dt_ms_for_delay": float(dt * 1000.0),
        }
    c = c / c_std
    f = f / f_std

    # np.correlate(fb, cmd): positive lag => fb lags cmd by lag samples.
    corr_full = np.correlate(f, c, mode="full") / float(n)
    lags = np.arange(-n + 1, n, dtype=np.int64)

    if period_s is not None and np.isfinite(period_s) and period_s > 0:
        max_lag = max(1, int(round(max_lag_fraction_of_period * period_s / dt)))
        select = np.abs(lags) <= max_lag
        if np.any(select):
            corr = corr_full[select]
            lag_subset = lags[select]
            base_offset = int(np.flatnonzero(select)[0])
        else:
            corr = corr_full
            lag_subset = lags
            base_offset = 0
    else:
        corr = corr_full
        lag_subset = lags
        base_offset = 0

    local_idx = int(np.nanargmax(corr))
    global_idx = base_offset + local_idx
    lag_samples = int(lag_subset[local_idx])
    peak = float(corr[local_idx])

    refined_lag = float(lag_samples)
    if 0 < global_idx < len(corr_full) - 1:
        y0 = float(corr_full[global_idx - 1])
        y1 = float(corr_full[global_idx])
        y2 = float(corr_full[global_idx + 1])
        denom = y0 - 2.0 * y1 + y2
        if abs(denom) > 1e-12:
            delta = 0.5 * (y0 - y2) / denom
            if abs(delta) <= 1.0:
                refined_lag = float(lag_samples) + float(delta)

    return {
        "delay_xcorr_ms": float(lag_samples * dt * 1000.0),
        "delay_xcorr_refined_ms": float(refined_lag * dt * 1000.0),
        "delay_xcorr_samples": float(lag_samples),
        "delay_xcorr_refined_samples": float(refined_lag),
        "xcorr_peak": peak,
        "sample_dt_ms_for_delay": float(dt * 1000.0),
    }


def sine_fit_delay_ms(
    t: np.ndarray,
    q_cmd: np.ndarray,
    q_fb: np.ndarray,
    period_s: float,
) -> Dict[str, float]:
    """Estimate delay by fitting y = B sin(wt) + C cos(wt) + D.

    Sign convention: positive delay means feedback lags command.
    """
    t = np.asarray(t, dtype=np.float64)
    cmd = np.asarray(q_cmd, dtype=np.float64)
    fb = np.asarray(q_fb, dtype=np.float64)
    period_s = float(period_s)
    mask = np.isfinite(t) & np.isfinite(cmd) & np.isfinite(fb)
    if mask.sum() < 8 or not np.isfinite(period_s) or period_s <= 0:
        return {
            "delay_sinefit_ms": float("nan"),
            "phase_lag_deg": float("nan"),
            "fit_amp_cmd_deg": float("nan"),
            "fit_amp_fb_deg": float("nan"),
            "fit_amp_ratio_pct": float("nan"),
            "fit_peak_fb_velocity_deg_s": float("nan"),
        }

    tt = t[mask]
    c = cmd[mask]
    f = fb[mask]
    w = 2.0 * math.pi / period_s
    X = np.column_stack([np.sin(w * tt), np.cos(w * tt), np.ones_like(tt)])

    try:
        beta_c, *_ = np.linalg.lstsq(X, c, rcond=None)
        beta_f, *_ = np.linalg.lstsq(X, f, rcond=None)
    except Exception:
        return {
            "delay_sinefit_ms": float("nan"),
            "phase_lag_deg": float("nan"),
            "fit_amp_cmd_deg": float("nan"),
            "fit_amp_fb_deg": float("nan"),
            "fit_amp_ratio_pct": float("nan"),
            "fit_peak_fb_velocity_deg_s": float("nan"),
        }

    amp_cmd = float(math.hypot(float(beta_c[0]), float(beta_c[1])))
    amp_fb = float(math.hypot(float(beta_f[0]), float(beta_f[1])))
    phi_cmd = float(math.atan2(float(beta_c[1]), float(beta_c[0])))
    phi_fb = float(math.atan2(float(beta_f[1]), float(beta_f[0])))

    phase_diff = _wrap_to_pi(phi_fb - phi_cmd)
    # fb = sin(w(t-delay)) => phase_diff = -w*delay.
    delay_s = -phase_diff / w
    phase_lag_deg = -math.degrees(phase_diff)

    return {
        "delay_sinefit_ms": float(delay_s * 1000.0),
        "phase_lag_deg": float(phase_lag_deg),
        "fit_amp_cmd_deg": amp_cmd,
        "fit_amp_fb_deg": amp_fb,
        "fit_amp_ratio_pct": float(amp_fb / amp_cmd * 100.0) if amp_cmd > 0 else float("nan"),
        "fit_peak_fb_velocity_deg_s": float(amp_fb * w),
    }


def compute_delay_metrics(
    t: np.ndarray,
    q_cmd: np.ndarray,
    q_fb: np.ndarray,
    period_s: float,
) -> Dict[str, float]:
    metrics = normalized_xcorr_delay_ms(t, q_cmd, q_fb, period_s=period_s)
    metrics.update(sine_fit_delay_ms(t, q_cmd, q_fb, period_s))
    if np.isfinite(metrics.get("delay_xcorr_refined_ms", np.nan)) and np.isfinite(metrics.get("delay_sinefit_ms", np.nan)):
        metrics["delay_method_diff_ms"] = float(metrics["delay_xcorr_refined_ms"] - metrics["delay_sinefit_ms"])
    else:
        metrics["delay_method_diff_ms"] = float("nan")
    return metrics


def get_feedback_velocity(
    data: Dict[str, np.ndarray],
    joint_index: int,
    velocity_source: str,
    diff_smooth_window: int,
) -> Tuple[np.ndarray, str]:
    if velocity_source == "sdk" and "dq_fb" in data:
        return np.asarray(data["dq_fb"][:, joint_index], dtype=np.float64), "sdk_dq_fb"

    t = pick_time_axis(data)
    q_fb = np.asarray(data["q_fb"][:, joint_index], dtype=np.float64)
    v = differentiate_position_deg_s(t, q_fb)
    v = moving_average_nan(v, diff_smooth_window)
    return v, "diff_q_fb"


def get_torque(data: Dict[str, np.ndarray], joint_index: int, torque_source: str) -> Tuple[np.ndarray, str]:
    key = f"torque_{torque_source}"
    if key in data:
        return np.asarray(data[key][:, joint_index], dtype=np.float64), key
    # Fallback order.
    for key in ("torque_sToq", "torque_cToq", "torque_them"):
        if key in data:
            return np.asarray(data[key][:, joint_index], dtype=np.float64), key
    n = len(pick_time_axis(data))
    return np.full(n, np.nan, dtype=np.float64), "none"


def resolve_home_joint_deg(
    meta: Optional[Dict[str, object]],
    data: Dict[str, np.ndarray],
    joint_index: int,
) -> Tuple[float, str]:
    """Resolve the nominal home/center position for the moving joint.

    Priority:
    1. ``home_joints_deg`` from trial meta json, because it is the exact
       home used by the robot script.
    2. Center of the command trajectory, i.e. (max(q_cmd)+min(q_cmd))/2.
       This works for a full or near-full sine window and also keeps the
       analysis usable if meta json is missing.
    """
    if isinstance(meta, dict):
        home = meta.get("home_joints_deg")
        if isinstance(home, (list, tuple)) and len(home) > joint_index:
            try:
                v = float(home[joint_index])
                if np.isfinite(v):
                    return v, "meta_home_joints_deg"
            except Exception:
                pass

    q_cmd_all = data.get("q_cmd")
    if q_cmd_all is not None:
        q_cmd_j = np.asarray(q_cmd_all[:, joint_index], dtype=np.float64)
        finite = q_cmd_j[np.isfinite(q_cmd_j)]
        if finite.size:
            return float(0.5 * (np.nanmax(finite) + np.nanmin(finite))), "q_cmd_midrange"

    return float("nan"), "missing"


def safe_ratio(numerator: float, denominator: float) -> float:
    numerator = float(numerator)
    denominator = float(denominator)
    if np.isfinite(numerator) and np.isfinite(denominator) and abs(denominator) > 1e-12:
        return float(numerator / abs(denominator))
    return float("nan")


def compute_metrics_for_trial(
    row: pd.Series,
    data: Dict[str, np.ndarray],
    args: argparse.Namespace,
    meta: Optional[Dict[str, object]] = None,
) -> Dict[str, object]:
    joint_name = str(row["joint_name"])
    j = joint_index_from_name(joint_name)
    amp = float(row.get("amplitude_deg", np.nan))
    vel_limit = float(row.get("vel_limit_deg_s", np.nan))
    period_s = float(row.get("period_s", np.nan))

    data, trim_meta = trim_data_after_first_cycles(
        data=data,
        period_s=period_s,
        drop_first_cycles=float(args.drop_first_cycles),
    )

    t = pick_time_axis(data)
    q_cmd = np.asarray(data["q_cmd"][:, j], dtype=np.float64)
    q_fb = np.asarray(data["q_fb"][:, j], dtype=np.float64)
    err = q_fb - q_cmd

    v_fb, velocity_used = get_feedback_velocity(
        data=data,
        joint_index=j,
        velocity_source=args.velocity_source,
        diff_smooth_window=args.diff_smooth_window,
    )
    torque, torque_used = get_torque(data, j, args.torque_source)

    rmse = finite_rmse(err)
    max_error = finite_nanmax_abs(err)
    rmse_amp_pct = float(rmse / amp * 100.0) if np.isfinite(rmse) and amp > 0 else float("nan")
    peak_vel = finite_nanmax_abs(v_fb)
    peak_torque = finite_nanmax_abs(torque)

    home_joint_deg, home_source = resolve_home_joint_deg(meta, data, j)
    q_fb_delta_abs_peak = finite_nanmax_abs(q_fb - home_joint_deg) if np.isfinite(home_joint_deg) else float("nan")
    position_peak_ratio = safe_ratio(q_fb_delta_abs_peak, amp)
    position_peak_ratio_pct = float(position_peak_ratio * 100.0) if np.isfinite(position_peak_ratio) else float("nan")
    # Positive means true overshoot beyond the theoretical amplitude; negative means under-amplitude.
    position_overshoot_pct = float((position_peak_ratio - 1.0) * 100.0) if np.isfinite(position_peak_ratio) else float("nan")

    cmd_peak_vel = float("nan")
    if "dq_cmd" in data:
        cmd_peak_vel = finite_nanmax_abs(np.asarray(data["dq_cmd"][:, j], dtype=np.float64))
    if not np.isfinite(cmd_peak_vel) or abs(cmd_peak_vel) <= 1e-12:
        cmd_peak_vel = safe_float(row.get("theoretical_peak_vel_deg_s"))
    if cmd_peak_vel is None or not np.isfinite(float(cmd_peak_vel)) or abs(float(cmd_peak_vel)) <= 1e-12:
        cmd_peak_vel = safe_float(row.get("target_speed_deg_s"))
    cmd_peak_vel = float(cmd_peak_vel) if cmd_peak_vel is not None else float("nan")

    velocity_peak_ratio = safe_ratio(peak_vel, cmd_peak_vel)
    velocity_peak_ratio_pct = float(velocity_peak_ratio * 100.0) if np.isfinite(velocity_peak_ratio) else float("nan")
    # Positive means feedback velocity exceeds the command peak; negative means lower than command peak.
    velocity_overshoot_pct = float((velocity_peak_ratio - 1.0) * 100.0) if np.isfinite(velocity_peak_ratio) else float("nan")
    delay_metrics = compute_delay_metrics(
        t=t,
        q_cmd=q_cmd,
        q_fb=q_fb,
        period_s=period_s,
    )

    # Realtime stats from sample arrays are recalculated as a cross-check, while
    # summary.csv/meta.json values are also propagated by build_metrics_table().
    dt = np.asarray(data.get("command_dt_ms", []), dtype=np.float64)
    dt_finite = dt[np.isfinite(dt)]
    jitter = np.asarray(data.get("command_jitter_ms", []), dtype=np.float64)
    jitter_finite = jitter[np.isfinite(jitter)]
    overrun = np.asarray(data.get("overrun_flag", []), dtype=np.float64)

    return {
        "rmse_deg": rmse,
        "max_error_deg": max_error,
        "rmse_over_amp_pct": rmse_amp_pct,
        "peak_fb_velocity_deg_s": peak_vel,
        "peak_torque": peak_torque,
        "home_joint_deg": home_joint_deg,
        "home_source_used": home_source,
        "peak_abs_delta_fb_deg": q_fb_delta_abs_peak,
        "position_peak_ratio_pct": position_peak_ratio_pct,
        "position_overshoot_pct": position_overshoot_pct,
        "cmd_peak_velocity_deg_s": cmd_peak_vel,
        "velocity_peak_ratio_pct": velocity_peak_ratio_pct,
        "velocity_overshoot_pct": velocity_overshoot_pct,
        "velocity_source_used": velocity_used,
        "torque_source_used": torque_used,
        "fb_velocity_over_limit": bool(np.isfinite(peak_vel) and np.isfinite(vel_limit) and peak_vel > vel_limit),
        "sample_count": int(len(t)),
        **trim_meta,
        "command_dt_mean_ms_calc": float(np.nanmean(dt_finite)) if dt_finite.size else float("nan"),
        "command_dt_max_ms_calc": float(np.nanmax(dt_finite)) if dt_finite.size else float("nan"),
        "command_dt_p95_ms_calc": float(np.nanpercentile(dt_finite, 95)) if dt_finite.size else float("nan"),
        "command_jitter_max_ms_calc": float(np.nanmax(jitter_finite)) if jitter_finite.size else float("nan"),
        "overrun_count_calc": int(np.nansum(overrun)) if overrun.size else 0,
        **delay_metrics,
    }


# ---------------------------------------------------------------------------
# Tables and plots
# ---------------------------------------------------------------------------
def build_metrics_table(summary: pd.DataFrame, run_dir: Path, args: argparse.Namespace) -> Tuple[pd.DataFrame, Dict[str, Dict[str, np.ndarray]]]:
    rows: List[Dict[str, object]] = []
    loaded_data: Dict[str, Dict[str, np.ndarray]] = {}

    for _, row in summary.iterrows():
        base = {
            "trial_index": row.get("trial_index"),
            "status": row.get("status"),
            "joint_name": row.get("joint_name"),
            "target_speed_deg_s": row.get("target_speed_deg_s"),
            "amplitude_deg": row.get("amplitude_deg"),
            "period_s": row.get("period_s"),
            "theoretical_peak_vel_deg_s": row.get("theoretical_peak_vel_deg_s"),
            "theoretical_peak_acc_deg_s2": row.get("theoretical_peak_acc_deg_s2"),
            "vel_limit_deg_s": row.get("vel_limit_deg_s"),
            "acc_limit_deg_s2": row.get("acc_limit_deg_s2"),
            "precheck_result": row.get("precheck_result"),
            "skip_reason": row.get("skip_reason"),
            "feedback_watch": row.get("feedback_watch"),
            "historical_feedback_peak_vel_deg_s": row.get("historical_feedback_peak_vel_deg_s"),
            "actual_duration_s": row.get("actual_duration_s"),
            "actual_cycles": row.get("actual_cycles"),
            "command_dt_mean_ms": row.get("command_dt_mean_ms"),
            "command_dt_max_ms": row.get("command_dt_max_ms"),
            "command_dt_p95_ms": row.get("command_dt_p95_ms"),
            "command_jitter_mean_ms": row.get("command_jitter_mean_ms"),
            "command_jitter_max_ms": row.get("command_jitter_max_ms"),
            "command_jitter_p95_ms": row.get("command_jitter_p95_ms"),
            "negative_sleep_count": row.get("negative_sleep_count"),
            "overrun_count": row.get("overrun_count"),
            "feedback_sample_count": row.get("feedback_sample_count"),
            "sample_csv": row.get("sample_csv"),
            "sample_npz": row.get("sample_npz"),
            "trial_json": row.get("trial_json"),
            "data_file_resolved": None,
            "data_source": None,
            "analysis_note": "",
        }

        status = str(row.get("status", "")).lower()
        if status != "executed":
            rows.append(base)
            continue

        data, data_path, data_source = load_trial_arrays(row, run_dir)
        if data is None:
            base["status"] = "executed_missing_data"
            base["analysis_note"] = "Executed in summary, but sample NPZ/CSV was not found."
            rows.append(base)
            continue

        trial_tag = infer_trial_tag(row)
        loaded_data[trial_tag] = data

        meta = load_trial_meta(row, run_dir)
        metrics = compute_metrics_for_trial(row, data, args, meta=meta)
        base.update(metrics)
        base["data_file_resolved"] = str(data_path) if data_path else None
        base["data_source"] = data_source
        rows.append(base)

    df = pd.DataFrame(rows)
    return df, loaded_data


def format_cell(value: object, decimals: int = 2, suffix: str = "") -> str:
    try:
        v = float(value)
        if not np.isfinite(v):
            return "—"
        return f"{v:.{decimals}f}{suffix}"
    except Exception:
        if value is None or (isinstance(value, float) and np.isnan(value)):
            return "—"
        return str(value)


def markdown_table(headers: Sequence[str], rows: Sequence[Sequence[str]]) -> str:
    out = []
    out.append("| " + " | ".join(headers) + " |")
    out.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        out.append("| " + " | ".join(str(x) for x in row) + " |")
    return "\n".join(out)


def write_markdown_report(metrics: pd.DataFrame, out_path: Path, run_dir: Path, args: argparse.Namespace, config: Dict[str, object]) -> None:
    lines: List[str] = []
    lines.append("# Marvin M6 单关节正弦追踪复测数据汇总")
    lines.append("")
    lines.append(f"- 数据目录：`{run_dir}`")
    lines.append(f"- 首周期处理：剔除前 `{args.drop_first_cycles:g}` 个完整周期后再计算 RMSE、速度峰值、延迟并绘图；若需关闭，可设置 `--drop_first_cycles 0`。")
    lines.append(f"- 速度曲线来源：`{args.velocity_source}`；若为 `diff`，则由反馈位置差分得到。")
    lines.append(f"- 力矩曲线来源：`torque_{args.torque_source}`")
    lines.append("- 延迟指标：互相关延迟 `delay_xcorr_refined_ms` 与正弦拟合延迟 `delay_sinefit_ms`，正值表示反馈滞后指令。低于一个采样周期的数值仅作为拟合/插值估计。")
    lines.append("")

    equipment = config.get("equipment", {}) if isinstance(config, dict) else {}
    if equipment:
        lines.append("## 测试设备与环境")
        lines.append("")
        rows = [[str(k), str(v)] for k, v in equipment.items()]
        lines.append(markdown_table(["项目", "值"], rows))
        lines.append("")

    for joint in JOINT_NAMES:
        jdf = metrics[metrics["joint_name"].astype(str) == joint].copy()
        if jdf.empty:
            continue
        amp_vals = jdf["amplitude_deg"].dropna().unique()
        amp_text = f"，振幅 {amp_vals[0]:.1f}°" if len(amp_vals) else ""
        lines.append(f"## {joint}{amp_text}")
        lines.append("")

        rows = []
        for _, r in jdf.sort_values("target_speed_deg_s").iterrows():
            status = str(r.get("status", ""))
            if status == "skipped":
                note = str(r.get("skip_reason", ""))
                rows.append([
                    format_cell(r.get("target_speed_deg_s"), 0, " °/s"),
                    "Skipped",
                    "—",
                    "—",
                    "—",
                    "—",
                    "—",
                    "—",
                    "—",
                    "—",
                    "—",
                    "—",
                    note.replace("|", "/"),
                ])
            elif status == "executed_missing_data":
                rows.append([
                    format_cell(r.get("target_speed_deg_s"), 0, " °/s"),
                    "Executed, data missing",
                    "—", "—", "—", "—", "—", "—", "—", "—", "—", "—",
                    str(r.get("analysis_note", "")),
                ])
            else:
                note_parts = []
                if bool(r.get("feedback_watch", False)):
                    note_parts.append("feedback-watch")
                if bool(r.get("fb_velocity_over_limit", False)):
                    note_parts.append("反馈速度超限")
                note = ", ".join(note_parts)
                rows.append([
                    format_cell(r.get("target_speed_deg_s"), 0, " °/s"),
                    "Executed",
                    format_cell(r.get("rmse_deg"), 2, "°"),
                    format_cell(r.get("max_error_deg"), 2, "°"),
                    format_cell(r.get("rmse_over_amp_pct"), 1, "%"),
                    format_cell(r.get("position_overshoot_pct"), 1, "%"),
                    format_cell(r.get("velocity_overshoot_pct"), 1, "%"),
                    format_cell(r.get("peak_fb_velocity_deg_s"), 1, " °/s"),
                    format_cell(r.get("fit_peak_fb_velocity_deg_s"), 1, " °/s"),
                    format_cell(r.get("peak_torque"), 3, " Nm"),
                    f"{format_cell(r.get('delay_xcorr_refined_ms'), 2)}/{format_cell(r.get('delay_sinefit_ms'), 2)}",
                    f"{format_cell(r.get('command_dt_mean_ms'), 3)}/{format_cell(r.get('command_dt_max_ms'), 3)}/{format_cell(r.get('command_dt_p95_ms'), 3)}",
                    note,
                ])
        lines.append(markdown_table(
            [
                "目标速度",
                "状态",
                "RMSE",
                "最大误差",
                "RMSE/振幅",
                "位置超调量",
                "速度超调量",
                "实际峰值速度",
                "拟合峰值速度",
                "峰值力矩",
                "延迟 xcorr/sinefit ms",
                "Δt mean/max/p95 ms",
                "备注",
            ],
            rows,
        ))
        lines.append("")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def make_joint_figure(
    joint: str,
    metrics: pd.DataFrame,
    run_dir: Path,
    out_dir: Path,
    args: argparse.Namespace,
) -> Optional[Path]:
    jdf = metrics[
        (metrics["joint_name"].astype(str) == joint)
        & (metrics["status"].astype(str) == "executed")
        & (metrics["data_file_resolved"].notna())
    ].copy()
    if args.executed_only_figures:
        # Already only executed; kept for explicit CLI readability.
        pass
    if jdf.empty:
        return None

    jdf = jdf.sort_values("target_speed_deg_s")
    nrows = len(jdf)
    joint_idx = joint_index_from_name(joint)

    # Match the "large summary panel" style: rows=speed cases, cols=position/error/velocity/torque.
    fig_w = args.figure_width
    fig_h = max(args.figure_row_height * nrows, 6.0)
    fig, axes = plt.subplots(nrows=nrows, ncols=4, figsize=(fig_w, fig_h), squeeze=False)
    drop_text = f" (first {args.drop_first_cycles:g} cycle dropped)" if float(args.drop_first_cycles) > 0 else ""
    fig.suptitle(f"{joint} — Sinusoidal Tracking Performance at Different Speeds{drop_text}", fontsize=14, fontweight="bold")

    for row_idx, (_, r) in enumerate(jdf.iterrows()):
        data, _, _ = load_trial_arrays(r, run_dir)
        if data is None:
            continue

        data, trim_meta = trim_data_after_first_cycles(
            data=data,
            period_s=float(r.get("period_s", np.nan)),
            drop_first_cycles=float(args.drop_first_cycles),
        )

        t = pick_time_axis(data)
        # Prefer relative axis starting at 0 after the trimmed analysis window.
        if np.isfinite(t).any():
            t0 = np.nanmin(t)
            t_plot = t - t0
        else:
            t_plot = np.arange(len(data["q_cmd"]))

        q_cmd = np.asarray(data["q_cmd"][:, joint_idx], dtype=np.float64)
        q_fb = np.asarray(data["q_fb"][:, joint_idx], dtype=np.float64)
        err = q_fb - q_cmd
        v_fb, velocity_used = get_feedback_velocity(data, joint_idx, args.velocity_source, args.diff_smooth_window)
        torque, torque_used = get_torque(data, joint_idx, args.torque_source)

        speed = float(r["target_speed_deg_s"])
        amp = float(r["amplitude_deg"])
        vel_limit = float(r["vel_limit_deg_s"]) if pd.notna(r.get("vel_limit_deg_s")) else np.nan

        # 1) Position tracking
        ax = axes[row_idx, 0]
        ax.plot(t_plot, q_cmd, lw=1.0, label="Command")
        ax.plot(t_plot, q_fb, lw=0.9, ls="--", label="Feedback")
        ax.set_ylabel(f"{int(speed)}°/s\nPosition (°)")
        if row_idx == 0:
            ax.set_title("Position Tracking")
            ax.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.25)

        # 2) Error
        ax = axes[row_idx, 1]
        ax.plot(t_plot, err, lw=0.9)
        ax.axhline(0.0, lw=0.6, alpha=0.5)
        rmse = r.get("rmse_deg", np.nan)
        max_err = r.get("max_error_deg", np.nan)
        rmse_amp = r.get("rmse_over_amp_pct", np.nan)
        ax.set_ylabel("Error (°)")
        if row_idx == 0:
            ax.set_title("Tracking Error")
        ax.set_title(
            ax.get_title() + f"\nRMSE={format_cell(rmse,2,'°')}  Max={format_cell(max_err,2,'°')}  RMSE/A={format_cell(rmse_amp,1,'%')}",
            fontsize=8,
        )
        ax.grid(True, alpha=0.25)

        # 3) Velocity
        ax = axes[row_idx, 2]
        ax.plot(t_plot, v_fb, lw=0.9)
        if np.isfinite(vel_limit):
            ax.axhline(vel_limit, ls="--", lw=0.8, alpha=0.7)
            ax.axhline(-vel_limit, ls="--", lw=0.8, alpha=0.7)
        peak_v = r.get("peak_fb_velocity_deg_s", np.nan)
        ax.set_ylabel("Velocity (°/s)")
        if row_idx == 0:
            ax.set_title("Feedback Velocity")
        ax.set_title(ax.get_title() + f"\nPeak={format_cell(peak_v,1,' °/s')} ({velocity_used})", fontsize=8)
        ax.grid(True, alpha=0.25)

        # 4) Torque
        ax = axes[row_idx, 3]
        ax.plot(t_plot, torque, lw=0.9)
        peak_tq = r.get("peak_torque", np.nan)
        ax.set_ylabel("Torque")
        if row_idx == 0:
            ax.set_title("Joint Torque")
        ax.set_title(ax.get_title() + f"\nPeak={format_cell(peak_tq,3,' Nm')} ({torque_used})", fontsize=8)
        ax.grid(True, alpha=0.25)

        for col in range(4):
            axes[row_idx, col].set_xlabel("Time since analysis window start (s)")

    fig.tight_layout(rect=[0, 0.02, 1, 0.97])
    out_path = out_dir / f"{joint}_tracking_summary.png"
    fig.savefig(out_path, dpi=args.dpi)
    plt.close(fig)
    return out_path


def write_realtime_summary(metrics: pd.DataFrame, out_path: Path) -> None:
    cols = [
        "trial_index",
        "joint_name",
        "target_speed_deg_s",
        "status",
        "actual_duration_s",
        "sample_count",
        "analysis_drop_first_cycles",
        "analysis_start_time_s",
        "analysis_dropped_samples",
        "analysis_sample_count",
        "command_dt_mean_ms",
        "command_dt_max_ms",
        "command_dt_p95_ms",
        "command_jitter_mean_ms",
        "command_jitter_max_ms",
        "command_jitter_p95_ms",
        "negative_sleep_count",
        "overrun_count",
        "feedback_sample_count",
        "delay_xcorr_refined_ms",
        "delay_sinefit_ms",
        "phase_lag_deg",
        "xcorr_peak",
        "fit_amp_ratio_pct",
        "fit_peak_fb_velocity_deg_s",
    ]
    existing = [c for c in cols if c in metrics.columns]
    metrics[existing].to_csv(out_path, index=False, encoding="utf-8-sig")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze and plot Marvin M6 per-joint sine sweep results.")
    parser.add_argument("--result_root", type=str, default="results_per_joint_amp_sweep_200hz",
                        help="Result root relative to project root. Ignored if --run_dir is set. Windows-style relative separators are accepted on Linux.")
    parser.add_argument("--run_dir", type=str, default=None,
                        help="Specific run directory. If omitted, the latest run under --result_root is used. Windows-style relative separators are accepted on Linux.")
    parser.add_argument("--output_dir", type=str, default=None,
                        help="Output directory. Default: <run_dir>/analysis_report.")
    parser.add_argument("--joint_filter", nargs="*", default=None,
                        help="Optional joints to analyze, e.g. J3 J5 7. Default: all.")
    parser.add_argument("--velocity_source", choices=["diff", "sdk"], default="diff",
                        help="Velocity used for plot/table. diff = d(q_fb)/dt; sdk = dq_fb from sample.")
    parser.add_argument("--diff_smooth_window", type=int, default=1,
                        help="Odd moving-average window for differentiated velocity. 1 means no smoothing.")
    parser.add_argument("--drop_first_cycles", type=float, default=1.0,
                        help="Drop this many initial sine cycles before metrics/delay/figures. Default 1.0 starts analysis from the second cycle.")
    parser.add_argument("--torque_source", choices=["sToq", "cToq", "them"], default="sToq",
                        help="Torque signal used for plot/table.")
    parser.add_argument("--dpi", type=int, default=180, help="Figure DPI.")
    parser.add_argument("--figure_width", type=float, default=18.0, help="Figure width in inches.")
    parser.add_argument("--figure_row_height", type=float, default=2.5, help="Figure row height in inches.")
    parser.add_argument("--executed_only_figures", action="store_true", default=True,
                        help="Only executed trials are plotted. Skipped trials remain in tables.")
    return parser.parse_args()


def normalize_joint_filter(items: Optional[Sequence[str]]) -> Optional[List[str]]:
    if not items:
        return None
    out = []
    for item in items:
        text = str(item).strip().upper()
        if text.startswith("J"):
            text = text[1:]
        idx = int(text)
        if idx < 1 or idx > 7:
            raise ValueError(f"Invalid joint filter: {item!r}")
        out.append(f"J{idx}")
    return out


def main() -> int:
    args = parse_args()
    if float(args.drop_first_cycles) < 0.0:
        raise ValueError("drop_first_cycles must be >= 0")

    run_dir = resolve_input_run_dir(args)
    if not run_dir.exists():
        raise FileNotFoundError(f"Run directory does not exist: {run_dir}")

    out_dir = normalize_cli_path(args.output_dir).resolve() if args.output_dir else run_dir / "analysis_report"
    fig_dir = out_dir / "figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    fig_dir.mkdir(parents=True, exist_ok=True)

    summary = load_summary(run_dir)
    config = load_run_config(run_dir)
    joint_filter = normalize_joint_filter(args.joint_filter)

    if joint_filter:
        summary = summary[summary["joint_name"].astype(str).isin(joint_filter)].copy()

    print(f"[input] run_dir: {run_dir}")
    print(f"[input] summary rows: {len(summary)}")
    print(f"[output] analysis dir: {out_dir}")

    metrics, _loaded = build_metrics_table(summary, run_dir, args)

    metrics_csv = out_dir / "metrics_summary_all.csv"
    metrics.to_csv(metrics_csv, index=False, encoding="utf-8-sig")
    print(f"[write] {metrics_csv}")

    realtime_csv = out_dir / "realtime_summary.csv"
    write_realtime_summary(metrics, realtime_csv)
    print(f"[write] {realtime_csv}")

    delay_cols = [
        "trial_index", "joint_name", "target_speed_deg_s", "status",
        "analysis_drop_first_cycles", "analysis_start_time_s", "analysis_dropped_samples", "analysis_sample_count",
        "delay_xcorr_ms", "delay_xcorr_refined_ms", "delay_xcorr_samples", "delay_xcorr_refined_samples",
        "delay_sinefit_ms", "delay_method_diff_ms", "phase_lag_deg", "xcorr_peak",
        "sample_dt_ms_for_delay", "fit_amp_cmd_deg", "fit_amp_fb_deg", "fit_amp_ratio_pct",
        "fit_peak_fb_velocity_deg_s",
    ]
    delay_csv = out_dir / "delay_summary.csv"
    metrics[[c for c in delay_cols if c in metrics.columns]].to_csv(delay_csv, index=False, encoding="utf-8-sig")
    print(f"[write] {delay_csv}")

    md_path = out_dir / "metrics_tables.md"
    write_markdown_report(metrics, md_path, run_dir, args, config)
    print(f"[write] {md_path}")

    figure_paths: List[Path] = []
    joints_to_plot = joint_filter if joint_filter else JOINT_NAMES
    for joint in joints_to_plot:
        fig_path = make_joint_figure(joint, metrics, run_dir, fig_dir, args)
        if fig_path:
            figure_paths.append(fig_path)
            print(f"[figure] {fig_path}")
        else:
            print(f"[figure] skip {joint}: no executed sample data")

    # Save a small manifest for downstream report generation.
    manifest = {
        "run_dir": str(run_dir),
        "output_dir": str(out_dir),
        "metrics_summary_all": str(metrics_csv),
        "realtime_summary": str(realtime_csv),
        "delay_summary": str(delay_csv),
        "metrics_tables_md": str(md_path),
        "figures": [str(p) for p in figure_paths],
        "velocity_source": args.velocity_source,
        "diff_smooth_window": args.diff_smooth_window,
        "drop_first_cycles": args.drop_first_cycles,
        "torque_source": args.torque_source,
    }
    manifest_path = out_dir / "analysis_manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f"[write] {manifest_path}")

    print("[done] analysis complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
