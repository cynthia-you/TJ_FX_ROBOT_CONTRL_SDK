import csv
import json
import math
import os
import statistics
from datetime import datetime

from luna_feedback_tools import NAN, any_finite, safe_float


PLOT_AVAILABLE = True
PLOT_IMPORT_ERROR = ""
try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception as plot_exc:
    PLOT_AVAILABLE = False
    PLOT_IMPORT_ERROR = str(plot_exc)


def to_jsonable(value):
    if isinstance(value, dict):
        return {k: to_jsonable(v) for k, v in value.items()}
    if isinstance(value, list):
        return [to_jsonable(v) for v in value]
    if isinstance(value, tuple):
        return [to_jsonable(v) for v in value]
    if isinstance(value, float):
        if math.isfinite(value):
            return value
        return None
    return value


def csv_cell(value):
    if isinstance(value, (list, dict, tuple)):
        return json.dumps(to_jsonable(value), ensure_ascii=False)
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    return value


def percentile(values, p):
    finite = []
    for v in values:
        fv = safe_float(v, NAN)
        if math.isfinite(fv):
            finite.append(fv)
    if not finite:
        return None

    finite.sort()
    if len(finite) == 1:
        return finite[0]

    rank = (len(finite) - 1) * float(p) / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return finite[lo]
    w = rank - lo
    return finite[lo] * (1.0 - w) + finite[hi] * w


def mean_or_none(values):
    finite = []
    for v in values:
        fv = safe_float(v, NAN)
        if math.isfinite(fv):
            finite.append(fv)
    if not finite:
        return None
    return sum(finite) / float(len(finite))


def std_or_none(values):
    finite = []
    for v in values:
        fv = safe_float(v, NAN)
        if math.isfinite(fv):
            finite.append(fv)
    if not finite:
        return None
    if len(finite) == 1:
        return 0.0
    return statistics.pstdev(finite)


def max_or_none(values):
    finite = []
    for v in values:
        fv = safe_float(v, NAN)
        if math.isfinite(fv):
            finite.append(fv)
    if not finite:
        return None
    return max(finite)


def abs_stats(values, with_p95=False):
    abs_values = []
    for v in values:
        fv = safe_float(v, NAN)
        if math.isfinite(fv):
            abs_values.append(abs(fv))

    if not abs_values:
        if with_p95:
            return None, None, None
        return None, None

    max_abs = max(abs_values)
    mean_abs = sum(abs_values) / float(len(abs_values))
    if with_p95:
        return max_abs, mean_abs, percentile(abs_values, 95.0)
    return max_abs, mean_abs


def bucket_samples(samples):
    buckets = {}
    for sample in samples:
        gname = sample.get("group_name")
        jidx_raw = sample.get("joint_idx")
        try:
            jidx = int(jidx_raw)
        except Exception:
            continue
        key = (gname, jidx)
        if key not in buckets:
            buckets[key] = []
        buckets[key].append(sample)
    return buckets


def compute_joint_metrics(samples_joint, dt):
    sample_count = len(samples_joint)
    stale_count = sum(1 for s in samples_joint if bool(s.get("feedback_stale", False)))
    stale_ratio = (float(stale_count) / float(sample_count)) if sample_count > 0 else None

    active_q_err = [safe_float(s.get("active_q_err_deg"), NAN) for s in samples_joint]
    active_ctoq = [safe_float(s.get("active_ctoq"), NAN) for s in samples_joint]
    active_q_fb_vel = [safe_float(s.get("active_q_fb_vel"), NAN) for s in samples_joint]
    active_fb_torque = [safe_float(s.get("active_fb_torque"), NAN) for s in samples_joint]

    max_abs_error_deg, mean_abs_error_deg, p95_abs_error_deg = abs_stats(active_q_err, with_p95=True)
    max_abs_ctoq, mean_abs_ctoq = abs_stats(active_ctoq, with_p95=False)
    max_abs_fb_vel, mean_abs_fb_vel = abs_stats(active_q_fb_vel, with_p95=False)
    max_abs_fb_torque, mean_abs_fb_torque = abs_stats(active_fb_torque, with_p95=False)

    t_cmd = [safe_float(s.get("t_cmd_s"), NAN) for s in samples_joint]
    finite_t = [v for v in t_cmd if math.isfinite(v)]
    delta_ms = []
    if len(finite_t) >= 2:
        for i in range(1, len(finite_t)):
            delta_ms.append((finite_t[i] - finite_t[i - 1]) * 1000.0)

    dt_ms = float(dt) * 1000.0
    jitter_ms = [d - dt_ms for d in delta_ms]

    timing = {
        "delta_t_mean_ms": mean_or_none(delta_ms),
        "delta_t_std_ms": std_or_none(delta_ms),
        "delta_t_max_ms": max_or_none(delta_ms),
        "delta_t_p95_ms": percentile(delta_ms, 95.0),
        "jitter_mean_ms": mean_or_none(jitter_ms),
        "jitter_std_ms": std_or_none(jitter_ms),
        "jitter_max_ms": max_or_none(jitter_ms),
        "jitter_p95_ms": percentile(jitter_ms, 95.0),
    }

    missing_fields = set()
    for sample in samples_joint:
        for mf in sample.get("missing_fields", []):
            missing_fields.add(mf)

    velocity_source = "rt" if any_finite(active_q_fb_vel) else "missing"
    torque_source = "rt" if any_finite(active_fb_torque) else "missing"

    return {
        "sample_count": sample_count,
        "stale_ratio": stale_ratio,
        "max_abs_error_deg": max_abs_error_deg,
        "mean_abs_error_deg": mean_abs_error_deg,
        "p95_abs_error_deg": p95_abs_error_deg,
        "max_abs_ctoq": max_abs_ctoq,
        "mean_abs_ctoq": mean_abs_ctoq,
        "max_abs_fb_vel": max_abs_fb_vel,
        "mean_abs_fb_vel": mean_abs_fb_vel,
        "max_abs_fb_torque": max_abs_fb_torque,
        "mean_abs_fb_torque": mean_abs_fb_torque,
        "timing": timing,
        "velocity_source": velocity_source,
        "torque_source": torque_source,
        "missing_fields": sorted(missing_fields),
    }


def compute_metrics(samples, dt, group_tests):
    buckets = bucket_samples(samples)

    groups = {}
    for item in group_tests:
        group_name = item["group_name"]
        groups[group_name] = {
            "group_type": item["group_type"],
            "group_index": item["group_index"],
            "joint_dim": int(item["joint_count"]),
            "tests": {},
        }

    for (group_name, joint_idx), items in sorted(buckets.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        joint_metrics = compute_joint_metrics(items, dt)
        groups[group_name]["tests"][f"J{joint_idx + 1}"] = joint_metrics

    all_delta_ms = []
    for _, items in buckets.items():
        seg_t_cmd = [safe_float(s.get("t_cmd_s"), NAN) for s in items]
        seg_t_cmd = [v for v in seg_t_cmd if math.isfinite(v)]
        if len(seg_t_cmd) < 2:
            continue
        seg_t_cmd.sort()
        for i in range(1, len(seg_t_cmd)):
            all_delta_ms.append((seg_t_cmd[i] - seg_t_cmd[i - 1]) * 1000.0)

    dt_ms = float(dt) * 1000.0
    all_jitter_ms = [d - dt_ms for d in all_delta_ms]

    timing_global = {
        "delta_t_mean_ms": mean_or_none(all_delta_ms),
        "delta_t_std_ms": std_or_none(all_delta_ms),
        "delta_t_max_ms": max_or_none(all_delta_ms),
        "delta_t_p95_ms": percentile(all_delta_ms, 95.0),
        "jitter_mean_ms": mean_or_none(all_jitter_ms),
        "jitter_std_ms": std_or_none(all_jitter_ms),
        "jitter_max_ms": max_or_none(all_jitter_ms),
        "jitter_p95_ms": percentile(all_jitter_ms, 95.0),
    }

    return {
        "groups": groups,
        "timing_global": timing_global,
        "timing_note": "Global timing is computed within each group/joint segment only; intervals between joint tests are excluded.",
    }


def save_raw_csv(samples, csv_path):
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    if not samples:
        with open(csv_path, "w", encoding="utf-8-sig", newline="") as f:
            f.write("\n")
        return csv_path

    fieldnames = [
        "test_name",
        "group_name",
        "group_type",
        "group_index",
        "joint_idx",
        "joint_name",
        "t_cmd_s",
        "send_ts_perf_s",
        "t_fb_rt_s",
        "t_fb_sg_s",
        "rt_age_s",
        "sg_age_s",
        "rt_seq",
        "frame_serial",
        "sg_seq",
        "feedback_seq",
        "feedback_stale",
        "state",
        "err_code",
        "q_cmd_deg",
        "q_fb_deg",
        "q_err_deg",
        "active_q_cmd_deg",
        "active_q_fb_deg",
        "active_q_err_deg",
        "q_fb_vel",
        "active_q_fb_vel",
        "ctoq",
        "active_ctoq",
        "fb_torque",
        "active_fb_torque",
        "ext_pos",
        "active_ext_pos",
        "q_rt_cmd_deg",
        "missing_fields",
    ]

    with open(csv_path, "w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            row = {}
            for key in fieldnames:
                row[key] = csv_cell(sample.get(key))
            writer.writerow(row)

    return csv_path


def save_result_json(result, json_path):
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(to_jsonable(result), f, ensure_ascii=False, indent=2)
    return json_path


def _plot_line_series(path, title, t_list, series_list, y_label):
    if not PLOT_AVAILABLE:
        return None

    fig, ax = plt.subplots(figsize=(10, 4.5))
    has_any = False

    for series in series_list:
        y = series["y"]
        label = series.get("label", "")
        color = series.get("color", None)

        n = min(len(t_list), len(y))
        xs = []
        ys = []
        for i in range(n):
            xv = safe_float(t_list[i], NAN)
            yv = safe_float(y[i], NAN)
            if math.isfinite(xv) and math.isfinite(yv):
                xs.append(xv)
                ys.append(yv)

        if xs:
            ax.plot(xs, ys, linewidth=1.1, label=label, color=color)
            has_any = True

    if not has_any:
        ax.text(0.5, 0.5, "No valid data", ha="center", va="center", transform=ax.transAxes)

    ax.set_title(title)
    ax.set_xlabel("t (s)")
    ax.set_ylabel(y_label)
    ax.grid(True, linestyle="--", alpha=0.4)
    if has_any:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _plot_group_summary(path, group_name, group_metrics):
    if not PLOT_AVAILABLE:
        return None

    tests = group_metrics.get("tests", {})
    labels = []
    max_err = []
    p95_err = []

    for joint_name in sorted(tests.keys(), key=lambda x: int(x[1:])):
        labels.append(joint_name)
        m = tests[joint_name]
        max_v = m.get("max_abs_error_deg")
        p95_v = m.get("p95_abs_error_deg")
        max_err.append(safe_float(max_v, NAN))
        p95_err.append(safe_float(p95_v, NAN))

    fig, ax = plt.subplots(figsize=(10, 4.8))
    if not labels:
        ax.text(0.5, 0.5, "No metrics data", ha="center", va="center", transform=ax.transAxes)
    else:
        x = list(range(len(labels)))
        w = 0.38
        ax.bar([i - w / 2.0 for i in x], max_err, width=w, label="max_abs_error_deg")
        ax.bar([i + w / 2.0 for i in x], p95_err, width=w, label="p95_abs_error_deg")
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.legend(loc="best")

    ax.set_title(f"{group_name} summary")
    ax.set_xlabel("Joint")
    ax.set_ylabel("deg")
    ax.grid(True, linestyle="--", alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def generate_luna_plots(samples, metrics, output_dir, tag):
    os.makedirs(output_dir, exist_ok=True)

    plot_paths = {
        "per_joint": {},
        "group_summary": {},
    }

    if not PLOT_AVAILABLE:
        plot_paths["plot_warning"] = f"matplotlib unavailable: {PLOT_IMPORT_ERROR}"
        return plot_paths

    buckets = bucket_samples(samples)

    for (group_name, joint_idx), items in sorted(buckets.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        joint_name = f"J{joint_idx + 1}"
        key = f"{group_name}_{joint_name}"

        t_list = [safe_float(s.get("t_cmd_s"), NAN) for s in items]
        q_cmd = [safe_float(s.get("active_q_cmd_deg"), NAN) for s in items]
        q_fb = [safe_float(s.get("active_q_fb_deg"), NAN) for s in items]
        q_err = [safe_float(s.get("active_q_err_deg"), NAN) for s in items]
        ctoq = [safe_float(s.get("active_ctoq"), NAN) for s in items]
        q_vel = [safe_float(s.get("active_q_fb_vel"), NAN) for s in items]
        fb_torque = [safe_float(s.get("active_fb_torque"), NAN) for s in items]

        pos_path = os.path.join(output_dir, f"{group_name}_{joint_name}_position_{tag}.png")
        err_path = os.path.join(output_dir, f"{group_name}_{joint_name}_error_{tag}.png")
        ctoq_path = os.path.join(output_dir, f"{group_name}_{joint_name}_ctoq_{tag}.png")
        vel_path = os.path.join(output_dir, f"{group_name}_{joint_name}_velocity_{tag}.png")
        torque_path = os.path.join(output_dir, f"{group_name}_{joint_name}_rt_torque_{tag}.png")

        _plot_line_series(
            pos_path,
            f"{group_name} {joint_name} position tracking",
            t_list,
            [
                {"y": q_cmd, "label": "active_q_cmd_deg", "color": "tab:blue"},
                {"y": q_fb, "label": "active_q_fb_deg", "color": "tab:green"},
            ],
            "deg",
        )
        _plot_line_series(
            err_path,
            f"{group_name} {joint_name} position error",
            t_list,
            [{"y": q_err, "label": "active_q_err_deg", "color": "tab:red"}],
            "deg",
        )
        _plot_line_series(
            ctoq_path,
            f"{group_name} {joint_name} ctoq",
            t_list,
            [{"y": ctoq, "label": "active_ctoq", "color": "tab:orange"}],
            "ctoq",
        )
        _plot_line_series(
            vel_path,
            f"{group_name} {joint_name} feedback velocity",
            t_list,
            [{"y": q_vel, "label": "active_q_fb_vel", "color": "tab:purple"}],
            "deg/s",
        )
        _plot_line_series(
            torque_path,
            f"{group_name} {joint_name} rt torque",
            t_list,
            [{"y": fb_torque, "label": "active_fb_torque", "color": "tab:brown"}],
            "torque",
        )

        plot_paths["per_joint"][key] = {
            "position": pos_path,
            "error": err_path,
            "ctoq": ctoq_path,
            "velocity": vel_path,
            "rt_torque": torque_path,
        }

    groups = metrics.get("groups", {})
    for group_name, group_metrics in groups.items():
        summary_path = os.path.join(output_dir, f"{group_name}_summary_{tag}.png")
        _plot_group_summary(summary_path, group_name, group_metrics)
        plot_paths["group_summary"][group_name] = summary_path

    return plot_paths


def summarize_key_metrics(metrics):
    lines = []
    groups = metrics.get("groups", {})
    for group_name in sorted(groups.keys()):
        tests = groups[group_name].get("tests", {})
        for joint_name in sorted(tests.keys(), key=lambda x: int(x[1:])):
            m = tests[joint_name]
            lines.append(
                {
                    "group": group_name,
                    "joint": joint_name,
                    "sample_count": m.get("sample_count"),
                    "stale_ratio": m.get("stale_ratio"),
                    "max_abs_error_deg": m.get("max_abs_error_deg"),
                    "p95_abs_error_deg": m.get("p95_abs_error_deg"),
                    "velocity_source": m.get("velocity_source"),
                    "torque_source": m.get("torque_source"),
                }
            )
    return lines


def save_and_plot_luna_result(
    test_name,
    collector_summary,
    group_tests,
    result_root_dir,
    robot_ip,
    dt,
    sin_freq_hz,
    time_per_joint,
    vel_ratio,
    acc_ratio,
    poll_period_s,
    stale_threshold_s,
):
    postprocess_errors = []

    tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(result_root_dir, tag)
    os.makedirs(output_dir, exist_ok=True)

    samples = []
    if isinstance(collector_summary, dict):
        samples = collector_summary.get("samples", [])

    csv_path = ""
    try:
        csv_path = os.path.join(output_dir, f"raw_samples_{tag}.csv")
        save_raw_csv(samples, csv_path)
    except Exception as exc:
        postprocess_errors.append(f"save_raw_csv failed: {exc}")

    metrics = {}
    try:
        metrics = compute_metrics(samples, dt, group_tests)
    except Exception as exc:
        postprocess_errors.append(f"compute_metrics failed: {exc}")
        metrics = {}

    plot_paths = {}
    try:
        plot_paths = generate_luna_plots(samples, metrics, output_dir, tag)
    except Exception as exc:
        postprocess_errors.append(f"generate_luna_plots failed: {exc}")
        plot_paths = {}

    result = {
        "meta": {
            "test_name": test_name,
            "robot_ip": robot_ip,
            "dt": dt,
            "ctrl_hz": (1.0 / dt) if dt > 0 else None,
            "sin_freq_hz": sin_freq_hz,
            "time_per_joint_s": time_per_joint,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "velocity_ratio": vel_ratio,
            "acc_ratio": acc_ratio,
            "poll_period_s": poll_period_s,
            "stale_threshold_s": stale_threshold_s,
        },
        "collector": collector_summary,
        "samples": samples,
        "groups": metrics.get("groups", {}),
        "metrics": metrics,
        "plot_paths": plot_paths,
        "postprocess_errors": postprocess_errors,
    }

    json_path = ""
    try:
        json_path = os.path.join(output_dir, f"result_{tag}.json")
        save_result_json(result, json_path)
    except Exception as exc:
        postprocess_errors.append(f"save_result_json failed: {exc}")

    output = {
        "output_dir": output_dir,
        "csv_path": csv_path,
        "json_path": json_path,
        "metrics": metrics,
        "plot_paths": plot_paths,
        "postprocess_errors": postprocess_errors,
    }
    return output


def print_luna_output_summary(output):
    output_dir = output.get("output_dir", "")
    csv_path = output.get("csv_path", "")
    json_path = output.get("json_path", "")
    metrics = output.get("metrics", {})
    plot_paths = output.get("plot_paths", {})
    postprocess_errors = output.get("postprocess_errors", [])

    print("==== LUNA_JOINT_TEST outputs ====")
    print(f"output_dir: {output_dir}")
    if csv_path:
        print(f"csv: {csv_path}")
    if json_path:
        print(f"json: {json_path}")

    per_joint_paths = plot_paths.get("per_joint", {})
    if isinstance(per_joint_paths, dict):
        for key in sorted(per_joint_paths.keys()):
            path_info = per_joint_paths[key]
            pos_path = path_info.get("position") if isinstance(path_info, dict) else ""
            print(f"plot_position_{key}: {pos_path}")

    group_summary_paths = plot_paths.get("group_summary", {})
    if isinstance(group_summary_paths, dict):
        for group_name in sorted(group_summary_paths.keys()):
            print(f"plot_summary_{group_name}: {group_summary_paths[group_name]}")

    if postprocess_errors:
        print("postprocess warnings:")
        for msg in postprocess_errors:
            print(f"  - {msg}")

    print("==== Key metrics ====")
    for item in summarize_key_metrics(metrics):
        print(
            f"{item['group']} {item['joint']}: "
            f"N={item['sample_count']}, "
            f"stale_ratio={item['stale_ratio']}, "
            f"max_abs_err={item['max_abs_error_deg']}, "
            f"p95_abs_err={item['p95_abs_error_deg']}, "
            f"vel_src={item['velocity_source']}, "
            f"torque_src={item['torque_source']}"
        )

    timing_global = metrics.get("timing_global", {})
    if timing_global:
        print("==== Timing global ====")
        print(f"delta_t_mean_ms: {timing_global.get('delta_t_mean_ms')}")
        print(f"delta_t_std_ms: {timing_global.get('delta_t_std_ms')}")
        print(f"delta_t_max_ms: {timing_global.get('delta_t_max_ms')}")
        print(f"delta_t_p95_ms: {timing_global.get('delta_t_p95_ms')}")
        print(f"jitter_mean_ms: {timing_global.get('jitter_mean_ms')}")
        print(f"jitter_std_ms: {timing_global.get('jitter_std_ms')}")
        print(f"jitter_max_ms: {timing_global.get('jitter_max_ms')}")
        print(f"jitter_p95_ms: {timing_global.get('jitter_p95_ms')}")
