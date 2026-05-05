import math
import threading
import time


NAN = float("nan")


def safe_get(data, path, default=None):
    cur = data
    for key in path:
        try:
            if isinstance(cur, dict):
                cur = cur[key]
            elif isinstance(cur, (list, tuple)) and isinstance(key, int):
                cur = cur[key]
            else:
                return default
        except Exception:
            return default
    return cur


def safe_get_multi(data, paths, default=None):
    for path in paths:
        value = safe_get(data, path, None)
        if value is not None:
            return value
    return default


def safe_vec(values, dim, fill_value=NAN):
    out = [fill_value] * int(dim)
    if not isinstance(values, (list, tuple)):
        return out

    n = min(int(dim), len(values))
    for i in range(n):
        try:
            out[i] = float(values[i])
        except Exception:
            out[i] = fill_value
    return out


def safe_float(value, default=NAN):
    try:
        return float(value)
    except Exception:
        return default


def any_finite(values):
    for v in values:
        try:
            if math.isfinite(float(v)):
                return True
        except Exception:
            pass
    return False


def vec_sub(a, b):
    n = min(len(a), len(b))
    out = [NAN] * n
    for i in range(n):
        ai = safe_float(a[i], NAN)
        bi = safe_float(b[i], NAN)
        if math.isfinite(ai) and math.isfinite(bi):
            out[i] = ai - bi
    return out


def start_luna_feedback_collector(robot, poll_period_s=0.005, stale_threshold_s=0.100):
    collector = {
        "robot": robot,
        "poll_period_s": max(0.001, float(poll_period_s)),
        "stale_threshold_s": max(0.0, float(stale_threshold_s)),
        "start_perf_counter_s": time.perf_counter(),
        "lock": threading.Lock(),
        "stop_event": threading.Event(),
        "thread": None,
        "latest_rt": None,
        "latest_sg": None,
        "latest_rt_perf_s": None,
        "latest_sg_perf_s": None,
        "feedback_seq": 0,
        "rt_exception_count": 0,
        "sg_exception_count": 0,
        "samples": [],
        "collector_join_timeout": False,
    }

    def _loop():
        while not collector["stop_event"].is_set():
            rt_data = None
            sg_data = None
            rt_ts = None
            sg_ts = None

            try:
                rt_data = robot.get_rt_dict()
                rt_ts = time.perf_counter()
            except Exception:
                with collector["lock"]:
                    collector["rt_exception_count"] += 1

            try:
                sg_data = robot.get_sg_dict()
                sg_ts = time.perf_counter()
            except Exception:
                with collector["lock"]:
                    collector["sg_exception_count"] += 1

            with collector["lock"]:
                if isinstance(rt_data, dict):
                    collector["latest_rt"] = rt_data
                    collector["latest_rt_perf_s"] = rt_ts
                if isinstance(sg_data, dict):
                    collector["latest_sg"] = sg_data
                    collector["latest_sg_perf_s"] = sg_ts
                collector["feedback_seq"] += 1

            collector["stop_event"].wait(collector["poll_period_s"])

    thread = threading.Thread(target=_loop, name="luna-feedback-collector", daemon=True)
    collector["thread"] = thread
    thread.start()
    return collector


def extract_group_feedback(rt_data, sg_data, group_type, group_index):
    missing_fields = []

    if group_type == "arm":
        idx = int(group_index)
        q_rt_cmd = safe_vec(
            safe_get_multi(
                rt_data,
                [
                    ["arms", idx, "cmd", "joint_pos"],
                ],
                None,
            ),
            7,
        )
        q_fb = safe_vec(
            safe_get_multi(
                rt_data,
                [
                    ["arms", idx, "fb", "joint_pos"],
                    ["arms", idx, "fb", "fb_pos"],
                ],
                None,
            ),
            7,
        )
        q_fb_vel = safe_vec(
            safe_get_multi(
                rt_data,
                [
                    ["arms", idx, "fb", "joint_vel"],
                    ["arms", idx, "fb", "fb_vel"],
                ],
                None,
            ),
            7,
        )
        fb_torque = safe_vec(
            safe_get_multi(
                rt_data,
                [
                    ["arms", idx, "fb", "joint_torque"],
                    ["arms", idx, "fb", "fb_sensor"],
                ],
                None,
            ),
            7,
        )
        ctoq = safe_vec(
            safe_get_multi(
                sg_data,
                [
                    ["arms", idx, "get", "joint_current"],
                    ["arms", idx, "get", "joint_torque"],
                ],
                None,
            ),
            7,
        )
        ext_pos = safe_vec(
            safe_get_multi(
                sg_data,
                [
                    ["arms", idx, "get", "joint_ext_pos"],
                    ["arms", idx, "get", "ext_pos"],
                ],
                None,
            ),
            7,
        )
        state = safe_get(rt_data, ["arms", idx, "state", "cur"], None)
        err_code = safe_get(rt_data, ["arms", idx, "state", "err"], None)

        if not any_finite(q_fb):
            missing_fields.append("rt.arms.fb.joint_pos")
        if not any_finite(q_fb_vel):
            missing_fields.append("rt.arms.fb.joint_vel")
        if not any_finite(fb_torque):
            missing_fields.append("rt.arms.fb.joint_torque")
        if not any_finite(ctoq):
            missing_fields.append("sg.arms.get.joint_current")
        if not any_finite(ext_pos):
            missing_fields.append("sg.arms.get.joint_ext_pos")

    elif group_type == "head":
        q_rt_cmd = safe_vec(safe_get(rt_data, ["head", "cmd_pos"], None), 2)
        q_fb = safe_vec(safe_get(rt_data, ["head", "fb_pos"], None), 2)
        q_fb_vel = [NAN, NAN]
        fb_torque = [NAN, NAN]
        ctoq = safe_vec(
            safe_get_multi(
                sg_data,
                [
                    ["head", "get", "current"],
                    ["head", "get", "joint_torque"],
                ],
                None,
            ),
            2,
        )
        ext_pos = safe_vec(safe_get(sg_data, ["head", "get", "ext_pos"], None), 2)
        state = safe_get(rt_data, ["head", "state", "cur"], None)
        err_code = safe_get(rt_data, ["head", "state", "err"], None)

        if not any_finite(q_fb):
            missing_fields.append("rt.head.fb_pos")
        missing_fields.append("rt.head.fb_vel")
        missing_fields.append("rt.head.fb_torque")
        if not any_finite(ctoq):
            missing_fields.append("sg.head.get.current")
        if not any_finite(ext_pos):
            missing_fields.append("sg.head.get.ext_pos")

    elif group_type == "body":
        q_rt_cmd_full = safe_vec(safe_get(rt_data, ["body", "cmd_pos"], None), 6)
        q_fb_full = safe_vec(safe_get(rt_data, ["body", "fb_pos"], None), 6)
        q_fb_vel_full = safe_vec(safe_get(rt_data, ["body", "fb_vel"], None), 6)
        fb_torque_full = safe_vec(
            safe_get_multi(
                rt_data,
                [
                    ["body", "fb_torque"],
                    ["body", "fb_sensor"],
                ],
                None,
            ),
            6,
        )
        ctoq_full = safe_vec(
            safe_get_multi(
                sg_data,
                [
                    ["body", "get", "current"],
                    ["body", "get", "joint_torque"],
                ],
                None,
            ),
            6,
        )
        ext_pos_full = safe_vec(safe_get(sg_data, ["body", "get", "ext_pos"], None), 6)

        q_rt_cmd = q_rt_cmd_full[:6]
        q_fb = q_fb_full[:6]
        q_fb_vel = q_fb_vel_full[:6]
        fb_torque = fb_torque_full[:6]
        ctoq = ctoq_full[:6]
        ext_pos = ext_pos_full[:6]
        state = safe_get(rt_data, ["body", "state", "cur"], None)
        err_code = safe_get(rt_data, ["body", "state", "err"], None)

        if not any_finite(q_fb):
            missing_fields.append("rt.body.fb_pos")
        if not any_finite(q_fb_vel):
            missing_fields.append("rt.body.fb_vel")
        if not any_finite(fb_torque):
            missing_fields.append("rt.body.fb_torque")
        if not any_finite(ctoq):
            missing_fields.append("sg.body.get.current")
        if not any_finite(ext_pos):
            missing_fields.append("sg.body.get.ext_pos")

    else:
        raise ValueError(f"Unsupported group_type: {group_type}")

    return {
        "q_rt_cmd_deg": q_rt_cmd,
        "q_fb_deg": q_fb,
        "q_fb_vel": q_fb_vel,
        "fb_torque": fb_torque,
        "ctoq": ctoq,
        "ext_pos": ext_pos,
        "state": state,
        "err_code": err_code,
        "missing_fields": sorted(set(missing_fields)),
    }


def append_luna_feedback_sample(
    collector,
    group_name,
    group_type,
    group_index,
    joint_idx,
    q_cmd_deg,
    t_cmd_s,
    send_ts_perf_s,
    test_name=None,
):
    joint_dim_map = {"arm": 7, "head": 2, "body": 6}
    joint_dim = joint_dim_map[group_type]

    q_cmd = safe_vec(q_cmd_deg, joint_dim)

    with collector["lock"]:
        latest_rt = collector.get("latest_rt")
        latest_sg = collector.get("latest_sg")
        latest_rt_perf_s = collector.get("latest_rt_perf_s")
        latest_sg_perf_s = collector.get("latest_sg_perf_s")
        feedback_seq = int(collector.get("feedback_seq", 0))
        stale_threshold_s = float(collector.get("stale_threshold_s", 0.1))

    now_perf_s = time.perf_counter()

    rt_age_s = NAN
    sg_age_s = NAN
    if latest_rt_perf_s is not None:
        rt_age_s = now_perf_s - float(latest_rt_perf_s)
    if latest_sg_perf_s is not None:
        sg_age_s = now_perf_s - float(latest_sg_perf_s)

    feedback_stale = False
    if latest_rt is None or latest_sg is None:
        feedback_stale = True
    if math.isfinite(rt_age_s) and (rt_age_s < 0.0 or rt_age_s > stale_threshold_s):
        feedback_stale = True
    if math.isfinite(sg_age_s) and (sg_age_s < 0.0 or sg_age_s > stale_threshold_s):
        feedback_stale = True

    extracted = extract_group_feedback(latest_rt, latest_sg, group_type, group_index)

    q_fb = extracted["q_fb_deg"]
    q_err = vec_sub(q_fb, q_cmd)

    active_q_cmd = safe_float(q_cmd[joint_idx] if joint_idx < len(q_cmd) else NAN, NAN)
    active_q_fb = safe_float(q_fb[joint_idx] if joint_idx < len(q_fb) else NAN, NAN)
    active_q_err = safe_float(q_err[joint_idx] if joint_idx < len(q_err) else NAN, NAN)

    q_fb_vel = extracted["q_fb_vel"]
    active_q_fb_vel = safe_float(q_fb_vel[joint_idx] if joint_idx < len(q_fb_vel) else NAN, NAN)

    ctoq = extracted["ctoq"]
    active_ctoq = safe_float(ctoq[joint_idx] if joint_idx < len(ctoq) else NAN, NAN)

    fb_torque = extracted["fb_torque"]
    active_fb_torque = safe_float(fb_torque[joint_idx] if joint_idx < len(fb_torque) else NAN, NAN)

    ext_pos = extracted["ext_pos"]
    active_ext_pos = safe_float(ext_pos[joint_idx] if joint_idx < len(ext_pos) else NAN, NAN)

    frame_serial = safe_get(latest_rt, ["frame_serial"], None)
    sg_frame_serial = safe_get(latest_sg, ["frame_serial"], None)

    t0 = float(collector["start_perf_counter_s"])
    t_fb_rt_s = NAN if latest_rt_perf_s is None else (float(latest_rt_perf_s) - t0)
    t_fb_sg_s = NAN if latest_sg_perf_s is None else (float(latest_sg_perf_s) - t0)

    sample = {
        "test_name": test_name,
        "group_name": group_name,
        "group_type": group_type,
        "group_index": group_index,
        "joint_idx": int(joint_idx),
        "joint_name": f"J{joint_idx + 1}",
        "t_cmd_s": float(t_cmd_s),
        "send_ts_perf_s": float(send_ts_perf_s),
        "t_fb_rt_s": t_fb_rt_s,
        "t_fb_sg_s": t_fb_sg_s,
        "rt_age_s": rt_age_s,
        "sg_age_s": sg_age_s,
        "rt_seq": frame_serial,
        "frame_serial": frame_serial,
        "sg_seq": sg_frame_serial,
        "feedback_seq": feedback_seq,
        "feedback_stale": bool(feedback_stale),
        "state": extracted["state"],
        "err_code": extracted["err_code"],
        "q_cmd_deg": q_cmd,
        "q_fb_deg": q_fb,
        "q_err_deg": q_err,
        "active_q_cmd_deg": active_q_cmd,
        "active_q_fb_deg": active_q_fb,
        "active_q_err_deg": active_q_err,
        "q_fb_vel": q_fb_vel,
        "active_q_fb_vel": active_q_fb_vel,
        "ctoq": ctoq,
        "active_ctoq": active_ctoq,
        "fb_torque": fb_torque,
        "active_fb_torque": active_fb_torque,
        "ext_pos": ext_pos,
        "active_ext_pos": active_ext_pos,
        "q_rt_cmd_deg": extracted["q_rt_cmd_deg"],
        "missing_fields": extracted["missing_fields"],
    }

    collector["samples"].append(sample)


def finalize_luna_feedback_collector(collector):
    if collector is None:
        return None

    stop_event = collector.get("stop_event")
    thread = collector.get("thread")

    if stop_event is not None:
        stop_event.set()

    if thread is not None and hasattr(thread, "join"):
        thread.join(timeout=2.0)
        if thread.is_alive():
            collector["collector_join_timeout"] = True

    return {
        "sample_count": len(collector.get("samples", [])),
        "feedback_seq": int(collector.get("feedback_seq", 0)),
        "rt_exception_count": int(collector.get("rt_exception_count", 0)),
        "sg_exception_count": int(collector.get("sg_exception_count", 0)),
        "collector_join_timeout": bool(collector.get("collector_join_timeout", False)),
        "poll_period_s": float(collector.get("poll_period_s", 0.0)),
        "stale_threshold_s": float(collector.get("stale_threshold_s", 0.0)),
        "samples": collector.get("samples", []),
    }
