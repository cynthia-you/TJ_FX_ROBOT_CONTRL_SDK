import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


ARM_LEFT_INDEX = 0
ARM_RIGHT_INDEX = 1
JOINT_DIM = 7
NAN = float("nan")


def _safe_joint_vec7(values: Any) -> List[float]:
    """把输入整理为长度 7 的关节向量，不足补 NaN。"""
    out = [NAN] * JOINT_DIM
    if values is None:
        return out

    if isinstance(values, np.ndarray):
        src = values.reshape(-1).tolist()
    elif isinstance(values, (list, tuple)):
        src = list(values)
    else:
        src = []

    n = min(JOINT_DIM, len(src))
    for i in range(n):
        try:
            out[i] = float(src[i])
        except Exception:
            out[i] = NAN
    return out


def _safe_arm_output(sub_data: Any, arm_index: int) -> Dict[str, Any]:
    """从缓存反馈里安全取某个 arm 的输出字典。"""
    if not isinstance(sub_data, dict):
        return {}
    outputs = sub_data.get("outputs", None)
    if not isinstance(outputs, list):
        return {}
    if arm_index < 0 or arm_index >= len(outputs):
        return {}
    out = outputs[arm_index]
    return out if isinstance(out, dict) else {}


def _subscribe_once(collector: Dict[str, Any]) -> Dict[str, Any]:
    """底层单次采样，参考专业脚本的 subscribe_once 思路。"""
    robot = collector["robot"]
    dcss = collector["dcss"]
    return robot.subscribe(dcss)


def _get_latest_feedback_cached(collector: Dict[str, Any]) -> Tuple[Any, Optional[float], int, float]:
    """前台读取最新缓存，不做 subscribe。"""
    with collector["lock"]:
        latest_sub_data = collector.get("latest_sub_data")
        latest_feedback_perf_s = collector.get("latest_feedback_perf_s")
        feedback_seq = int(collector.get("feedback_seq", 0))
        stale_threshold_s = float(collector.get("stale_threshold_s", 0.100))
    return latest_sub_data, latest_feedback_perf_s, feedback_seq, stale_threshold_s


def _feedback_collect_loop(collector: Dict[str, Any]) -> None:
    """后台线程：仅做 subscribe + 更新时间戳缓存，尽量保持最薄。"""
    stop_event = collector["stop_event"]
    poll_period_s = float(collector["poll_period_s"])

    while not stop_event.is_set():
        try:
            sub_data = _subscribe_once(collector)
            now_perf_s = time.perf_counter()
            with collector["lock"]:
                collector["latest_sub_data"] = sub_data
                collector["latest_feedback_perf_s"] = float(now_perf_s)
                collector["feedback_seq"] = int(collector.get("feedback_seq", 0)) + 1
        except Exception:
            # 异常只计数，不让线程退出
            with collector["lock"]:
                collector["feedback_exception_count"] = int(collector.get("feedback_exception_count", 0)) + 1
        finally:
            stop_event.wait(poll_period_s)


def start_feedback_collector(
    robot,
    dcss,
    collection_mode: str = "async",
    poll_period_s: float = 0.002,
    stale_threshold_s: float = 0.100,
) -> Dict[str, Any]:
    """启动反馈采集器。默认异步后台线程缓存反馈。"""
    mode = str(collection_mode).strip().lower()
    poll_period = max(0.0005, float(poll_period_s))
    stale_threshold = max(0.0, float(stale_threshold_s))

    collector: Dict[str, Any] = {
        "robot": robot,
        "dcss": dcss,
        "collection_mode": mode,
        "poll_period_s": poll_period,
        "stale_threshold_s": stale_threshold,
        "samples": [],
        "lock": threading.Lock(),
        "latest_sub_data": None,
        "latest_feedback_perf_s": None,
        "feedback_seq": 0,
        "feedback_exception_count": 0,
        "thread": None,
        "stop_event": threading.Event(),
        "start_perf_counter_s": time.perf_counter(),
    }

    if mode == "async":
        thread = threading.Thread(
            target=_feedback_collect_loop,
            args=(collector,),
            daemon=True,
            name="nvidia-feedback-cache-thread",
        )
        collector["thread"] = thread
        thread.start()
    else:
        # 非 async 模式仅做一次预采样，append 仍走缓存读取逻辑
        try:
            sub_data = _subscribe_once(collector)
            now_perf_s = time.perf_counter()
            with collector["lock"]:
                collector["latest_sub_data"] = sub_data
                collector["latest_feedback_perf_s"] = float(now_perf_s)
                collector["feedback_seq"] = 1
        except Exception:
            with collector["lock"]:
                collector["feedback_exception_count"] = 1

    return collector


def append_feedback_sample(
    collector: Dict[str, Any],
    left_q_cmd_deg,
    right_q_cmd_deg,
    t_cmd_s: Optional[float] = None,
    send_ts_perf_s: Optional[float] = None,
) -> None:
    """前台每周期仅做：读缓存 + 记录命令和反馈，不做 subscribe。"""
    start_perf_s = float(collector["start_perf_counter_s"])
    send_ts = float(send_ts_perf_s) if send_ts_perf_s is not None else time.perf_counter()
    cmd_t = float(t_cmd_s) if t_cmd_s is not None else float(send_ts - start_perf_s)

    latest_sub_data, latest_feedback_perf_s, feedback_seq, stale_threshold_s = _get_latest_feedback_cached(collector)

    left_q_cmd = _safe_joint_vec7(left_q_cmd_deg)
    right_q_cmd = _safe_joint_vec7(right_q_cmd_deg)

    if latest_sub_data is None or latest_feedback_perf_s is None:
        left_q_fb = [NAN] * JOINT_DIM
        left_s_to_q = [NAN] * JOINT_DIM
        left_firc_dot = [NAN] * JOINT_DIM

        right_q_fb = [NAN] * JOINT_DIM
        right_s_to_q = [NAN] * JOINT_DIM
        right_firc_dot = [NAN] * JOINT_DIM

        t_fb_s = NAN
        fb_age_s = NAN
        feedback_stale = True
    else:
        left_out = _safe_arm_output(latest_sub_data, ARM_LEFT_INDEX)
        right_out = _safe_arm_output(latest_sub_data, ARM_RIGHT_INDEX)

        left_q_fb = _safe_joint_vec7(left_out.get("fb_joint_pos"))
        left_s_to_q = _safe_joint_vec7(left_out.get("fb_joint_sToq"))
        left_firc_dot = _safe_joint_vec7(left_out.get("est_joint_firc_dot"))

        right_q_fb = _safe_joint_vec7(right_out.get("fb_joint_pos"))
        right_s_to_q = _safe_joint_vec7(right_out.get("fb_joint_sToq"))
        right_firc_dot = _safe_joint_vec7(right_out.get("est_joint_firc_dot"))

        fb_age_s = float(send_ts - float(latest_feedback_perf_s))
        t_fb_s = float(float(latest_feedback_perf_s) - start_perf_s)

        outputs_ok = bool(left_out) and bool(right_out)
        feedback_stale = (fb_age_s < 0.0) or (fb_age_s > stale_threshold_s) or (not outputs_ok)

    left_e_q = [left_q_fb[i] - left_q_cmd[i] for i in range(JOINT_DIM)]
    right_e_q = [right_q_fb[i] - right_q_cmd[i] for i in range(JOINT_DIM)]

    sample = {
        "t_cmd_s": float(cmd_t),
        "send_ts_perf_s": float(send_ts),
        "t_fb_s": float(t_fb_s),
        "fb_age_s": float(fb_age_s),
        "feedback_seq": int(feedback_seq),
        "feedback_stale": bool(feedback_stale),
        "left_q_cmd_deg": left_q_cmd,
        "left_q_fb_deg": left_q_fb,
        "left_e_q_deg": left_e_q,
        "left_fb_joint_sToq": left_s_to_q,
        "left_est_joint_firc_dot": left_firc_dot,
        "right_q_cmd_deg": right_q_cmd,
        "right_q_fb_deg": right_q_fb,
        "right_e_q_deg": right_e_q,
        "right_fb_joint_sToq": right_s_to_q,
        "right_est_joint_firc_dot": right_firc_dot,
    }

    collector["samples"].append(sample)


def finalize_feedback_collector(collector: Dict[str, Any]) -> Dict[str, Any]:
    """停止后台线程并整理结果。"""
    join_timeout = False

    if str(collector.get("collection_mode", "")).lower() == "async":
        stop_event = collector.get("stop_event")
        thread = collector.get("thread")

        if stop_event is not None:
            stop_event.set()

        if thread is not None and hasattr(thread, "join"):
            thread.join(timeout=2.0)
            if thread.is_alive():
                join_timeout = True

    samples = collector.get("samples", [])
    if not samples:
        raise RuntimeError("未采集到任何反馈样本，请先调用 append_feedback_sample")

    def _stack_float_1d(key: str) -> np.ndarray:
        return np.asarray([s[key] for s in samples], dtype=np.float64)

    def _stack_bool_1d(key: str) -> np.ndarray:
        return np.asarray([s[key] for s in samples], dtype=bool)

    def _stack_i64_1d(key: str) -> np.ndarray:
        return np.asarray([s[key] for s in samples], dtype=np.int64)

    def _stack_float_2d(key: str) -> np.ndarray:
        return np.asarray([s[key] for s in samples], dtype=np.float64)

    result = {
        "sample_count": int(len(samples)),
        "t_cmd_s": _stack_float_1d("t_cmd_s"),
        "t_fb_s": _stack_float_1d("t_fb_s"),
        "fb_age_s": _stack_float_1d("fb_age_s"),
        "feedback_seq": _stack_i64_1d("feedback_seq"),
        "feedback_stale": _stack_bool_1d("feedback_stale"),
        "feedback_exception_count": int(collector.get("feedback_exception_count", 0)),
        "collector_join_timeout": bool(join_timeout),
        "left": {
            "q_cmd_deg": _stack_float_2d("left_q_cmd_deg"),
            "q_fb_deg": _stack_float_2d("left_q_fb_deg"),
            "e_q_deg": _stack_float_2d("left_e_q_deg"),
            "fb_joint_sToq": _stack_float_2d("left_fb_joint_sToq"),
            "est_joint_firc_dot": _stack_float_2d("left_est_joint_firc_dot"),
        },
        "right": {
            "q_cmd_deg": _stack_float_2d("right_q_cmd_deg"),
            "q_fb_deg": _stack_float_2d("right_q_fb_deg"),
            "e_q_deg": _stack_float_2d("right_e_q_deg"),
            "fb_joint_sToq": _stack_float_2d("right_fb_joint_sToq"),
            "est_joint_firc_dot": _stack_float_2d("right_est_joint_firc_dot"),
        },
    }
    return result