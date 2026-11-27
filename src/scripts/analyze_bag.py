#!/usr/bin/env python3
"""Fast SQLite + zstd analyzer for AIC ROS 2 bags (single-bag & multi-bag table).

Works in two modes seamlessly:
1. Full telemetry mode (RDK X5 / ROS 2 host with rclpy.serialization):
   Queries SQLite directly, skips image payload blobs (timestamps only for FPS),
   decompresses zstd-compressed telemetry messages, and computes lap time,
   speed stats, reverse-brake coast distance, red-avoid lateral accel / swing,
   YOLO digit triggers, and T-junction merge events.
2. Offline metadata mode (e.g. macOS without ROS 2 Python packages):
   Reads run_meta.json, metadata.yaml, and SQLite topic timestamps / rates.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
import ctypes
import ctypes.util
import json
import math
from pathlib import Path
import re
import sqlite3
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"
IMAGE_TOPICS = {"/image_raw", "/line_follower/debug_image"}

_ZSTD_BACKEND_INIT = False
_ZSTD_MODULE = None
_LIBZSTD = None


def _init_zstd() -> None:
    global _ZSTD_BACKEND_INIT, _ZSTD_MODULE, _LIBZSTD
    if _ZSTD_BACKEND_INIT:
        return
    _ZSTD_BACKEND_INIT = True
    try:
        import zstandard  # type: ignore

        _ZSTD_MODULE = zstandard.ZstdDecompressor()
        return
    except ImportError:
        pass

    candidates = [
        ctypes.util.find_library("zstd"),
        "libzstd.so.1",
        "libzstd.so",
        "libzstd.dylib",
        "/opt/homebrew/lib/libzstd.dylib",
        "/usr/local/lib/libzstd.dylib",
    ]
    for cand in candidates:
        if not cand:
            continue
        try:
            lib = ctypes.CDLL(cand)
            lib.ZSTD_getFrameContentSize.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
            lib.ZSTD_getFrameContentSize.restype = ctypes.c_uint64
            lib.ZSTD_decompress.argtypes = [
                ctypes.c_void_p,
                ctypes.c_size_t,
                ctypes.c_void_p,
                ctypes.c_size_t,
            ]
            lib.ZSTD_decompress.restype = ctypes.c_size_t
            lib.ZSTD_isError.argtypes = [ctypes.c_size_t]
            lib.ZSTD_isError.restype = ctypes.c_uint
            _LIBZSTD = lib
            return
        except OSError:
            continue


def decompress_if_zstd(blob: bytes) -> bytes:
    """Decompress a message payload if it begins with the zstd frame magic."""
    if not blob or not blob.startswith(ZSTD_MAGIC):
        return blob
    _init_zstd()
    if _ZSTD_MODULE is not None:
        return _ZSTD_MODULE.decompress(blob)
    if _LIBZSTD is not None:
        src = ctypes.create_string_buffer(blob, len(blob))
        frame_size = int(_LIBZSTD.ZSTD_getFrameContentSize(src, len(blob)))
        # ZSTD_CONTENTSIZE_UNKNOWN = (uint64)-1, ZSTD_CONTENTSIZE_ERROR = (uint64)-2
        cap = frame_size if 0 < frame_size < (64 * 1024 * 1024) else max(len(blob) * 16, 65536)
        for _ in range(5):
            dst = ctypes.create_string_buffer(cap)
            rc = _LIBZSTD.ZSTD_decompress(dst, cap, src, len(blob))
            if not _LIBZSTD.ZSTD_isError(rc):
                return dst.raw[: int(rc)]
            cap *= 2
    raise RuntimeError(
        "检测到 zstd 压缩消息，但未找到 python3-zstandard 或 libzstd 动态库"
    )


def compute_fps_and_max_gap(timestamps_ns: Sequence[int]) -> Dict[str, Any]:
    """Compute message count, duration (s), average FPS, and max inter-frame gap (ms)."""
    n = len(timestamps_ns)
    if n == 0:
        return {"count": 0, "duration_s": 0.0, "fps": 0.0, "max_gap_ms": 0.0}
    if n == 1:
        return {"count": 1, "duration_s": 0.0, "fps": 0.0, "max_gap_ms": 0.0}
    sorted_ts = sorted(timestamps_ns)
    duration_s = max(0.0, (sorted_ts[-1] - sorted_ts[0]) / 1e9)
    max_gap_ns = max(sorted_ts[i] - sorted_ts[i - 1] for i in range(1, n))
    fps = ((n - 1) / duration_s) if duration_s > 1e-6 else 0.0
    return {
        "count": n,
        "duration_s": round(duration_s, 3),
        "fps": round(fps, 2),
        "max_gap_ms": round(max_gap_ns / 1e6, 2),
    }


def group_time_episodes(
    timestamps_s: Sequence[float], max_gap_s: float = 0.35
) -> List[Tuple[int, int]]:
    """Group sorted timestamps into contiguous [start_idx, end_idx] episodes."""
    if not timestamps_s:
        return []
    episodes: List[Tuple[int, int]] = []
    start = 0
    for i in range(1, len(timestamps_s)):
        if (timestamps_s[i] - timestamps_s[i - 1]) > max_gap_s:
            episodes.append((start, i - 1))
            start = i
    episodes.append((start, len(timestamps_s) - 1))
    return episodes


def _nearest_value(times: Sequence[float], values: Sequence[float], target_t: float, default: float = 0.0) -> float:
    if not times:
        return default
    idx = bisect_left(times, target_t)
    if idx == 0:
        return values[0]
    if idx >= len(times):
        return values[-1]
    before_t, after_t = times[idx - 1], times[idx]
    if abs(target_t - before_t) <= abs(after_t - target_t):
        return values[idx - 1]
    return values[idx]


def compute_avoid_metrics(
    avoid_samples: Sequence[Tuple[float, float, float]],
    odom_samples: Sequence[Tuple[float, float]] = (),
    max_gap_s: float = 0.35,
    flip_deadband: float = 0.15,
) -> Dict[str, Any]:
    """Summarize red-avoidance episodes from /cmd_vel_avoid (t, v, w) and /odom (t, v)."""
    if not avoid_samples:
        return {
            "sample_count": 0,
            "episode_count": 0,
            "total_avoid_s": 0.0,
            "max_abs_yaw": 0.0,
            "max_lat_accel": 0.0,
            "yaw_sign_flips": 0,
            "max_yaw_step": 0.0,
        }

    ordered = sorted(avoid_samples, key=lambda item: item[0])
    times = [row[0] for row in ordered]
    odom_ordered = sorted(odom_samples, key=lambda item: item[0])
    odom_times = [row[0] for row in odom_ordered]
    odom_speeds = [abs(row[1]) for row in odom_ordered]

    episodes = group_time_episodes(times, max_gap_s=max_gap_s)
    total_avoid_s = 0.0
    max_abs_yaw = 0.0
    max_lat_accel = 0.0
    yaw_sign_flips = 0
    max_yaw_step = 0.0

    for s_idx, e_idx in episodes:
        total_avoid_s += max(0.0, ordered[e_idx][0] - ordered[s_idx][0])
        prev_sign = 0
        prev_w: Optional[float] = None
        for idx in range(s_idx, e_idx + 1):
            t, lin_x, ang_z = ordered[idx]
            odom_v = _nearest_value(odom_times, odom_speeds, t, default=0.0)
            v_eff = max(abs(lin_x), odom_v)
            abs_w = abs(ang_z)
            max_abs_yaw = max(max_abs_yaw, abs_w)
            max_lat_accel = max(max_lat_accel, v_eff * abs_w)
            if prev_w is not None:
                max_yaw_step = max(max_yaw_step, abs(ang_z - prev_w))
            prev_w = ang_z

            if abs_w >= flip_deadband:
                sign = 1 if ang_z > 0.0 else -1
                if prev_sign != 0 and sign != prev_sign:
                    yaw_sign_flips += 1
                prev_sign = sign

    return {
        "sample_count": len(ordered),
        "episode_count": len(episodes),
        "total_avoid_s": round(total_avoid_s, 3),
        "max_abs_yaw": round(max_abs_yaw, 3),
        "max_lat_accel": round(max_lat_accel, 3),
        "yaw_sign_flips": yaw_sign_flips,
        "max_yaw_step": round(max_yaw_step, 3),
    }


def _normalize_odom_series(
    odom_samples: Sequence[Tuple[float, ...]]
) -> List[Tuple[float, float, float]]:
    """Normalize (t, x, y, v) or (t, dist_m, v) into cumulative distance (t, dist_m, v)."""
    if not odom_samples:
        return []
    ordered = sorted(odom_samples, key=lambda row: row[0])
    if len(ordered[0]) == 3:
        return [(float(r[0]), float(r[1]), float(r[2])) for r in ordered]
    out: List[Tuple[float, float, float]] = []
    cum_dist = 0.0
    prev_x = float(ordered[0][1])
    prev_y = float(ordered[0][2])
    for row in ordered:
        t, x, y, v = float(row[0]), float(row[1]), float(row[2]), float(row[3])
        step = math.hypot(x - prev_x, y - prev_y)
        if v < -0.05:
            cum_dist -= step
        else:
            cum_dist += step
        out.append((t, cum_dist, v))
        prev_x, prev_y = x, y
    return out


def compute_braking_episodes(
    stop_samples: Sequence[Tuple[float, float]],
    odom_samples: Sequence[Tuple[float, ...]],
    max_gap_s: float = 0.35,
    stop_speed_thresh: float = 0.03,
) -> List[Dict[str, Any]]:
    """Analyze yellow-line stop / reverse-brake episodes.

    Args:
        stop_samples: (t, linear_x) published on /cmd_vel_stop.
        odom_samples: (t, x, y, v) or (t, cumulative_dist_m, v) from /odom.
    """
    if not stop_samples:
        return []
    ordered_stops = sorted(stop_samples, key=lambda r: r[0])
    norm_odom = _normalize_odom_series(odom_samples)
    odom_times = [r[0] for r in norm_odom]

    stop_times = [r[0] for r in ordered_stops]
    episodes = group_time_episodes(stop_times, max_gap_s=max_gap_s)
    results: List[Dict[str, Any]] = []

    for s_idx, e_idx in episodes:
        t_start = ordered_stops[s_idx][0]
        t_end = ordered_stops[e_idx][0]
        min_cmd_v = min(ordered_stops[i][1] for i in range(s_idx, e_idx + 1))
        reverse_engaged = min_cmd_v < -0.05

        coast_cm: Optional[float] = None
        time_to_zero_s: Optional[float] = None
        min_odom_v: Optional[float] = None

        if norm_odom:
            start_i = bisect_left(odom_times, t_start)
            if start_i >= len(norm_odom):
                start_i = len(norm_odom) - 1
            start_dist = norm_odom[start_i][1]
            window_end = max(t_end + 0.5, t_start + 1.5)
            max_forward_delta = 0.0
            min_v = norm_odom[start_i][2]
            for j in range(start_i, len(norm_odom)):
                ot, odist, ov = norm_odom[j]
                if ot > window_end:
                    break
                min_v = min(min_v, ov)
                max_forward_delta = max(max_forward_delta, odist - start_dist)
                if time_to_zero_s is None and ov <= stop_speed_thresh and (ot - t_start) >= 0.0:
                    time_to_zero_s = round(ot - t_start, 3)
            coast_cm = round(max_forward_delta * 100.0, 2)
            min_odom_v = round(min_v, 3)

        results.append(
            {
                "start_t": round(t_start, 3),
                "duration_s": round(max(0.0, t_end - t_start), 3),
                "reverse_engaged": reverse_engaged,
                "min_cmd_v": round(min_cmd_v, 3),
                "coast_cm": coast_cm,
                "time_to_zero_s": time_to_zero_s,
                "min_odom_v": min_odom_v,
            }
        )
    return results


def parse_rosout_events(messages: Sequence[Tuple[float, str]]) -> Dict[str, Any]:
    """Extract high-level competition events from /rosout log messages."""
    digit_turns: List[Dict[str, Any]] = []
    yellow_stops: List[float] = []
    tjunction_armed = 0
    tjunction_triggered = 0
    tjunction_rescued = 0
    tjunction_exited = 0
    lost_warnings = 0

    digit_pat = re.compile(
        r"(?:Triggering\s+turn|Pair\s+decision|Triggered\s+turn).*?(?:digit|winner|chosen)\s*[:=]\s*([0-9]+)",
        re.IGNORECASE,
    )
    conf_pat = re.compile(r"conf\s*[:=]\s*([0-9.]+)", re.IGNORECASE)
    share_pat = re.compile(r"share\s*[:=]\s*([0-9.]+)", re.IGNORECASE)

    for ts, msg in messages:
        lower = msg.lower()
        if "yellow" in lower and ("stop" in lower or "brake" in lower):
            yellow_stops.append(round(ts, 3))
        if "t-junction" in lower or "tjunction" in lower:
            if "armed" in lower:
                tjunction_armed += 1
            if "trigger" in lower or "enter" in lower or "merge turn" in lower:
                tjunction_triggered += 1
            if "rescue" in lower:
                tjunction_rescued += 1
            if "exit" in lower or "finish" in lower or "complete" in lower:
                tjunction_exited += 1
        if "state=lost" in lower or "line lost" in lower:
            lost_warnings += 1

        m_digit = digit_pat.search(msg)
        if m_digit:
            m_conf = conf_pat.search(msg)
            m_share = share_pat.search(msg)
            digit_turns.append(
                {
                    "t": round(ts, 3),
                    "digit": int(m_digit.group(1)),
                    "conf": float(m_conf.group(1)) if m_conf else None,
                    "share": float(m_share.group(1)) if m_share else None,
                }
            )

    return {
        "yellow_stop_logs": len(yellow_stops),
        "digit_turns": digit_turns,
        "tjunction_armed": tjunction_armed,
        "tjunction_triggered": tjunction_triggered,
        "tjunction_rescued": tjunction_rescued,
        "tjunction_exited": tjunction_exited,
        "lost_warnings": lost_warnings,
    }


def _load_run_meta(bag_dir: Path) -> Dict[str, Any]:
    meta_path = bag_dir / "run_meta.json"
    if not meta_path.is_file():
        return {}
    try:
        return json.loads(meta_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def _load_metadata_yaml_summary(bag_dir: Path) -> Dict[str, Any]:
    meta_path = bag_dir / "metadata.yaml"
    if not meta_path.is_file():
        return {}
    text = meta_path.read_text(encoding="utf-8", errors="replace")
    out: Dict[str, Any] = {}
    for key in ("compression_format", "compression_mode", "message_count"):
        m = re.search(rf"^\s*{key}:\s*(.+)$", text, re.MULTILINE)
        if m:
            out[key] = m.group(1).strip().strip('"').strip("'")
    m_dur = re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text)
    if m_dur:
        out["duration_s"] = round(int(m_dur.group(1)) / 1e9, 3)
    return out


def _try_get_ros2_deserializer():
    try:
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore

        return deserialize_message, get_message
    except ImportError:
        return None, None


def _yaw_from_quat(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def analyze_bag(bag_dir: Path) -> Dict[str, Any]:
    """Analyze one recorded ROS 2 bag folder and return a structured report dict."""
    bag_dir = bag_dir.resolve()
    if not bag_dir.is_dir():
        raise FileNotFoundError(f"找不到录包目录：{bag_dir}")

    run_meta = _load_run_meta(bag_dir)
    yaml_meta = _load_metadata_yaml_summary(bag_dir)
    db3_files = sorted(bag_dir.glob("*.db3"))
    total_size_mb = round(
        sum(p.stat().st_size for p in db3_files if p.is_file()) / (1024 * 1024), 2
    )

    topic_counts: Dict[str, int] = {}
    image_timestamps_ns: List[int] = []
    debug_image_timestamps_ns: List[int] = []
    global_t_min_ns: Optional[int] = None
    global_t_max_ns: Optional[int] = None

    odom_raw: List[Tuple[int, float, float, float]] = []
    cmd_vel_raw: List[Tuple[int, float, float]] = []
    avoid_raw: List[Tuple[int, float, float]] = []
    stop_raw: List[Tuple[int, float]] = []
    det_cmd_raw: List[Tuple[int, float, float]] = []
    start_cmd_raw: List[Tuple[int, bool]] = []
    track_states: List[Tuple[int, str]] = []
    rosout_raw: List[Tuple[int, str]] = []

    deserialize_message, get_message = _try_get_ros2_deserializer()
    telemetry_available = deserialize_message is not None and get_message is not None
    msg_type_cache: Dict[str, Any] = {}

    wanted_telemetry = {
        "/odom",
        "/cmd_vel",
        "/cmd_vel_avoid",
        "/cmd_vel_stop",
        "/cmd_vel_detection",
        "/line_follower/start_cmd",
        "/line_follower/track_state",
        "/rosout",
    }

    for db3_path in db3_files:
        conn = sqlite3.connect(f"file:{db3_path}?mode=ro", uri=True)
        try:
            cur = conn.cursor()
            cur.execute("SELECT id, name, type FROM topics")
            topics_by_id = {row[0]: (row[1], row[2]) for row in cur.fetchall()}

            cur.execute("SELECT topic_id, COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages GROUP BY topic_id")
            for tid, cnt, tmin, tmax in cur.fetchall():
                tname = topics_by_id.get(tid, (f"topic_{tid}", ""))[0]
                topic_counts[tname] = topic_counts.get(tname, 0) + int(cnt)
                if tmin is not None:
                    global_t_min_ns = int(tmin) if global_t_min_ns is None else min(global_t_min_ns, int(tmin))
                if tmax is not None:
                    global_t_max_ns = int(tmax) if global_t_max_ns is None else max(global_t_max_ns, int(tmax))

            # Image streams: timestamps only (never pull megabytes of image blobs)
            for tid, (tname, _) in topics_by_id.items():
                if tname == "/image_raw":
                    cur.execute("SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
                    image_timestamps_ns.extend(int(r[0]) for r in cur.fetchall())
                elif tname == "/line_follower/debug_image":
                    cur.execute("SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
                    debug_image_timestamps_ns.extend(int(r[0]) for r in cur.fetchall())

            if telemetry_available and deserialize_message is not None and get_message is not None:
                selected_ids = [
                    tid for tid, (tname, _) in topics_by_id.items() if tname in wanted_telemetry
                ]
                if selected_ids:
                    placeholders = ",".join("?" for _ in selected_ids)
                    cur.execute(
                        f"SELECT topic_id, timestamp, data FROM messages "
                        f"WHERE topic_id IN ({placeholders}) ORDER BY timestamp",
                        selected_ids,
                    )
                    for tid, ts_ns, blob in cur:
                        tname, ttype = topics_by_id[tid]
                        if ttype not in msg_type_cache:
                            msg_type_cache[ttype] = get_message(ttype)
                        raw_bytes = decompress_if_zstd(bytes(blob))
                        msg = deserialize_message(raw_bytes, msg_type_cache[ttype])
                        ts = int(ts_ns)
                        if tname == "/odom":
                            odom_raw.append(
                                (
                                    ts,
                                    float(msg.pose.pose.position.x),
                                    float(msg.pose.pose.position.y),
                                    float(msg.twist.twist.linear.x),
                                )
                            )
                        elif tname == "/cmd_vel":
                            cmd_vel_raw.append((ts, float(msg.linear.x), float(msg.angular.z)))
                        elif tname == "/cmd_vel_avoid":
                            avoid_raw.append((ts, float(msg.linear.x), float(msg.angular.z)))
                        elif tname == "/cmd_vel_stop":
                            stop_raw.append((ts, float(msg.linear.x)))
                        elif tname == "/cmd_vel_detection":
                            det_cmd_raw.append((ts, float(msg.linear.x), float(msg.angular.z)))
                        elif tname == "/line_follower/start_cmd":
                            start_cmd_raw.append((ts, bool(msg.data)))
                        elif tname == "/line_follower/track_state":
                            track_states.append((ts, str(msg.data)))
                        elif tname == "/rosout":
                            rosout_raw.append((ts, str(msg.msg)))
        finally:
            conn.close()

    t0_ns = global_t_min_ns or 0
    duration_s = (
        round((global_t_max_ns - global_t_min_ns) / 1e9, 2)
        if (global_t_min_ns is not None and global_t_max_ns is not None)
        else float(yaml_meta.get("duration_s", run_meta.get("duration_sec", 0.0)))
    )

    rel_odom = [((ts - t0_ns) / 1e9, x, y, v) for ts, x, y, v in odom_raw]
    rel_odom_speed = [((ts - t0_ns) / 1e9, v) for ts, _, _, v in odom_raw]
    rel_avoid = [((ts - t0_ns) / 1e9, v, w) for ts, v, w in avoid_raw]
    rel_stop = [((ts - t0_ns) / 1e9, v) for ts, v in stop_raw]
    rel_rosout = [((ts - t0_ns) / 1e9, m) for ts, m in rosout_raw]

    norm_odom = _normalize_odom_series(rel_odom)
    total_odom_dist_m = round(norm_odom[-1][1], 3) if norm_odom else None
    moving_speeds = [abs(v) for _, _, v in norm_odom if abs(v) > 0.05]
    mean_moving_speed = (
        round(sum(moving_speeds) / len(moving_speeds), 3) if moving_speeds else None
    )
    peak_speed = round(max((abs(v) for _, _, v in norm_odom), default=0.0), 3) if norm_odom else None

    active_run_s: Optional[float] = None
    true_times = [(ts - t0_ns) / 1e9 for ts, flag in start_cmd_raw if flag]
    if true_times:
        false_times = [(ts - t0_ns) / 1e9 for ts, flag in start_cmd_raw if not flag and ((ts - t0_ns) / 1e9) > true_times[0]]
        end_run_t = false_times[-1] if false_times else (duration_s or true_times[-1])
        active_run_s = round(max(0.0, end_run_t - true_times[0]), 2)

    lost_frames = sum(1 for _, state in track_states if "LOST" in state.upper())
    avoid_metrics = compute_avoid_metrics(rel_avoid, rel_odom_speed)
    braking_episodes = compute_braking_episodes(rel_stop, rel_odom)
    rosout_events = parse_rosout_events(rel_rosout)

    return {
        "bag_name": bag_dir.name,
        "bag_path": str(bag_dir),
        "mode": "full_ros2" if telemetry_available else "offline_metadata",
        "meta": run_meta,
        "yaml_meta": yaml_meta,
        "size_mb": total_size_mb,
        "duration_s": duration_s,
        "active_run_s": active_run_s,
        "topic_counts": topic_counts,
        "camera_fps": compute_fps_and_max_gap(image_timestamps_ns),
        "debug_image_fps": compute_fps_and_max_gap(debug_image_timestamps_ns),
        "odom": {
            "dist_m": total_odom_dist_m,
            "mean_moving_speed": mean_moving_speed,
            "peak_speed": peak_speed,
        },
        "track_state": {
            "total_frames": len(track_states),
            "lost_frames": lost_frames,
        },
        "avoid": avoid_metrics,
        "braking": braking_episodes,
        "events": rosout_events,
    }


def discover_bags(inputs: Sequence[Path]) -> List[Path]:
    """Resolve explicit bag dirs or parent directories containing recorded bags."""
    bags: List[Path] = []
    seen = set()
    for p in inputs:
        if not p.exists():
            continue
        if p.is_dir() and (list(p.glob("*.db3")) or (p / "metadata.yaml").is_file()):
            rp = p.resolve()
            if rp not in seen:
                seen.add(rp)
                bags.append(rp)
        elif p.is_dir():
            for child in sorted(p.iterdir()):
                if child.is_dir() and (
                    list(child.glob("*.db3")) or (child / "metadata.yaml").is_file()
                ):
                    rc = child.resolve()
                    if rc not in seen:
                        seen.add(rc)
                        bags.append(rc)
    return bags


def format_single_summary(report: Dict[str, Any]) -> str:
    meta = report.get("meta", {})
    cam = report["camera_fps"]
    odom = report["odom"]
    avoid = report["avoid"]
    braking = report["braking"]
    events = report["events"]

    lines = [
        f"=== 录包分析：{report['bag_name']} ({report['size_mb']} MB, {report['duration_s']} s, mode={report['mode']}) ===",
    ]
    if meta:
        lines.append(
            f"元数据：map={meta.get('map') or '-'} | tag={meta.get('tag') or '-'} | "
            f"params={meta.get('params_basename') or '-'} | "
            f"model={meta.get('yolo_model_basename') or '-'} "
            f"(md5:{(meta.get('yolo_model_md5') or '')[:8] or '-'})"
        )
        if meta.get("note"):
            lines.append(f"备注：{meta['note']}")

    lines.append(
        f"相机 /image_raw：{cam['count']} 帧 | {cam['fps']} FPS | max_gap={cam['max_gap_ms']} ms"
    )
    if report["mode"] == "full_ros2":
        coast_list = [f"{b['coast_cm']}cm" for b in braking if b.get("coast_cm") is not None]
        digits = [str(d["digit"]) for d in events["digit_turns"]]
        lines.append(
            f"运动学：active={report['active_run_s'] or '-'} s | dist={odom['dist_m'] or '-'} m | "
            f"v_mean={odom['mean_moving_speed'] or '-'} m/s | v_peak={odom['peak_speed'] or '-'} m/s | "
            f"LOST={report['track_state']['lost_frames']} 帧"
        )
        lines.append(
            f"红障避让：{avoid['episode_count']} 次 ({avoid['total_avoid_s']} s) | "
            f"max|w|={avoid['max_abs_yaw']} rad/s | max_lat_a={avoid['max_lat_accel']} m/s^2 | "
            f"yaw正负翻转={avoid['yaw_sign_flips']}"
        )
        lines.append(
            f"黄线制动：{len(braking)} 次 | coast=[{', '.join(coast_list) if coast_list else '-'}]"
        )
        lines.append(
            f"数字与丁字口：digits=[{','.join(digits) if digits else '-'}] | "
            f"T-merge arm/trig/rescue/exit="
            f"{events['tjunction_armed']}/{events['tjunction_triggered']}/"
            f"{events['tjunction_rescued']}/{events['tjunction_exited']}"
        )
    else:
        lines.append("（当前环境无 rclpy：仅输出元数据与 SQLite 话题/帧率统计；车端运行可看全量运动学指标）")
        top_counts = sorted(report["topic_counts"].items(), key=lambda kv: kv[0])
        if top_counts:
            lines.append("话题计数：" + ", ".join(f"{k}={v}" for k, v in top_counts))
    return "\n".join(lines)


def format_comparison_markdown(reports: Sequence[Dict[str, Any]]) -> str:
    """Format a Markdown comparison table across multiple bags."""
    headers = [
        "Bag",
        "Map",
        "Params",
        "Model",
        "Dur(s)",
        "FPS",
        "MaxGap(ms)",
        "MeanV",
        "LOST",
        "Avoid# / MaxLatA / Flips",
        "MaxCoast(cm)",
        "Digits",
        "T-Merge",
    ]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for rep in reports:
        meta = rep.get("meta", {})
        cam = rep["camera_fps"]
        odom = rep["odom"]
        avoid = rep["avoid"]
        coasts = [b["coast_cm"] for b in rep["braking"] if b.get("coast_cm") is not None]
        max_coast = f"{max(coasts):.1f}" if coasts else "-"
        digits = ",".join(str(d["digit"]) for d in rep["events"]["digit_turns"]) or "-"
        ev = rep["events"]
        t_merge = f"{ev['tjunction_triggered']}/{ev['tjunction_rescued']}/{ev['tjunction_exited']}"
        avoid_str = (
            f"{avoid['episode_count']} / {avoid['max_lat_accel']} / {avoid['yaw_sign_flips']}"
            if rep["mode"] == "full_ros2"
            else "-"
        )
        row = [
            rep["bag_name"],
            meta.get("map") or "-",
            meta.get("params_basename") or "-",
            meta.get("yolo_model_basename") or "-",
            f"{rep['duration_s']}",
            f"{cam['fps']}",
            f"{cam['max_gap_ms']}",
            f"{odom['mean_moving_speed']}" if odom["mean_moving_speed"] is not None else "-",
            str(rep["track_state"]["lost_frames"]) if rep["mode"] == "full_ros2" else "-",
            avoid_str,
            max_coast,
            digits,
            t_merge if rep["mode"] == "full_ros2" else "-",
        ]
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines) + "\n"


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Analyze ROS 2 bags recorded with record_bag.sh (single run or multi-bag comparison)."
    )
    parser.add_argument(
        "paths",
        nargs="+",
        help="One or more bag directories or parent folders containing bags.",
    )
    parser.add_argument(
        "--compare",
        action="store_true",
        help="Output multi-bag Markdown comparison table.",
    )
    parser.add_argument("--json", dest="json_out", help="Write JSON report to file.")
    parser.add_argument("--markdown", dest="md_out", help="Write Markdown table to file.")
    args = parser.parse_args(argv)

    bags = discover_bags([Path(p) for p in args.paths])
    if not bags:
        print(f"未找到有效录包目录：{', '.join(args.paths)}", file=sys.stderr)
        return 1

    reports = [analyze_bag(bag) for bag in bags]
    if args.compare or len(reports) > 1:
        md = format_comparison_markdown(reports)
        print(md, end="")
    else:
        print(format_single_summary(reports[0]))

    if args.md_out:
        md = format_comparison_markdown(reports)
        out_p = Path(args.md_out)
        out_p.parent.mkdir(parents=True, exist_ok=True)
        out_p.write_text(md, encoding="utf-8")
        print(f"Markdown 报告已保存：{out_p}")

    if args.json_out:
        out_j = Path(args.json_out)
        out_j.parent.mkdir(parents=True, exist_ok=True)
        payload: Any = reports[0] if len(reports) == 1 and not args.compare else reports
        out_j.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"JSON 报告已保存：{out_j}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
