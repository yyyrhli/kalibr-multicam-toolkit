#!/usr/bin/env python3
"""会话时间戳：frames_*.csv + frame_timestamps.csv + recording_report.json。

对齐原则：
  - 各相机用自己的 unix_ns，不要求同一 frame_id
  - csv 比 mp4 多 1 帧时丢掉末帧（编码器常少写一帧）
  - frame_seq 空洞是采集丢帧，时间戳里已经没有这些帧
"""
from __future__ import annotations

import bisect
import csv
import json
from collections import defaultdict
from pathlib import Path


def find_cam_video(session_dir: Path, cam: int) -> Path:
    hits = sorted(session_dir.glob(f"fisheye{cam}_*.mp4")) + sorted(session_dir.glob(f"cam{cam}_*.mp4"))
    if not hits:
        raise FileNotFoundError(f"{session_dir} 缺少 fisheye{cam}_*.mp4 / cam{cam}_*.mp4")
    return hits[0]


def load_frame_index(session_dir: Path) -> dict[int, list[int]]:
    """cam_id -> unix_ns 列表，按采集顺序。"""
    frames_csv = next(session_dir.glob("frames_*.csv"), None)
    if frames_csv is None:
        raise FileNotFoundError(f"缺少 frames_*.csv: {session_dir}")
    by: dict[int, list[int]] = defaultdict(list)
    with frames_csv.open(encoding="utf-8") as f:
        for line in f:
            if line.startswith("#") or not line.strip() or line.startswith("cam_id"):
                continue
            parts = line.strip().split(",")
            by[int(parts[0])].append(int(parts[2]))
    return dict(by)


def seq_holes_from_timestamps(session_dir: Path) -> dict[str, list[tuple[int, int, int]]]:
    path = session_dir / "frame_timestamps.csv"
    holes: dict[str, list[tuple[int, int, int]]] = defaultdict(list)
    if not path.is_file():
        return {}
    with path.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    by: dict[str, list[int]] = defaultdict(list)
    for row in rows:
        by[row["camera"]].append(int(row["frame_seq"]))
    for cam, seq in by.items():
        for a, b in zip(seq, seq[1:]):
            if b != a + 1:
                holes[cam].append((a, b, b - a))
    return dict(holes)


def encoded_frame_counts(session_dir: Path) -> dict[str, int]:
    report = session_dir / "recording_report.json"
    if not report.is_file():
        return {}
    return json.loads(report.read_text(encoding="utf-8")).get("encoded_frames") or {}


def map_video_timestamps(csv_ns: list[int], video_n: int) -> list[int]:
    if video_n <= 0:
        raise RuntimeError("视频帧数为 0")
    if len(csv_ns) == video_n:
        return list(csv_ns)
    if len(csv_ns) == video_n + 1:
        return list(csv_ns[:video_n])
    if len(csv_ns) == video_n - 1:
        return list(csv_ns)
    raise RuntimeError(f"时间戳 {len(csv_ns)} 与视频帧 {video_n} 无法对齐")


def _median_dt_ns(ts: list[int]) -> int:
    if len(ts) < 2:
        return 33_333_333
    dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1) if ts[i + 1] > ts[i]]
    if not dts:
        return 33_333_333
    dts.sort()
    return int(dts[len(dts) // 2])


def _nearest_idx(ts: list[int], t: int) -> int:
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return 0
    if i >= len(ts):
        return len(ts) - 1
    return i if abs(ts[i] - t) < abs(ts[i - 1] - t) else i - 1


def _largest_synchronized_subset(
    chosen: list[tuple[int, int, int, int]], sync_ns: int
) -> list[tuple[int, int, int, int]]:
    chosen = sorted(chosen, key=lambda item: item[2])
    best: list[tuple[int, int, int, int]] = []
    left = 0
    for right in range(len(chosen)):
        while chosen[right][2] - chosen[left][2] > int(sync_ns):
            left += 1
        window = chosen[left : right + 1]
        if len(window) > len(best):
            best = window
    return best


def pick_common_timeslots(
    timestamps: dict[int, list[int]],
    *,
    stride: int,
    start_frame: int = 0,
    sync_ns: int = 15_000_000,
    min_cams: int = 4,
) -> list[tuple[int, int, int, int]]:
    """按公共 unix 时间槽抽帧，保证同一槽内各路时间差 < sync_ns（供 Kalibr approx-sync）。"""
    cams = sorted(timestamps)
    if not cams:
        return []
    t_lo = []
    t_hi = []
    dt_ns = _median_dt_ns(timestamps[cams[0]])
    for cam in cams:
        ts = timestamps[cam]
        if not ts:
            raise RuntimeError(f"fisheye{cam} 没有时间戳")
        i0 = min(max(int(start_frame), 0), len(ts) - 1)
        t_lo.append(int(ts[i0]))
        t_hi.append(int(ts[-1]))
    t0 = max(t_lo)
    t1 = min(t_hi)
    step = max(int(stride) * dt_ns, dt_ns)
    picked: list[tuple[int, int, int, int]] = []
    used: dict[int, set[int]] = {c: set() for c in cams}
    t = t0
    n_slot = 0
    n_ok = 0
    while t <= t1:
        n_slot += 1
        chosen: list[tuple[int, int, int, int]] = []
        for cam in cams:
            ts = timestamps[cam]
            idx = _nearest_idx(ts, t)
            if abs(int(ts[idx]) - t) <= int(sync_ns) and idx not in used[cam]:
                chosen.append((cam, idx, int(ts[idx]), 0))
        # The Kalibr synchronizer compares image timestamps with each other,
        # not with our ideal slot center. Keep the largest subset whose actual
        # timestamp span satisfies sync_ns; this remains correct across drops.
        chosen = _largest_synchronized_subset(chosen, sync_ns)
        if len(chosen) >= int(min_cams):
            for cam, idx, unix_ns, n_tags in chosen:
                used[cam].add(idx)
                picked.append((cam, idx, unix_ns, n_tags))
            n_ok += 1
        t += step
    picked.sort(key=lambda x: (x[2], x[0]))
    print(
        f"[select] common-slots stride={stride} dt={dt_ns/1e6:.2f}ms "
        f"sync={sync_ns/1e6:.1f}ms min_cams={min_cams} "
        f"slots={n_slot} used={n_ok} picked={len(picked)} "
        f"per_cam={{ {', '.join(f'{c}:{sum(1 for h in picked if h[0]==c)}' for c in cams)} }}",
        flush=True,
    )
    return picked
