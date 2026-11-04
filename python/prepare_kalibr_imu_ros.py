#!/usr/bin/env python3
"""把多路鱼眼 + IMU 按 unix_ns 打成 ROS1 bag，供本机 TartanCalib/Kalibr。

推荐用法（标定板已固定、设备先静置再运动）：
  --dump-all-after --start-frame <静置结束后的帧> --stride 10

OpenCV 预筛在鱼眼上容易漏检，Kalibr 自带 AprilGrid 提取更稳。
默认使用原始 IMU 与 unix_ns，不做时间补偿。只有选择 filtered
时才按显式参数补偿滤波延迟。
"""
from __future__ import annotations

import argparse
import concurrent.futures
import heapq
import json
import math
import multiprocessing as mp
import os
import queue
import shutil
import sys
import threading
import time
from collections import defaultdict
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import cv2
import numpy as np
import yaml

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from camchain_camn import fisheye_to_camn  # noqa: E402
from camera_model import KALIBR_CAMERA_MODEL, require_ds_camchain  # noqa: E402
from aprilgrid_selection import (  # noqa: E402
    scan_all_candidates,
    select_and_validate,
    write_report as write_selection_report,
)
from imu_rk002 import load_imu, write_imu_yaml  # noqa: E402
from session_timestamps import (  # noqa: E402
    encoded_frame_counts,
    find_cam_video,
    load_frame_index,
    map_video_timestamps,
    pick_common_timeslots,
    seq_holes_from_timestamps,
)


def kalibr_model_from_camchain(path: Path) -> str:
    require_ds_camchain(path)
    return KALIBR_CAMERA_MODEL


def _april_dictionary():
    name = "DICT_APRILTAG_36h11"
    if not hasattr(cv2.aruco, name):
        raise RuntimeError("当前 OpenCV 无 DICT_APRILTAG_36h11")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def _make_detector_params():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    # 超广角鱼眼上 tag 直边是弯的，四边形拟合容差必须放大
    for k, v in {
        "adaptiveThreshWinSizeMin": 3,
        "adaptiveThreshWinSizeMax": 53,
        "adaptiveThreshWinSizeStep": 4,
        "minMarkerPerimeterRate": 0.008,
        "maxMarkerPerimeterRate": 4.0,
        "polygonalApproxAccuracyRate": 0.12,
        "minCornerDistanceRate": 0.01,
        "minDistanceToBorder": 1,
        "minMarkerDistanceRate": 0.01,
        "maxErroneousBitsInBorderRate": 0.6,
        "errorCorrectionRate": 1.0,
        "perspectiveRemovePixelPerCell": 8,
    }.items():
        if hasattr(params, k):
            setattr(params, k, v)
    if hasattr(params, "cornerRefinementMethod"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if hasattr(params, "markerBorderBits"):
        params.markerBorderBits = 2
    return params


def detect_april_tags(bgr: np.ndarray, dictionary, detector_params, *, min_tags: int) -> int:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dictionary, detector_params)
        _corners, ids, _ = det.detectMarkers(gray)
    else:
        _corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=detector_params)
    if ids is None or len(ids) == 0:
        return 0
    n = int(np.unique(ids.reshape(-1)).size)
    return n if n >= int(min_tags) else 0


def scan_camera(
    cam_id: int,
    video: Path,
    timestamps: list[int],
    stride: int,
    min_tags: int,
    start_idx: int = 0,
) -> list[tuple[int, int, int, int]]:
    cv2.setNumThreads(1)
    cap = cv2.VideoCapture(str(video))
    dictionary = _april_dictionary()
    params = _make_detector_params()
    hits: list[tuple[int, int, int, int]] = []
    idx = 0
    n_ts = len(timestamps)
    while idx < int(start_idx):
        if not cap.grab():
            cap.release()
            print(f"[detect] fisheye{cam_id} failed skip to {start_idx}", flush=True)
            return hits
        idx += 1
    while True:
        ok, fr = cap.read()
        if not ok or fr is None:
            break
        if idx < n_ts and (idx % stride == 0):
            n_tags = detect_april_tags(fr, dictionary, params, min_tags=min_tags)
            if n_tags > 0:
                hits.append((cam_id, idx, int(timestamps[idx]), n_tags))
        idx += 1
        if idx % 400 == 0:
            print(f"[detect] fisheye{cam_id} {idx}/{n_ts} hits={len(hits)}", flush=True)
    cap.release()
    print(f"[detect] fisheye{cam_id} decoded={idx} hits={len(hits)} stride={stride}", flush=True)
    return hits


def pick_timeslots(
    hits: list[tuple[int, int, int, int]],
    *,
    slot_s: float,
    max_slots: int,
) -> list[tuple[int, int, int, int]]:
    if not hits:
        return []
    hits = sorted(hits, key=lambda x: x[2])
    slots: dict[int, list[tuple[int, int, int, int]]] = defaultdict(list)
    slot_ns = int(slot_s * 1e9)
    t0 = hits[0][2]
    for h in hits:
        slots[(h[2] - t0) // max(slot_ns, 1)].append(h)
    keys = sorted(slots)
    if max_slots > 0 and len(keys) > max_slots:
        pick_idx = np.linspace(0, len(keys) - 1, max_slots).round().astype(int)
        keys = [keys[i] for i in sorted(set(pick_idx.tolist()))]
    picked: list[tuple[int, int, int, int]] = []
    for k in keys:
        best: dict[int, tuple[int, int, int, int]] = {}
        for h in slots[k]:
            cam = h[0]
            if cam not in best or h[3] > best[cam][3]:
                best[cam] = h
        picked.extend(best.values())
    picked.sort(key=lambda x: (x[2], x[0]))
    return picked


def write_aprilgrid_yaml(
    path: Path,
    *,
    tag_cols: int,
    tag_rows: int,
    tag_size: float,
    tag_spacing: float,
    min_tags: int = 6,
) -> None:
    path.write_text(
        "\n".join(
            [
                "target_type: 'aprilgrid'",
                f"tagCols: {int(tag_cols)}",
                f"tagRows: {int(tag_rows)}",
                f"tagSize: {float(tag_size):.6f}",
                f"tagSpacing: {float(tag_spacing):.6f}",
                "reverseTagCols: true",
                "blackTagBorder: 2",
                "doSubpixRefinement: true",
                "maxSubpixDisplacement2: 4.0",
                f"minTagsForValidObs: {int(min_tags)}",
                "minBorderDistance: 4.0",
                "",
            ]
        ),
        encoding="utf-8",
    )


_STREAM_DONE = object()


@dataclass(frozen=True)
class _StreamFailure:
    error: BaseException


def _parallel_merge_streams(
    stream_factories: list[Callable[[], Iterable[tuple[int, Any, Any]]]],
    *,
    workers: int,
    queue_depth: int,
) -> Iterator[tuple[int, Any, Any]]:
    """Bounded producer/consumer merge that keeps global timestamp order.

    Factories are partitioned across workers.  A worker merges its assigned
    streams locally, and this consumer merges one head item from each worker.
    This supports 1..N workers without buffering a complete camera stream.
    """
    if not stream_factories:
        return
    if queue_depth <= 0:
        raise ValueError("queue_depth must be greater than zero")
    worker_count = min(max(1, int(workers)), len(stream_factories))
    if worker_count == 1:
        streams = [iter(factory()) for factory in stream_factories]
        yield from heapq.merge(*streams, key=lambda item: item[0])
        return
    groups: list[list[Callable[[], Iterable[tuple[int, Any, Any]]]]] = [
        [] for _ in range(worker_count)
    ]
    for index, factory in enumerate(stream_factories):
        groups[index % worker_count].append(factory)

    queues = [queue.Queue(maxsize=int(queue_depth)) for _ in groups]
    cancelled = threading.Event()

    def put(item_queue: queue.Queue, item: object) -> bool:
        while not cancelled.is_set():
            try:
                item_queue.put(item, timeout=0.1)
                return True
            except queue.Full:
                continue
        return False

    def produce(
        factories: list[Callable[[], Iterable[tuple[int, Any, Any]]]],
        item_queue: queue.Queue,
    ) -> None:
        try:
            streams = [iter(factory()) for factory in factories]
            for event in heapq.merge(*streams, key=lambda item: item[0]):
                if not put(item_queue, event):
                    return
        except BaseException as error:
            put(item_queue, _StreamFailure(error))
            return
        put(item_queue, _STREAM_DONE)

    def next_item(item_queue: queue.Queue):
        item = item_queue.get()
        if isinstance(item, _StreamFailure):
            raise item.error
        return item

    executor = concurrent.futures.ThreadPoolExecutor(
        max_workers=worker_count,
        thread_name_prefix="bag-camera",
    )
    futures = [
        executor.submit(produce, group, item_queue)
        for group, item_queue in zip(groups, queues)
    ]
    try:
        heads: dict[int, tuple[int, Any, Any]] = {}
        for index, item_queue in enumerate(queues):
            item = next_item(item_queue)
            if item is not _STREAM_DONE:
                heads[index] = item
        while heads:
            index = min(heads, key=lambda key: heads[key][0])
            yield heads[index]
            item = next_item(queues[index])
            if item is _STREAM_DONE:
                del heads[index]
            else:
                heads[index] = item
        for future in futures:
            future.result()
    finally:
        cancelled.set()
        executor.shutdown(wait=True, cancel_futures=True)


def write_bag(
    bag_path: Path,
    videos: dict[int, Path],
    timestamps: dict[int, list[int]],
    picked: list[tuple[int, int, int, int]],
    imu_samples,
    imu_topic: str,
    *,
    camera_workers: int = 4,
    queue_depth: int = 2,
) -> int:
    import rosbag
    from genpy import Time
    from sensor_msgs.msg import Image, Imu
    from std_msgs.msg import Header

    if bag_path.exists() and bag_path.is_dir():
        raise RuntimeError(f"ROS1 bag 输出路径不能是目录：{bag_path}")
    bag_path.parent.mkdir(parents=True, exist_ok=True)

    by_cam: dict[int, list[tuple[int, int]]] = defaultdict(list)
    for cam, idx, ts, _n in picked:
        by_cam[cam].append((idx, ts))

    def camera_events(cam: int, items: list[tuple[int, int]]):
        want = {idx: ts for idx, ts in items}
        capture = cv2.VideoCapture(str(videos[cam]))
        try:
            for index in range(max(want) + 1):
                if not capture.grab():
                    raise RuntimeError(f"{videos[cam]}: 无法解码索引 {index}")
                if index not in want:
                    continue
                ok, frame = capture.retrieve()
                if not ok or frame is None:
                    raise RuntimeError(f"{videos[cam]}: 无法读取索引 {index}")
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                stamp_ns = int(want[index])
                msg = Image()
                msg.header = Header()
                msg.header.stamp = Time(
                    int(stamp_ns // 1_000_000_000),
                    int(stamp_ns % 1_000_000_000),
                )
                msg.header.frame_id = f"fisheye{cam}"
                msg.height, msg.width = gray.shape[:2]
                msg.encoding = "mono8"
                msg.is_bigendian = 0
                msg.step = int(msg.width)
                msg.data = gray.tobytes()
                yield stamp_ns, f"/fisheye{cam}/image_raw", msg
        finally:
            capture.release()

    def imu_events():
        for sample in imu_samples:
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = Time(
                int(sample.t_ns // 1_000_000_000),
                int(sample.t_ns % 1_000_000_000),
            )
            msg.header.frame_id = "imu0"
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity.x = float(sample.gyr[0])
            msg.angular_velocity.y = float(sample.gyr[1])
            msg.angular_velocity.z = float(sample.gyr[2])
            msg.linear_acceleration.x = float(sample.acc[0])
            msg.linear_acceleration.y = float(sample.acc[1])
            msg.linear_acceleration.z = float(sample.acc[2])
            yield int(sample.t_ns), imu_topic, msg

    cv2.setNumThreads(1)
    camera_factories = [
        (lambda cam=cam, items=items: camera_events(cam, items))
        for cam, items in sorted(by_cam.items())
    ]
    camera_stream = _parallel_merge_streams(
        camera_factories,
        workers=camera_workers,
        queue_depth=queue_depth,
    )
    event_count = 0
    started = time.perf_counter()
    with rosbag.Bag(str(bag_path), "w") as writer:
        for stamp_ns, topic, msg in heapq.merge(
            camera_stream,
            imu_events(),
            key=lambda item: item[0],
        ):
            message_time = Time(
                int(stamp_ns // 1_000_000_000),
                int(stamp_ns % 1_000_000_000),
            )
            writer.write(topic, msg, message_time)
            event_count += 1
    elapsed_seconds = time.perf_counter() - started
    image_rate = len(picked) / max(elapsed_seconds, 1e-9)
    print(
        f"[bag] {bag_path} format=ros1 events={event_count} images={len(picked)} "
        f"imu={len(imu_samples)} encoding=mono8 streaming=true compressed=false "
        f"camera_workers={min(max(1, camera_workers), len(by_cam))} "
        f"queue_depth={queue_depth} elapsed_s={elapsed_seconds:.3f} "
        f"image_rate_fps={image_rate:.2f}",
        flush=True,
    )
    return event_count


def write_run_script(out_dir: Path, camchain: Path, imu_yaml: Path, target: Path, bag: Path) -> None:
    env = PKG_ROOT / "setup" / "source_kalibr.sh"
    convert_py = PKG_ROOT / "python" / "convert_kalibr_imucam.py"
    camera_convert = PKG_ROOT / "python" / "convert_kalibr_camchain.py"
    script = out_dir / "run_kalibr.sh"
    script.write_text(
        f"""#!/usr/bin/env bash
set -e
# 本机 ROS1 TartanCalib：不要加 --recover-covariance，样条会崩。
source {env}
export MPLBACKEND=Agg
export PYTHONUNBUFFERED=1
cd {out_dir}
xvfb-run -a "$KALIBR_RUNTIME" rosrun kalibr kalibr_calibrate_imu_camera \\
  --bag {bag} \\
  --cams {camchain} \\
  --imu {imu_yaml} \\
  --target {target} \\
  --timeoffset-padding 0.08 \\
  --dont-show-report
"$KALIBR_RUNTIME" python {camera_convert} --export-dir {out_dir} --camchain {out_dir}/calib-camchain-imucam.yaml --results-txt {out_dir}/calib-results-imucam.txt --out-json {out_dir}/kalibr_calibration.json
"$KALIBR_RUNTIME" python {convert_py} --export-dir {out_dir}
""",
        encoding="utf-8",
    )
    script.chmod(0o755)


def main() -> int:
    ap = argparse.ArgumentParser(description="鱼眼+IMU 官方 ROS Kalibr 准备")
    ap.add_argument("--session-dir", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--camchain", type=Path, required=True, help="已标定的 camchain.yaml（fisheyeN 或 camN 均可）")
    imu_calibration_source = ap.add_mutually_exclusive_group()
    imu_calibration_source.add_argument(
        "--still-imu",
        type=Path,
        default=None,
        help="独立静置会话的 imu_*.bin；白噪声用整段（跳过开头 0.5s），bag 仍用 --session-dir 的 IMU",
    )
    imu_calibration_source.add_argument(
        "--accepted-imu-yaml",
        type=Path,
        default=None,
        help="已通过静置阶段的 imu.yaml；直接复用，不再猜测 still attempt",
    )
    ap.add_argument("--stride", type=int, default=10)
    ap.add_argument("--bag-camera-workers", type=int, default=4)
    ap.add_argument("--bag-queue-depth", type=int, default=2)
    ap.add_argument("--frame-selection-workers", type=int, default=4)
    ap.add_argument("--progress-json", type=Path)
    ap.add_argument("--min-tags", type=int, default=12)
    ap.add_argument("--start-frame", type=int, default=0, help="跳过静置段；0 表示从首帧开始")
    ap.add_argument(
        "--dump-all-after",
        action="store_true",
        help="兼容旧参数；现在仍会执行 AprilGrid 检测和覆盖验收",
    )
    ap.add_argument("--sync-ms", type=float, default=15.0)
    ap.add_argument("--min-board-area-ratio", type=float, default=0.005)
    ap.add_argument("--min-blur-score", type=float, default=25.0)
    ap.add_argument("--min-border-margin-ratio", type=float, default=0.0)
    ap.add_argument("--min-perspective-ratio", type=float, default=0.2)
    ap.add_argument("--coverage-grid-cols", type=int, default=4)
    ap.add_argument("--coverage-grid-rows", type=int, default=3)
    ap.add_argument("--min-coverage-ratio", type=float, default=0.6)
    ap.add_argument("--min-center-quadrants", type=int, default=3)
    ap.add_argument("--min-scale-ratio", type=float, default=1.5)
    ap.add_argument("--min-detected-frames-per-camera", type=int, default=20)
    ap.add_argument("--min-detected-cameras-per-slot", type=int, default=1)
    ap.add_argument("--min-covisible-frames", type=int, default=3)
    ap.add_argument("--information-filter", choices=("on", "off"), default="on")
    ap.add_argument("--duplicate-distance", type=float, default=0.10)
    ap.add_argument("--max-selection-gap-seconds", type=float, default=2.0)
    ap.add_argument("--slot-s", type=float, default=0.25)
    ap.add_argument(
        "--imu-signal",
        choices=("raw", "filtered"),
        default="raw",
        help="raw=原始量+unix_ns（无补偿）；filtered=滤波量+显式滤波延迟补偿",
    )
    ap.add_argument(
        "--filtered-time-compensation-ms",
        type=float,
        default=15.0,
        help="仅 imu-signal=filtered 生效；从 unix_ns 中减去的实际滤波延迟",
    )
    ap.add_argument(
        "--use-raw",
        action="store_const",
        const="raw",
        dest="imu_signal",
        help=argparse.SUPPRESS,
    )
    ap.add_argument("--max-slots", type=int, default=420)
    ap.add_argument("--tag-cols", type=int, default=6)
    ap.add_argument("--tag-rows", type=int, default=6)
    ap.add_argument("--tag-size", type=float, default=0.055)
    ap.add_argument("--tag-spacing", type=float, default=0.3)
    args = ap.parse_args()
    if args.stride <= 0:
        ap.error("--stride 必须大于 0")
    if not 1 <= args.bag_camera_workers <= 4:
        ap.error("--bag-camera-workers 必须在 1–4 之间")
    if args.bag_queue_depth <= 0:
        ap.error("--bag-queue-depth 必须大于 0")
    # Backward-compatible with a UI process that loaded the former default 12;
    # scan_all_candidates still caps this to one reader per camera.
    if not 1 <= args.frame_selection_workers <= 24:
        ap.error("--frame-selection-workers 必须在 1–24 之间")
    if args.min_tags < 6:
        ap.error("--min-tags 不能小于 6，避免 DS/DLT 内参初始化收到稀疏边缘帧")
    if args.min_blur_score < 0:
        ap.error("--min-blur-score 不能为负数")
    if not 0 <= args.min_border_margin_ratio < 0.5:
        ap.error("--min-border-margin-ratio 必须在 0–0.5 之间")
    if not 0 <= args.min_perspective_ratio <= 1:
        ap.error("--min-perspective-ratio 必须在 0–1 之间")
    if args.filtered_time_compensation_ms < 0:
        ap.error("--filtered-time-compensation-ms 不能为负数")
    if args.duplicate_distance < 0 or args.max_selection_gap_seconds <= 0:
        ap.error("信息选帧距离必须非负，最大时间缺口必须大于 0")

    session_dir = args.session_dir.resolve()
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[start] session={session_dir} out={out_dir}", flush=True)
    started = time.monotonic()

    def report_progress(phase: str, fraction: float, eta_seconds: int = 0, **extra) -> None:
        if args.progress_json is None:
            return
        path = args.progress_json.resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "phase": phase,
            "fraction": max(0.0, min(1.0, float(fraction))),
            "eta_seconds": max(0, int(eta_seconds)),
            "elapsed_seconds": round(time.monotonic() - started, 3),
            **extra,
        }
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)

    report_progress("对齐多路时间戳与 IMU", 0.02)

    videos = {i: find_cam_video(session_dir, i) for i in range(4)}
    frame_ns = load_frame_index(session_dir)
    holes = seq_holes_from_timestamps(session_dir)
    encoded = encoded_frame_counts(session_dir)
    timestamps: dict[int, list[int]] = {}
    video_n: dict[int, int] = {}
    for cam, video in videos.items():
        n = int(encoded.get(f"fisheye{cam}", 0))
        if n <= 0:
            cap = cv2.VideoCapture(str(video))
            n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            cap.release()
        video_n[cam] = n
        timestamps[cam] = map_video_timestamps(frame_ns[cam], n)
        print(
            f"[align] fisheye{cam} csv={len(frame_ns[cam])} video={n} used={len(timestamps[cam])} "
            f"seq_holes={holes.get(f'fisheye{cam}', [])}",
            flush=True,
        )

    imu_bin = next(session_dir.glob("imu_*.bin"))
    use_raw = args.imu_signal == "raw"
    imu_samples, imu_meta = load_imu(
        imu_bin,
        use_raw=use_raw,
        filtered_time_compensation_ms=args.filtered_time_compensation_ms,
    )
    print(
        f"[imu] valid={imu_meta['n_valid']} rate={imu_meta['rate_hz']:.2f}Hz "
        f"static={imu_meta['static_s']:.2f}s gaps>10ms={imu_meta['dt_gaps_gt_10ms']}",
        flush=True,
    )
    still_meta = None
    if args.still_imu is not None:
        still_path = args.still_imu.resolve()
        _still_samples, still_meta = load_imu(
            still_path,
            static_end_s=None,
            static_start_s=15.0,
            static_tail_s=15.0,
            use_raw=use_raw,
            filtered_time_compensation_ms=args.filtered_time_compensation_ms,
        )
        still_meta = dict(still_meta)
        still_meta["used_for"] = "imu.yaml white-noise"
        print(
            f"[imu-still] {still_path} valid={still_meta['n_valid']} "
            f"static={still_meta['static_s']:.2f}s "
            f"acc_nd={still_meta['acc_noise_density']} gyr_nd={still_meta['gyr_noise_density']}",
            flush=True,
        )

    sync_ns = int(float(args.sync_ms) * 1e6)
    candidates = pick_common_timeslots(
        timestamps,
        stride=int(args.stride),
        start_frame=int(args.start_frame),
        sync_ns=sync_ns,
        min_cams=4,
    )

    detection_started = time.monotonic()

    def detection_progress(completed: int, total: int) -> None:
        fraction = completed / total if total else 1.0
        elapsed = time.monotonic() - detection_started
        eta = int(elapsed / completed * (total - completed)) if completed else 0
        report_progress(
            f"AprilGrid 候选帧检测 {completed}/{total}",
            0.05 + 0.80 * fraction,
            eta,
            completed_candidates=completed,
            total_candidates=total,
        )

    detections = scan_all_candidates(
        videos,
        timestamps,
        candidates,
        min_tags=args.min_tags,
        min_board_area_ratio=args.min_board_area_ratio,
        grid_cols=args.coverage_grid_cols,
        grid_rows=args.coverage_grid_rows,
        min_blur_score=args.min_blur_score,
        min_border_margin_ratio=args.min_border_margin_ratio,
        min_perspective_ratio=args.min_perspective_ratio,
        workers=args.frame_selection_workers,
        progress_callback=detection_progress,
    )
    report_progress("按覆盖、视角和时间去除低信息视图", 0.88)
    picked, coverage_report = select_and_validate(
        candidates,
        detections,
        camera_ids=sorted(videos),
        sync_ns=sync_ns,
        min_detected_cameras_per_slot=args.min_detected_cameras_per_slot,
        min_frames_per_camera=args.min_detected_frames_per_camera,
        grid_cols=args.coverage_grid_cols,
        grid_rows=args.coverage_grid_rows,
        min_coverage_ratio=args.min_coverage_ratio,
        min_center_quadrants=args.min_center_quadrants,
        min_scale_ratio=args.min_scale_ratio,
        min_covisible_frames=args.min_covisible_frames,
        information_filter=args.information_filter == "on",
        duplicate_distance=args.duplicate_distance,
        max_gap_ns=int(args.max_selection_gap_seconds * 1e9),
    )
    coverage_report["quality_thresholds"] = {
        "min_tags": args.min_tags,
        "min_board_area_ratio": args.min_board_area_ratio,
        "min_blur_score": args.min_blur_score,
        "min_border_margin_ratio": args.min_border_margin_ratio,
        "min_perspective_ratio": args.min_perspective_ratio,
    }
    coverage_path = out_dir / "frame_selection_report.json"
    write_selection_report(coverage_path, coverage_report)
    if coverage_report["status"] != "passed":
        raise RuntimeError(
            "AprilGrid 画面覆盖验收失败："
            + "；".join(coverage_report["failures"])
            + f"；详见 {coverage_path}"
        )

    per_cam = {f"fisheye{c}": sum(1 for h in picked if h[0] == c) for c in range(4)}
    print(f"[select] per_cam={per_cam}", flush=True)
    if len(picked) < 40:
        raise RuntimeError(f"有效帧过少: {len(picked)}")

    camchain_dst = out_dir / "camchain.yaml"
    shutil.copy2(args.camchain, camchain_dst)
    # IMU 标定只认 cam0/cam1/... 键
    fisheye_to_camn(camchain_dst, camchain_dst, n_cams=4)

    target = out_dir / "aprilgrid_target.yaml"
    write_aprilgrid_yaml(
        target,
        tag_cols=args.tag_cols,
        tag_rows=args.tag_rows,
        tag_size=args.tag_size,
        tag_spacing=args.tag_spacing,
        min_tags=args.min_tags,
    )
    imu_yaml = out_dir / "imu.yaml"
    if args.accepted_imu_yaml is not None:
        accepted_imu_yaml = args.accepted_imu_yaml.resolve()
        accepted_payload = yaml.safe_load(accepted_imu_yaml.read_text(encoding="utf-8"))
        required = (
            "accelerometer_noise_density",
            "accelerometer_random_walk",
            "gyroscope_noise_density",
            "gyroscope_random_walk",
            "update_rate",
        )
        if not isinstance(accepted_payload, dict):
            raise RuntimeError(f"已验收 IMU YAML 格式错误：{accepted_imu_yaml}")
        try:
            values = [float(accepted_payload[key]) for key in required]
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(
                f"已验收 IMU YAML 缺少有效噪声参数：{accepted_imu_yaml}"
            ) from error
        if any(not math.isfinite(value) or value <= 0 for value in values):
            raise RuntimeError(f"已验收 IMU YAML 含非正有限参数：{accepted_imu_yaml}")
        if str(accepted_payload.get("rostopic", "")) != "/imu0":
            raise RuntimeError("已验收 IMU YAML rostopic 必须是 /imu0")
        shutil.copy2(accepted_imu_yaml, imu_yaml)
        imu_yaml_source = str(accepted_imu_yaml)
    else:
        write_imu_yaml(imu_yaml, still_meta or imu_meta, "/imu0")
        imu_yaml_source = str(args.still_imu.resolve()) if args.still_imu else str(imu_bin)

    (out_dir / "selected_frames.json").write_text(
        json.dumps(
            {
                "picked": [
                    {"cam": h[0], "video_index": h[1], "unix_ns": h[2], "n_tags": h[3]} for h in picked
                ],
                "count": len(picked),
                "per_cam": per_cam,
                "stride": args.stride,
                "slot_s": args.slot_s,
                "coverage_report": str(coverage_path),
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    (out_dir / "align_report.json").write_text(
        json.dumps(
            {
                "session_dir": str(session_dir),
                "video_frames": video_n,
                "csv_frames": {str(k): len(v) for k, v in frame_ns.items()},
                "used_timestamps": {str(k): len(v) for k, v in timestamps.items()},
                "seq_holes": {k: [{"from": a, "to": b, "step": s} for a, b, s in v] for k, v in holes.items()},
                "imu": imu_meta,
                "imu_still": still_meta,
                "time_base": (
                    "unix_ns; IMU uses raw acc/gyr; no time compensation"
                    if use_raw
                    else (
                        "unix_ns; IMU uses filtered acc/gyr; timestamps shifted by "
                        f"-{args.filtered_time_compensation_ms:g}ms; cameras use frames_*.csv unix_ns"
                    )
                ),
                "notes": [
                    "各相机独立时间戳，不要求多路同一 frame_id",
                    "csv 比 mp4 多 1 帧时丢弃末帧时间戳",
                    "seq 空洞为采集丢帧",
                ],
            },
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    bag = out_dir / "calib.bag"
    report_progress("并行解码并写入 mono8 ROS1 联合 bag", 0.93)
    write_bag(
        bag,
        videos,
        timestamps,
        picked,
        imu_samples,
        "/imu0",
        camera_workers=args.bag_camera_workers,
        queue_depth=args.bag_queue_depth,
    )
    report_progress("候选帧、IMU 与 bag 准备完成", 1.0)
    write_run_script(out_dir, camchain_dst, imu_yaml, target, bag)
    (out_dir / "kalibr_meta.json").write_text(
        json.dumps(
            {
                "session_dir": str(session_dir),
                "target_type": "aprilgrid",
                "cams": [0, 1, 2, 3],
                "ref_cam": 0,
                "num_cameras": 4,
                "camera_names": [f"fisheye{i}" for i in range(4)],
                "topics": [f"/fisheye{i}/image_raw" for i in range(4)],
                "imu_topic": "/imu0",
                "image_encoding": "mono8",
                "bag_format": "ros1",
                "bag_write_mode": "timestamp_merged_parallel_decode",
                "bag_camera_workers": args.bag_camera_workers,
                "bag_queue_depth": args.bag_queue_depth,
                "frame_selection_workers": min(
                    args.frame_selection_workers, len(videos)
                ),
                "frame_selection_workers_requested": args.frame_selection_workers,
                "frame_selection_mode": "one_sequential_reader_per_camera",
                "imu_signal": args.imu_signal,
                "filtered_time_compensation_ms": (
                    args.filtered_time_compensation_ms if not use_raw else 0.0
                ),
                "models": [kalibr_model_from_camchain(camchain_dst)] * 4,
                "target": str(target),
                "bag": str(bag),
                "camchain": str(camchain_dst),
                "imu_yaml": str(imu_yaml),
                "imu_yaml_source": imu_yaml_source,
                "num_selected_frames": len(picked),
                "per_cam_visible": per_cam,
                "frame_selection_report": str(coverage_path),
                "information_filter": args.information_filter,
                "duplicate_distance": args.duplicate_distance,
                "max_selection_gap_seconds": args.max_selection_gap_seconds,
                "source_camchain": str(args.camchain),
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"准备完成 → {out_dir}")
    print(f"官方 ROS 标定: bash {out_dir / 'run_kalibr.sh'}")
    return 0


if __name__ == "__main__":
    try:
        mp.set_start_method("fork")
    except RuntimeError:
        pass
    raise SystemExit(main())
