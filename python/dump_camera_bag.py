#!/usr/bin/env python3
"""按公共时间槽抽帧，检测 AprilGrid 并验收画面覆盖后写 bag。"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from aprilgrid_selection import scan_all_candidates, select_and_validate, write_report  # noqa: E402
from camera_model import KALIBR_CAMERA_MODEL  # noqa: E402
from prepare_kalibr_imu_ros import write_aprilgrid_yaml, write_bag  # noqa: E402
from session_timestamps import (  # noqa: E402
    encoded_frame_counts,
    find_cam_video,
    load_frame_index,
    map_video_timestamps,
    pick_common_timeslots,
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--session-dir", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--stride", type=int, default=10, help="公共时间槽间隔（30fps 时 10≈0.33s）")
    ap.add_argument("--bag-camera-workers", type=int, default=4)
    ap.add_argument("--bag-queue-depth", type=int, default=2)
    ap.add_argument("--frame-selection-workers", type=int, default=4)
    ap.add_argument("--progress-json", type=Path)
    ap.add_argument("--start-frame", type=int, default=0)
    ap.add_argument("--sync-ms", type=float, default=15.0, help="同一时间槽内允许的最大时差，须小于 Kalibr --approx-sync")
    ap.add_argument("--min-cams", type=int, default=4)
    ap.add_argument("--min-tags", type=int, default=12)
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
    args = ap.parse_args()
    if not 1 <= args.bag_camera_workers <= 4:
        ap.error("--bag-camera-workers 必须在 1–4 之间")
    if args.bag_queue_depth <= 0:
        ap.error("--bag-queue-depth 必须大于 0")
    # Accept the former 1–24 CLI range so an already-running UI that loaded the
    # old default (12) can retry safely. scan_all_candidates caps execution to
    # one worker per camera, therefore four cameras still create only 4 readers.
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
    if args.duplicate_distance < 0 or args.max_selection_gap_seconds <= 0:
        ap.error("信息选帧距离必须非负，最大时间缺口必须大于 0")
    session = args.session_dir.resolve()
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)

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
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
        temporary.replace(path)

    report_progress("对齐多路时间戳", 0.02)

    videos = {i: find_cam_video(session, i) for i in range(4)}
    frame_ns = load_frame_index(session)
    encoded = encoded_frame_counts(session)
    timestamps = {}
    for cam, video in videos.items():
        n = int(encoded.get(f"fisheye{cam}", 0))
        if n <= 0:
            import cv2

            cap = cv2.VideoCapture(str(video))
            n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            cap.release()
        timestamps[cam] = map_video_timestamps(frame_ns[cam], n)
        print(f"[align] fisheye{cam} csv={len(frame_ns[cam])} video={n} used={len(timestamps[cam])}", flush=True)

    candidates = pick_common_timeslots(
        timestamps,
        stride=int(args.stride),
        start_frame=int(args.start_frame),
        sync_ns=int(float(args.sync_ms) * 1e6),
        min_cams=int(args.min_cams),
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
    picked, selection_report = select_and_validate(
        candidates,
        detections,
        camera_ids=sorted(videos),
        sync_ns=int(float(args.sync_ms) * 1e6),
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
    selection_report["quality_thresholds"] = {
        "min_tags": args.min_tags,
        "min_board_area_ratio": args.min_board_area_ratio,
        "min_blur_score": args.min_blur_score,
        "min_border_margin_ratio": args.min_border_margin_ratio,
        "min_perspective_ratio": args.min_perspective_ratio,
    }
    report_path = out / "frame_selection_report.json"
    write_report(report_path, selection_report)
    if selection_report["status"] != "passed":
        raise RuntimeError(
            "AprilGrid 画面覆盖验收失败："
            + "；".join(selection_report["failures"])
            + f"；详见 {report_path}"
        )

    models = [KALIBR_CAMERA_MODEL] * 4
    target = out / "aprilgrid_target.yaml"
    write_aprilgrid_yaml(
        target,
        tag_cols=6,
        tag_rows=6,
        tag_size=0.055,
        tag_spacing=0.3,
        min_tags=args.min_tags,
    )
    bag = out / "calib.bag"
    report_progress("并行解码并写入 mono8 ROS1 bag", 0.93)
    write_bag(
        bag,
        videos,
        timestamps,
        picked,
        [],
        "/imu0",
        camera_workers=args.bag_camera_workers,
        queue_depth=args.bag_queue_depth,
    )
    report_progress("候选帧与 bag 准备完成", 1.0)
    (out / "kalibr_meta.json").write_text(
        json.dumps(
            {
                "session_dir": str(session),
                "topics": [f"/fisheye{i}/image_raw" for i in range(4)],
                "models": models,
                "target": str(target),
                "bag": str(bag),
                "num_selected_frames": len(picked),
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
                "frame_selection_report": str(report_path),
                "information_filter": args.information_filter,
                "duplicate_distance": args.duplicate_distance,
                "max_selection_gap_seconds": args.max_selection_gap_seconds,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"准备完成 → {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
