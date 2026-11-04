#!/usr/bin/env python3
"""AprilGrid-aware frame selection and dataset coverage validation."""
from __future__ import annotations

import bisect
import concurrent.futures
import json
import math
import multiprocessing
import queue
from dataclasses import asdict, dataclass
from pathlib import Path
from collections.abc import Callable

import cv2
import numpy as np


@dataclass(frozen=True)
class FrameDetection:
    cam: int
    video_index: int
    unix_ns: int
    n_tags: int
    center_x: float
    center_y: float
    board_area_ratio: float
    bbox_width_ratio: float
    bbox_height_ratio: float
    covered_cells: tuple[int, ...]
    board_angle_rad: float = 0.0
    perspective_ratio: float = 1.0
    blur_score: float = 0.0
    border_margin_ratio: float = 1.0


def _dictionary():
    name = "DICT_APRILTAG_36h11"
    if not hasattr(cv2.aruco, name):
        raise RuntimeError("当前 OpenCV 无 DICT_APRILTAG_36h11")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def _detector_params():
    params = (
        cv2.aruco.DetectorParameters_create()
        if hasattr(cv2.aruco, "DetectorParameters_create")
        else cv2.aruco.DetectorParameters()
    )
    # 超广角鱼眼的 tag 边缘会发生明显弯曲，预检参数需要相对宽松。
    for key, value in {
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
        if hasattr(params, key):
            setattr(params, key, value)
    if hasattr(params, "cornerRefinementMethod"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    # The production 6x6 board is printed with a two-cell black tag border.
    # OpenCV defaults to one and then rejects or misdecodes the real board.
    if hasattr(params, "markerBorderBits"):
        params.markerBorderBits = 2
    return params


def detect_frame(
    frame: np.ndarray,
    *,
    cam: int,
    video_index: int,
    unix_ns: int,
    min_tags: int,
    min_board_area_ratio: float,
    grid_cols: int,
    grid_rows: int,
    min_blur_score: float = 0.0,
    min_border_margin_ratio: float = 0.0,
    min_perspective_ratio: float = 0.0,
    dictionary=None,
    detector_params=None,
    detector=None,
) -> FrameDetection | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    dictionary = dictionary or _dictionary()
    params = detector_params or _detector_params()
    if detector is not None:
        corners, ids, _ = detector.detectMarkers(gray)
    elif hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None or len(ids) == 0:
        return None
    marker_ids = ids.reshape(-1)
    unique_ids = np.unique(marker_ids)
    # This detector is only a permissive frame pre-selector.  With a 195-degree
    # fisheye, OpenCV occasionally decodes a distorted candidate as an ID that
    # already exists in the same image.  Rejecting the complete frame here
    # discarded valid views even though Kalibr's own AprilGrid detector accepted
    # them.  Count unique IDs for the minimum-tag gate and leave authoritative
    # duplicate-ID validation to Kalibr.
    n_tags = int(unique_ids.size)
    if n_tags < int(min_tags):
        return None
    points = np.concatenate([np.asarray(corner).reshape(-1, 2) for corner in corners], axis=0)
    height, width = gray.shape[:2]
    hull = cv2.convexHull(points.astype(np.float32))
    area_ratio = float(cv2.contourArea(hull) / max(1.0, float(width * height)))
    if area_ratio < float(min_board_area_ratio):
        return None
    x0, y0 = points.min(axis=0)
    x1, y1 = points.max(axis=0)
    border_margin_ratio = float(
        min(x0 / width, y0 / height, (width - x1) / width, (height - y1) / height)
    )
    if border_margin_ratio < float(min_border_margin_ratio):
        return None
    roi_x0 = max(0, int(math.floor(x0)))
    roi_y0 = max(0, int(math.floor(y0)))
    roi_x1 = min(width, int(math.ceil(x1)) + 1)
    roi_y1 = min(height, int(math.ceil(y1)) + 1)
    roi = gray[roi_y0:roi_y1, roi_x0:roi_x1]
    if roi.size and max(roi.shape[:2]) > 320:
        scale = 320.0 / max(roi.shape[:2])
        roi = cv2.resize(
            roi,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_AREA,
        )
    blur_score = (
        float(cv2.Laplacian(roi, cv2.CV_32F).var()) if roi.size else 0.0
    )
    if blur_score < float(min_blur_score):
        return None
    center = points.mean(axis=0)
    normalized = np.column_stack((points[:, 0] / width, points[:, 1] / height))
    normalized = np.vstack((normalized, center / np.array([width, height], dtype=float)))
    cells = {
        min(grid_rows - 1, max(0, int(y * grid_rows))) * grid_cols
        + min(grid_cols - 1, max(0, int(x * grid_cols)))
        for x, y in normalized
    }
    # Use tag IDs to fit a local board-grid -> normalized-image affine map. Its
    # first axis gives in-plane rotation; its singular-value ratio is a stable,
    # inexpensive proxy for viewpoint foreshortening. Both help retain rotational
    # excitation that center/area-only selection would miss.
    board_points = []
    image_points = []
    for marker_corners, marker_id in zip(corners, ids.reshape(-1)):
        marker_id = int(marker_id)
        if not 0 <= marker_id < 36:
            continue
        raw_col = marker_id % 6
        board_points.append((5 - raw_col, marker_id // 6, 1.0))
        marker_center = np.asarray(marker_corners).reshape(-1, 2).mean(axis=0)
        image_points.append((marker_center[0] / width, marker_center[1] / height))
    board_angle_rad = 0.0
    perspective_ratio = 1.0
    if len(board_points) >= 3:
        affine, *_ = np.linalg.lstsq(
            np.asarray(board_points, dtype=float),
            np.asarray(image_points, dtype=float),
            rcond=None,
        )
        linear = affine[:2, :].T
        board_angle_rad = float(math.atan2(linear[1, 0], linear[0, 0]))
        singular = np.linalg.svd(linear, compute_uv=False)
        if singular[0] > 1e-12:
            perspective_ratio = float(singular[-1] / singular[0])
    if perspective_ratio < float(min_perspective_ratio):
        return None
    return FrameDetection(
        cam=cam,
        video_index=video_index,
        unix_ns=unix_ns,
        n_tags=n_tags,
        center_x=float(center[0] / width),
        center_y=float(center[1] / height),
        board_area_ratio=area_ratio,
        bbox_width_ratio=float((x1 - x0) / width),
        bbox_height_ratio=float((y1 - y0) / height),
        covered_cells=tuple(sorted(cells)),
        board_angle_rad=board_angle_rad,
        perspective_ratio=perspective_ratio,
        blur_score=blur_score,
        border_margin_ratio=border_margin_ratio,
    )


def scan_candidates(
    cam: int,
    video: Path,
    timestamps: list[int],
    candidate_indices: list[int],
    *,
    min_tags: int,
    min_board_area_ratio: float,
    grid_cols: int,
    grid_rows: int,
    min_blur_score: float = 0.0,
    min_border_margin_ratio: float = 0.0,
    min_perspective_ratio: float = 0.0,
    progress_queue=None,
    progress_batch: int = 5,
) -> list[FrameDetection]:
    wanted = set(candidate_indices)
    if not wanted:
        return []
    cv2.setNumThreads(1)
    capture = cv2.VideoCapture(str(video))
    dictionary = _dictionary()
    params = _detector_params()
    detector = (
        cv2.aruco.ArucoDetector(dictionary, params)
        if hasattr(cv2.aruco, "ArucoDetector")
        else None
    )
    detections: list[FrameDetection] = []
    unreported_candidates = 0
    try:
        # One process owns one camera and decodes it from the beginning in a
        # single forward pass.  Do not split a VFR MP4 into independently sought
        # ranges: OpenCV/FFmpeg frame-number seeking can land at a different
        # encoded frame near a concat boundary or the tail of the stream.
        for index in range(max(wanted) + 1):
            if not capture.grab():
                raise RuntimeError(f"{video}: 无法解码候选帧 {index}")
            if index not in wanted:
                continue
            ok, frame = capture.retrieve()
            if not ok or frame is None:
                raise RuntimeError(f"{video}: 无法读取候选帧 {index}")
            detection = detect_frame(
                frame,
                cam=cam,
                video_index=index,
                unix_ns=int(timestamps[index]),
                min_tags=min_tags,
                min_board_area_ratio=min_board_area_ratio,
                grid_cols=grid_cols,
                grid_rows=grid_rows,
                min_blur_score=min_blur_score,
                min_border_margin_ratio=min_border_margin_ratio,
                min_perspective_ratio=min_perspective_ratio,
                dictionary=dictionary,
                detector_params=params,
                detector=detector,
            )
            if detection is not None:
                detections.append(detection)
            unreported_candidates += 1
            if progress_queue is not None and unreported_candidates >= progress_batch:
                progress_queue.put(unreported_candidates)
                unreported_candidates = 0
    finally:
        if progress_queue is not None and unreported_candidates:
            progress_queue.put(unreported_candidates)
        capture.release()
    print(
        f"[detect] fisheye{cam} range={min(wanted)}-{max(wanted)} "
        f"candidates={len(wanted)} valid={len(detections)}",
        flush=True,
    )
    return detections


def scan_all_candidates(
    videos: dict[int, Path],
    timestamps: dict[int, list[int]],
    candidates: list[tuple[int, int, int, int]],
    *,
    min_tags: int,
    min_board_area_ratio: float,
    grid_cols: int,
    grid_rows: int,
    min_blur_score: float = 0.0,
    min_border_margin_ratio: float = 0.0,
    min_perspective_ratio: float = 0.0,
    workers: int | None = None,
    progress_callback: Callable[[int, int], None] | None = None,
) -> list[FrameDetection]:
    indices = {
        cam: sorted({index for item_cam, index, _time, _tags in candidates if item_cam == cam})
        for cam in sorted(videos)
    }
    # A task is always a complete camera stream. ``workers`` only limits how
    # many cameras run concurrently; it never creates multiple readers for one
    # video.  Four-camera datasets therefore default to four forward decoders.
    worker_count = min(max(1, int(workers or len(videos))), max(1, len(videos)))
    tasks = [(cam, indices[cam]) for cam in sorted(videos) if indices[cam]]

    detections: list[FrameDetection] = []
    completed_candidates = 0
    total_candidates = sum(len(camera_indices) for _cam, camera_indices in tasks)
    if progress_callback is not None:
        progress_callback(0, total_candidates)
    with multiprocessing.Manager() as manager:
        candidate_progress = manager.Queue()
        with concurrent.futures.ProcessPoolExecutor(
            max_workers=min(worker_count, max(1, len(tasks)))
        ) as executor:
            futures = {
                executor.submit(
                    scan_candidates,
                    cam,
                    videos[cam],
                    timestamps[cam],
                    camera_indices,
                    min_tags=min_tags,
                    min_board_area_ratio=min_board_area_ratio,
                    grid_cols=grid_cols,
                    grid_rows=grid_rows,
                    min_blur_score=min_blur_score,
                    min_border_margin_ratio=min_border_margin_ratio,
                    min_perspective_ratio=min_perspective_ratio,
                    progress_queue=candidate_progress,
                ): len(camera_indices)
                for cam, camera_indices in tasks
            }
            pending = set(futures)
            while pending:
                done, pending = concurrent.futures.wait(
                    pending,
                    timeout=0.25,
                    return_when=concurrent.futures.FIRST_COMPLETED,
                )
                while True:
                    try:
                        completed_candidates += int(candidate_progress.get_nowait())
                    except queue.Empty:
                        break
                if progress_callback is not None:
                    progress_callback(
                        min(completed_candidates, total_candidates), total_candidates
                    )
                for future in done:
                    detections.extend(future.result())
            while True:
                try:
                    completed_candidates += int(candidate_progress.get_nowait())
                except queue.Empty:
                    break
            if progress_callback is not None:
                progress_callback(total_candidates, total_candidates)
    return sorted(detections, key=lambda item: (item.unix_ns, item.cam))


def _nearest_time(times: list[int], value: int) -> int | None:
    if not times:
        return None
    index = bisect.bisect_left(times, value)
    candidates = []
    if index < len(times):
        candidates.append(times[index])
    if index > 0:
        candidates.append(times[index - 1])
    return min(candidates, key=lambda item: abs(item - value))


def _observation_distance(left: FrameDetection, right: FrameDetection) -> float:
    """Distance in normalized image/board geometry, not in frame-number space."""
    left_cells = set(left.covered_cells)
    right_cells = set(right.covered_cells)
    union = left_cells | right_cells
    cell_distance = 1.0 - len(left_cells & right_cells) / len(union) if union else 0.0
    log_scale = abs(
        math.log(max(left.board_area_ratio, 1e-9) / max(right.board_area_ratio, 1e-9))
    )
    angle_delta = abs(
        math.atan2(
            math.sin(left.board_angle_rad - right.board_angle_rad),
            math.cos(left.board_angle_rad - right.board_angle_rad),
        )
    ) / math.pi
    components = (
        left.center_x - right.center_x,
        left.center_y - right.center_y,
        0.5 * log_scale,
        left.bbox_width_ratio - right.bbox_width_ratio,
        left.bbox_height_ratio - right.bbox_height_ratio,
        0.5 * cell_distance,
        0.02 * (left.n_tags - right.n_tags),
        0.5 * angle_delta,
        0.5 * (left.perspective_ratio - right.perspective_ratio),
    )
    return math.sqrt(sum(value * value for value in components))


def _filter_informative_slots(
    records: list[tuple[int, list[tuple[int, int, int, int]], list[int]]],
    by_key: dict[tuple[int, int], FrameDetection],
    *,
    duplicate_distance: float,
    max_gap_ns: int,
    min_frames_per_camera: int,
    min_covisible_frames: int,
) -> tuple[list[tuple[int, list[tuple[int, int, int, int]], list[int]]], dict]:
    """Greedily retain novel views while preserving time and camera graph support.

    There is deliberately no fixed frame cap. A slot survives when it contributes
    new normalized board geometry, fills a required per-camera/co-visible count, or
    closes a configured temporal gap. This makes the decision independent of frame
    sequence numbers and robust to dropped frames.
    """
    if not records:
        return [], {"input_slots": 0, "output_slots": 0, "rejected_duplicate_slots": 0}

    kept = []
    selected_by_camera: dict[int, list[FrameDetection]] = {}
    feature_bins_by_camera: dict[int, set[tuple[str, int]]] = {}
    camera_counts: dict[int, int] = {}
    pair_counts: dict[tuple[int, int], int] = {}
    last_time: int | None = None

    for record_index, (slot_time, items, visible) in enumerate(records):
        observations = [
            by_key[(cam, index)]
            for cam, index, _timestamp, _tags in items
            if (cam, index) in by_key
        ]
        forced = record_index == 0 or (
            last_time is not None and slot_time - last_time >= max_gap_ns
        )
        if not forced:
            forced = any(
                camera_counts.get(cam, 0) < min_frames_per_camera for cam in visible
            )
        if not forced and min_covisible_frames > 0:
            forced = any(
                pair_counts.get((left, right), 0) < min_covisible_frames
                for index, left in enumerate(visible)
                for right in visible[index + 1 :]
            )
        observation_bins = {}
        for observation in observations:
            observation_bins[observation.cam] = {
                *(("cell", int(cell)) for cell in observation.covered_cells),
                (
                    "quadrant",
                    int(observation.center_x >= 0.5)
                    + 2 * int(observation.center_y >= 0.5),
                ),
                ("scale", math.floor(math.log(max(observation.board_area_ratio, 1e-9)) / 0.25)),
                ("angle", math.floor((observation.board_angle_rad + math.pi) / (math.pi / 6.0))),
                ("perspective", math.floor(observation.perspective_ratio / 0.1)),
            }
        # A new spatial/scale/orientation bin is always useful, even when it is
        # visible in only one camera.  Otherwise require corroborating novelty
        # in at least two cameras from the same timestamp slot; one noisy corner
        # estimate must not keep four near-duplicate images.
        adds_coverage = any(
            bins - feature_bins_by_camera.get(cam, set())
            for cam, bins in observation_bins.items()
        )
        distances = []
        for observation in observations:
            priors = selected_by_camera.get(observation.cam, [])
            distances.append(
                math.inf
                if not priors
                else min(_observation_distance(observation, prior) for prior in priors)
            )
        required_novel_cameras = min(2, len(distances))
        novel = (
            len(
                [distance for distance in distances if distance >= duplicate_distance]
            )
            >= required_novel_cameras
        )
        if not forced and not adds_coverage and not novel:
            continue
        kept.append((slot_time, items, visible))
        last_time = slot_time
        for observation in observations:
            selected_by_camera.setdefault(observation.cam, []).append(observation)
            feature_bins_by_camera.setdefault(observation.cam, set()).update(
                observation_bins[observation.cam]
            )
            camera_counts[observation.cam] = camera_counts.get(observation.cam, 0) + 1
        for index, left in enumerate(visible):
            for right in visible[index + 1 :]:
                pair_counts[(left, right)] = pair_counts.get((left, right), 0) + 1

    # The end of the acquisition is a useful temporal anchor. Keep it even if its
    # board geometry resembles the preceding slot.
    if kept[-1][0] != records[-1][0]:
        kept.append(records[-1])
    return kept, {
        "input_slots": len(records),
        "output_slots": len(kept),
        "rejected_duplicate_slots": len(records) - len(kept),
        "duplicate_distance": duplicate_distance,
        "max_gap_seconds": max_gap_ns / 1e9,
        "fixed_frame_cap": None,
    }


def select_and_validate(
    candidates: list[tuple[int, int, int, int]],
    detections: list[FrameDetection],
    *,
    camera_ids: list[int],
    sync_ns: int,
    min_detected_cameras_per_slot: int,
    min_frames_per_camera: int,
    grid_cols: int,
    grid_rows: int,
    min_coverage_ratio: float,
    min_center_quadrants: int,
    min_scale_ratio: float,
    min_covisible_frames: int,
    information_filter: bool = False,
    duplicate_distance: float = 0.06,
    max_gap_ns: int = 1_000_000_000,
) -> tuple[list[tuple[int, int, int, int]], dict]:
    by_key = {(item.cam, item.video_index): item for item in detections}
    reference_camera = camera_ids[0]
    reference_times = sorted(
        timestamp for cam, _index, timestamp, _tags in candidates if cam == reference_camera
    )
    slots: dict[int, list[tuple[int, int, int, int]]] = {time: [] for time in reference_times}
    for candidate in candidates:
        nearest = _nearest_time(reference_times, candidate[2])
        if nearest is not None and abs(nearest - candidate[2]) <= sync_ns:
            slots[nearest].append(candidate)

    records = []
    for slot_time, items in sorted(slots.items()):
        visible = sorted(
            cam for cam, index, _timestamp, _tags in items if (cam, index) in by_key
        )
        if len(visible) < min_detected_cameras_per_slot:
            continue
        records.append((slot_time, items, visible))

    filter_report = {
        "input_slots": len(records),
        "output_slots": len(records),
        "rejected_duplicate_slots": 0,
        "duplicate_distance": duplicate_distance,
        "max_gap_seconds": max_gap_ns / 1e9,
        "fixed_frame_cap": None,
    }
    if information_filter:
        records, filter_report = _filter_informative_slots(
            records,
            by_key,
            duplicate_distance=duplicate_distance,
            max_gap_ns=max_gap_ns,
            min_frames_per_camera=min_frames_per_camera,
            min_covisible_frames=min_covisible_frames,
        )

    selected: list[tuple[int, int, int, int]] = []
    slot_visibility: list[list[int]] = []
    for _time, items, visible in records:
        slot_visibility.append(visible)
        for cam, index, timestamp, _tags in items:
            detection = by_key.get((cam, index))
            # A valid time slot may contain cameras that did not see the board.
            # Keep only images with a successful AprilGrid detection so Kalibr
            # does not spend time decoding frames that cannot add observations.
            if detection is not None:
                selected.append((cam, index, timestamp, detection.n_tags))
    selected.sort(key=lambda item: (item[2], item[0]))
    selected_keys = {(cam, index) for cam, index, _timestamp, _tags in selected}
    selected_detections = [
        item for item in detections if (item.cam, item.video_index) in selected_keys
    ]

    failures: list[str] = []
    camera_reports = {}
    total_cells = grid_cols * grid_rows
    for cam in camera_ids:
        items = [item for item in selected_detections if item.cam == cam]
        cells = sorted({cell for item in items for cell in item.covered_cells})
        quadrants = sorted(
            {
                (0 if item.center_x < 0.5 else 1) + (0 if item.center_y < 0.5 else 2)
                for item in items
            }
        )
        areas = [item.board_area_ratio for item in items]
        scale_ratio = max(areas) / min(areas) if areas and min(areas) > 0 else 0.0
        coverage_ratio = len(cells) / total_cells
        report = {
            "detected_frames": len(items),
            "covered_cells": cells,
            "coverage_ratio": coverage_ratio,
            "center_quadrants": quadrants,
            "board_area_ratio_min": min(areas) if areas else 0.0,
            "board_area_ratio_max": max(areas) if areas else 0.0,
            "scale_ratio": scale_ratio,
            "board_angle_rad_min": min(
                (item.board_angle_rad for item in items), default=0.0
            ),
            "board_angle_rad_max": max(
                (item.board_angle_rad for item in items), default=0.0
            ),
            "perspective_ratio_min": min(
                (item.perspective_ratio for item in items), default=0.0
            ),
            "perspective_ratio_max": max(
                (item.perspective_ratio for item in items), default=0.0
            ),
            "blur_score_min": min(
                (item.blur_score for item in items), default=0.0
            ),
            "blur_score_max": max(
                (item.blur_score for item in items), default=0.0
            ),
            "border_margin_ratio_min": min(
                (item.border_margin_ratio for item in items), default=0.0
            ),
        }
        camera_reports[f"fisheye{cam}"] = report
        if len(items) < min_frames_per_camera:
            failures.append(f"fisheye{cam}: 有效标定板帧 {len(items)} < {min_frames_per_camera}")
        if coverage_ratio < min_coverage_ratio:
            failures.append(
                f"fisheye{cam}: 画面网格覆盖 {coverage_ratio:.1%} < {min_coverage_ratio:.1%}"
            )
        if len(quadrants) < min_center_quadrants:
            failures.append(
                f"fisheye{cam}: 标定板中心仅覆盖 {len(quadrants)} 个象限 < {min_center_quadrants}"
            )
        if scale_ratio < min_scale_ratio:
            failures.append(
                f"fisheye{cam}: 近远尺度比 {scale_ratio:.2f} < {min_scale_ratio:.2f}"
            )

    pair_counts = {}
    adjacency = {cam: set() for cam in camera_ids}
    for left_index, left in enumerate(camera_ids):
        for right in camera_ids[left_index + 1 :]:
            count = sum(left in visible and right in visible for visible in slot_visibility)
            pair_counts[f"{left}-{right}"] = count
            if count >= min_covisible_frames:
                adjacency[left].add(right)
                adjacency[right].add(left)
    reached = set()
    pending = [camera_ids[0]] if camera_ids else []
    while pending:
        cam = pending.pop()
        if cam in reached:
            continue
        reached.add(cam)
        pending.extend(adjacency[cam] - reached)
    graph_connected = reached == set(camera_ids)
    if not graph_connected:
        failures.append(
            f"四相机标定板共见图不连通（每条边至少 {min_covisible_frames} 帧）"
        )

    report = {
        "schema_version": 1,
        "status": "passed" if not failures else "failed",
        "candidate_images": len(candidates),
        "candidate_detections": len(detections),
        "selected_images": len(selected),
        "selected_detections": len(selected_detections),
        "selected_slots": len(slot_visibility),
        "information_filter": {
            "enabled": information_filter,
            **filter_report,
        },
        "thresholds": {
            "min_detected_cameras_per_slot": min_detected_cameras_per_slot,
            "min_frames_per_camera": min_frames_per_camera,
            "grid_cols": grid_cols,
            "grid_rows": grid_rows,
            "min_coverage_ratio": min_coverage_ratio,
            "min_center_quadrants": min_center_quadrants,
            "min_scale_ratio": min_scale_ratio,
            "min_covisible_frames": min_covisible_frames,
        },
        "cameras": camera_reports,
        "covisible_pair_frames": pair_counts,
        "covisibility_graph_connected": graph_connected,
        "failures": failures,
        "detections": [asdict(item) for item in selected_detections],
    }
    return selected, report


def write_report(path: Path, report: dict) -> None:
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
