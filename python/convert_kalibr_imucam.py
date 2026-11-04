#!/usr/bin/env python3
"""Kalibr IMU–相机结果 YAML → JSON（T_cam_imu + timeshift）。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from camera_model import KALIBR_CAMERA_MODEL, require_ds_camchain


def _T_to_Rt(T: np.ndarray) -> dict:
    return {
        "R": T[:3, :3].tolist(),
        "t_m": T[:3, 3].reshape(3).tolist(),
        "T_4x4": T.tolist(),
        "description": "p_dst = R @ p_src + t",
    }


def _invert_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def _as_T(value) -> np.ndarray:
    T = np.array(value, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"T 形状不是 4x4: {T.shape}")
    return T


def convert_export_dir(export_dir: Path) -> Path:
    export_dir = export_dir.resolve()
    camchain = export_dir / "calib-camchain-imucam.yaml"
    if not camchain.is_file():
        cands = sorted(export_dir.glob("*-camchain-imucam.yaml"))
        if not cands:
            raise SystemExit(f"未找到 camchain-imucam.yaml: {export_dir}")
        camchain = cands[0]
    raw = require_ds_camchain(camchain)
    meta_path = export_dir / "kalibr_meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}
    names = list(meta.get("camera_names") or [])

    cameras = {}
    items = [(k, v) for k, v in raw.items() if isinstance(v, dict)]
    for idx, (name, cam) in enumerate(items):
        pretty = names[idx] if idx < len(names) else name
        item: dict = {
            "kalibr_key": name,
            "rostopic": cam.get("rostopic"),
            "kalibr_model": KALIBR_CAMERA_MODEL,
            "camera_model": cam.get("camera_model"),
            "distortion_model": cam.get("distortion_model"),
            "intrinsics": cam.get("intrinsics"),
            "resolution": cam.get("resolution"),
        }
        if "T_cam_imu" in cam:
            T_cam_imu = _as_T(cam["T_cam_imu"])
            item["T_cam_from_imu"] = _T_to_Rt(T_cam_imu)
            item["T_imu_from_cam"] = _T_to_Rt(_invert_T(T_cam_imu))
        if "timeshift_cam_imu" in cam:
            item["timeshift_cam_imu_s"] = float(cam["timeshift_cam_imu"])
            item["timeshift_note"] = "t_imu = t_cam + timeshift_cam_imu  (Kalibr convention)"
        cameras[pretty] = item

    results_txt = export_dir / "calib-results-imucam.txt"
    if not results_txt.is_file():
        cands = sorted(export_dir.glob("*-results-imucam.txt"))
        results_txt = cands[0] if cands else None
    results_text = results_txt.read_text(encoding="utf-8") if results_txt and results_txt.is_file() else ""

    out = {
        "description": "ROS1 TartanCalib IMU-camera extrinsics + time offset",
        "calib_backend": "ros1_tartancalib",
        "source_camchain_imucam": str(camchain),
        "source_camchain": meta.get("source_camchain") or meta.get("camchain"),
        "imu_yaml": meta.get("imu_yaml"),
        "bag": meta.get("bag"),
        "coordinate_convention": "p_dst = R @ p_src + t   (T_dst_from_src), units meters",
        "cameras": cameras,
        "results_txt": results_text,
    }
    out_path = export_dir / "kalibr_imu_calibration.json"
    out_path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[convert] {out_path}")
    return out_path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-dir", type=Path, required=True)
    args = ap.parse_args()
    convert_export_dir(args.export_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
