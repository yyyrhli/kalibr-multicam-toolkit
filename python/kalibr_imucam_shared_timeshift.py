#!/usr/bin/env python3
"""Kalibr IMU–相机联合：多路共用一个 timeshift（硬同步），冻结时间只优化 T_cam_imu。

Kalibr 官方每路一个 cameraTimeToImuTimeDv。这里在 buildProblem 里写入同一
timeshiftCamToImuPrior，并强制 noTimeCalibration。

环境变量：
  KALIBR_SHARED_TIMESHIFT_S  t_imu = t_cam + shift（秒）
其余 argv 与 kalibr_calibrate_imu_camera 相同，会自动补 --no-time-calibration。
"""
from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path

import yaml


def _patch_yaml(export_dir: Path, shared: float) -> None:
    hits = list(export_dir.glob("calib-camchain-imucam.yaml")) + list(
        export_dir.glob("*-camchain-imucam.yaml")
    )
    if not hits:
        print("[shared-td] 没有 camchain-imucam.yaml，跳过写回", flush=True)
        return
    path = hits[0]
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    n = 0
    for _k, cam in raw.items():
        if isinstance(cam, dict) and ("rostopic" in cam or "camera_model" in cam):
            cam["timeshift_cam_imu"] = float(shared)
            n += 1
    path.write_text(yaml.safe_dump(raw, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(f"[shared-td] wrote timeshift_cam_imu={shared:.9f} to {n} cams in {path}", flush=True)


def main() -> int:
    shared = os.environ.get("KALIBR_SHARED_TIMESHIFT_S")
    if shared is None:
        raise SystemExit("需要环境变量 KALIBR_SHARED_TIMESHIFT_S")
    shared_f = float(shared)
    kalibr_bin = os.environ.get("KALIBR_CALIBRATE_IMU_CAMERA")
    if not kalibr_bin or not Path(kalibr_bin).is_file():
        raise SystemExit("KALIBR_CALIBRATE_IMU_CAMERA 未设置或不是文件")

    argv = [kalibr_bin] + sys.argv[1:]
    if "--no-time-calibration" not in argv:
        argv.append("--no-time-calibration")
    sys.argv = argv

    from kalibr_imu_camera_calibration.IccCalibrator import IccCalibrator

    orig = IccCalibrator.buildProblem

    def wrapped(self, *args, **kwargs):
        kwargs["noTimeCalibration"] = True
        for i, cam in enumerate(self.CameraChain.camList):
            cam.timeshiftCamToImuPrior = shared_f
            print(f"[shared-td] cam{i} timeshiftCamToImuPrior={shared_f:.9f} s", flush=True)
        return orig(self, *args, **kwargs)

    IccCalibrator.buildProblem = wrapped
    print(f"[shared-td] shared shift={shared_f:.9f} s  (t_imu = t_cam + shift)", flush=True)
    runpy.run_path(kalibr_bin, run_name="__main__")
    _patch_yaml(Path(".").resolve(), shared_f)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
