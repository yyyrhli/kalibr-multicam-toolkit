#!/usr/bin/env python3
"""相机链 YAML：fisheyeN 键 <-> Kalibr IMU 标定要求的 cam0/cam1/... 键。

kalibr_calibrate_cameras 产出经 convert 后顶层键是 fisheye0。
kalibr_calibrate_imu_camera 读 camchain 时硬编码 data['cam0']，必须转回去。
rostopic 仍用 /fisheyeN/image_raw，与 bag 一致。
"""
from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def fisheye_to_camn(src: Path, dst: Path | None = None, n_cams: int = 4) -> Path:
    raw = yaml.safe_load(src.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise RuntimeError(f"camchain 不是 mapping: {src}")
    out: dict = {}
    for i in range(n_cams):
        name = f"fisheye{i}"
        key = f"cam{i}"
        if name in raw:
            cam = dict(raw[name])
        elif key in raw:
            cam = dict(raw[key])
        else:
            raise RuntimeError(f"{src} 缺少 {name} 或 {key}")
        cam["rostopic"] = f"/fisheye{i}/image_raw"
        cam["cam_overlaps"] = [j for j in range(n_cams) if j != i]
        cam.pop("camera_name", None)
        cam.pop("camera_overlaps", None)
        out[key] = cam
    dst = dst or src
    dst.write_text(yaml.safe_dump(out, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return dst


def main() -> int:
    ap = argparse.ArgumentParser(description="把 fisheyeN camchain 转成 camN，供 IMU 联合标定")
    ap.add_argument("--in", dest="inp", type=Path, required=True)
    ap.add_argument("--out", dest="out", type=Path, required=True)
    ap.add_argument("--num-cams", type=int, default=4)
    args = ap.parse_args()
    fisheye_to_camn(args.inp, args.out, n_cams=args.num_cams)
    print(f"[camchain] {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
