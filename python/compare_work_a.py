#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


def cam_rms(txt: Path) -> dict[str, float]:
    out: dict[str, float] = {}
    if not txt.is_file():
        return out
    cam = None
    for line in txt.read_text(errors="replace").splitlines():
        m = re.match(r"cam(\d+)\s", line.strip())
        if m:
            cam = f"cam{m.group(1)}"
        if "reprojection error:" in line and "+-" in line and cam:
            floats = []
            for n in re.findall(r"[-+]?(?:\d+\.\d+|\d+)(?:[eE][-+]?\d+)?", line.replace("[", " ").replace("]", " ").replace(",", " ")):
                try:
                    floats.append(float(n))
                except ValueError:
                    pass
            if len(floats) >= 4:
                sx, sy = floats[-2], floats[-1]
                out[cam] = (sx**2 + sy**2) ** 0.5
    return out


def imu_stats(txt: Path) -> dict:
    out: dict = {"reproj_mean": {}, "timeshift_s": {}}
    if not txt.is_file():
        return out
    lines = txt.read_text(errors="replace").splitlines()
    for i, line in enumerate(lines):
        m = re.search(r"Reprojection error \(cam(\d+)\)(?: \[px\])?:\s+mean\s+([0-9.]+)", line)
        if m:
            out["reproj_mean"][f"cam{m.group(1)}"] = float(m.group(2))
        if "timeshift cam" in line and i + 1 < len(lines):
            m2 = re.search(r"cam(\d+)", line)
            try:
                out["timeshift_s"][f"cam{m2.group(1)}"] = float(lines[i + 1].strip())
            except (ValueError, AttributeError):
                pass
    vals = list(out["reproj_mean"].values())
    out["reproj_mean_avg"] = sum(vals) / len(vals) if vals else None
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--work", type=Path, default=Path("/root/kalibr_work_a"))
    args = ap.parse_args()
    work = args.work
    report: dict = {"server": "cpu_host dsw-1007688 96-core", "cameras": {}, "stills": {}, "imu_camera": {}}
    for d in sorted(work.glob("cam_*")):
        sid = d.name.removeprefix("cam_")
        chain = d / "calib-camchain.yaml"
        rms = cam_rms(d / "calib-results-cam.txt")
        has_nan = (not chain.is_file()) or ("nan" in chain.read_text().lower())
        report["cameras"][sid] = {
            "ok": chain.is_file() and not has_nan,
            "has_nan": has_nan,
            "rms": rms,
            "rms_avg": sum(rms.values()) / len(rms) if rms else None,
        }
    for d in sorted(work.glob("still_yaml_*")):
        y = d / "imu.yaml"
        report["stills"][d.name.removeprefix("still_yaml_")] = y.read_text() if y.is_file() else None
    for d in sorted(work.glob("imucam_*")):
        txt = d / "calib-results-imucam.txt"
        report["imu_camera"][d.name] = {
            "ok": (d / "kalibr_imu_calibration.json").is_file(),
            **imu_stats(txt),
        }
    cams_ok = {k: v for k, v in report["cameras"].items() if v.get("ok") and v.get("rms_avg")}
    imus_ok = {k: v for k, v in report["imu_camera"].items() if v.get("ok") and v.get("reproj_mean_avg")}
    report["best_camera_session"] = min(cams_ok, key=lambda k: cams_ok[k]["rms_avg"]) if cams_ok else None
    report["best_imu_combo"] = min(imus_ok, key=lambda k: imus_ok[k]["reproj_mean_avg"]) if imus_ok else None
    out = work / "comparison.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"best_camera_session": report["best_camera_session"], "best_imu_combo": report["best_imu_combo"], "cameras": {k: v.get("rms_avg") for k, v in report["cameras"].items()}, "imu": {k: v.get("reproj_mean_avg") for k, v in report["imu_camera"].items()}}, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
