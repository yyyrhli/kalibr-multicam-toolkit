#!/usr/bin/env python3
"""把会话下 pack_NNN 按编号拼成 Kalibr 可用的一场数据。"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from imu_rk002 import IMU_HEADER, IMU_MAGIC, IMU_REC  # noqa: E402
from session_timestamps import load_frame_index, map_video_timestamps  # noqa: E402


def pack_index(p: Path) -> int:
    return int(p.name.split("_")[-1])


def list_packs(session: Path) -> list[Path]:
    packs = sorted(session.glob("pack_*"), key=pack_index)
    out = []
    for pack in packs:
        imu = next(pack.glob("imu_*.bin"), None)
        if imu is None or imu.stat().st_size < IMU_HEADER + IMU_REC:
            print(f"[skip] {pack.name} empty imu", flush=True)
            continue
        out.append(pack)
    if not out:
        raise RuntimeError(f"没有有效 pack: {session}")
    return out


def stitch_imu(packs: list[Path], dst: Path) -> int:
    chunks: list[bytes] = []
    n_rec = 0
    for i, pack in enumerate(packs):
        raw = next(pack.glob("imu_*.bin")).read_bytes()
        if raw[:8] != IMU_MAGIC:
            raise RuntimeError(f"{pack} IMU magic {raw[:8]!r}")
        recs = raw[IMU_HEADER:]
        n = len(recs) // IMU_REC
        recs = recs[: n * IMU_REC]
        n_rec += n
        if i == 0:
            chunks.append(raw[:IMU_HEADER] + recs)
        else:
            chunks.append(recs)
    dst.write_bytes(b"".join(chunks))
    return n_rec


def stitch_frames(ns: dict[int, list[int]], dst: Path, session_id: str) -> None:
    lines = [
        "# stitched packs frames csv",
        f"# session_id={session_id}",
        "# time_base=unix_ns",
        "cam_id,inst_fps,unix_ns,imu_nearest_unix_ns,nn_delta_ms",
    ]
    for cam in range(4):
        prev = None
        for t in ns.get(cam, []):
            fps = 0.0 if prev is None else (1e9 / max(t - prev, 1))
            lines.append(f"{cam},{fps:.6f},{t},,")
            prev = t
    dst.write_text("\n".join(lines) + "\n", encoding="utf-8")


def ffmpeg_concat(videos: list[Path], dst: Path) -> None:
    lst = dst.with_suffix(".concat.txt")
    lst.write_text("".join(f"file '{p.resolve()}'\n" for p in videos), encoding="utf-8")
    cmd = [
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
        str(lst),
        "-c",
        "copy",
        str(dst),
    ]
    subprocess.run(cmd, check=True)


def video_n(path: Path) -> int:
    import cv2

    cap = cv2.VideoCapture(str(path))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    cap.release()
    return n


def align_stitched_timestamps(
    timestamps: list[int], video_frames: int
) -> tuple[list[int], str]:
    """Align once after concatenation; timestamp gaps preserve mid-stream drops."""
    used_timestamps = map_video_timestamps(timestamps, video_frames)
    action = "already_aligned"
    if video_frames == len(timestamps) + 1:
        action = "ignore_last_video_frame"
    elif len(timestamps) == video_frames + 1:
        action = "drop_last_timestamp"
    return used_timestamps, action


def stitch_session(session: Path, out: Path, *, with_video: bool) -> dict:
    packs = list_packs(session)
    out.mkdir(parents=True, exist_ok=True)
    sid = session.name
    print(f"[stitch] {session} packs={[p.name for p in packs]} -> {out}", flush=True)

    imu_n = stitch_imu(packs, out / f"imu_{sid}.bin")
    aligned: dict[int, list[int]] = {i: [] for i in range(4)}
    encoded: dict[str, int] = {}
    video_alignment: dict[str, dict] = {}
    physical_video_frames: dict[str, int] = {}

    if with_video:
        for cam in range(4):
            parts: list[Path] = []
            all_timestamps: list[int] = []
            pack_counts: list[dict] = []
            for pack in packs:
                video = next(pack.glob(f"cam{cam}_*.mp4"))
                frame_count = video_n(video)
                ns = load_frame_index(pack)
                if cam not in ns:
                    raise RuntimeError(f"{pack} 缺少 cam{cam} 时间戳")
                all_timestamps.extend(ns[cam])
                parts.append(video)
                pack_counts.append(
                    {
                        "pack": pack.name,
                        "csv_frames": len(ns[cam]),
                        "video_frames": frame_count,
                    }
                )
                print(
                    f"  {pack.name} cam{cam} csv={len(ns[cam])} "
                    f"video={frame_count} action=defer_to_full_session",
                    flush=True,
                )
            dst = out / f"fisheye{cam}_{sid}.mp4"
            ffmpeg_concat(parts, dst)
            n_cat = video_n(dst)
            used, action = align_stitched_timestamps(all_timestamps, n_cat)
            aligned[cam] = used
            encoded[f"fisheye{cam}"] = len(used)
            physical_video_frames[f"fisheye{cam}"] = n_cat
            video_alignment[f"fisheye{cam}"] = {
                "packs": pack_counts,
                "total_csv_frames": len(all_timestamps),
                "total_video_frames": n_cat,
                "used_frames": len(used),
                "action": action,
            }
            print(
                f"  [align-full] cam{cam} csv={len(all_timestamps)} "
                f"video={n_cat} used={len(used)} action={action}",
                flush=True,
            )
    else:
        for pack in packs:
            ns = load_frame_index(pack)
            for cam, ts in ns.items():
                aligned[int(cam)].extend(ts)

    stitch_frames(aligned, out / f"frames_{sid}.csv", sid)
    report = {
        "session_id": sid,
        "source": str(session),
        "packs": [p.name for p in packs],
        "imu_records": imu_n,
        "encoded_frames": encoded,
        "physical_video_frames": physical_video_frames,
        "csv_frames": {f"fisheye{k}": len(v) for k, v in aligned.items()},
        "video_alignment": video_alignment,
    }
    (out / "recording_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"[ok] {sid} imu_rec={imu_n} frames={report['csv_frames']}", flush=True)
    return report


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--session-dir", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--with-video", action="store_true")
    args = ap.parse_args()
    stitch_session(args.session_dir.resolve(), args.out_dir.resolve(), with_video=args.with_video)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
