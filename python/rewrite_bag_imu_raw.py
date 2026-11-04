#!/usr/bin/env python3
"""把已有 IMU 联合 bag 里的图像原样拷出，IMU 换成 RKIMU002 原始 acc/gyr + t_unix。"""
from __future__ import annotations

import argparse
import heapq
import json
import shutil
import sys
from pathlib import Path

import rosbag
from genpy import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parent))
from imu_rk002 import load_imu, write_imu_yaml  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src-bag", type=Path, required=True)
    ap.add_argument("--imu-bin", type=Path, required=True)
    ap.add_argument("--still-imu", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--camchain", type=Path, required=True)
    ap.add_argument("--target", type=Path, required=True)
    args = ap.parse_args()

    out = args.out_dir.resolve()
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    imu_samples, imu_meta = load_imu(args.imu_bin.resolve(), use_raw=True)
    _still, still_meta = load_imu(
        args.still_imu.resolve(),
        static_end_s=None,
        static_start_s=15.0,
        static_tail_s=15.0,
        use_raw=True,
    )
    still_meta = dict(still_meta)
    still_meta["used_for"] = "imu.yaml white-noise from raw"
    print(
        f"[imu-raw] n={imu_meta['n_valid']} rate={imu_meta['rate_hz']:.2f}Hz "
        f"still_acc_nd={still_meta['acc_noise_density']} gyr_nd={still_meta['gyr_noise_density']}",
        flush=True,
    )
    write_imu_yaml(out / "imu.yaml", still_meta, "/imu0")
    shutil.copy2(args.camchain, out / "camchain.yaml")
    shutil.copy2(args.target, out / "aprilgrid_target.yaml")

    bag = out / "calib.bag"
    src_bag = args.src_bag.resolve()
    with rosbag.Bag(str(src_bag), "r") as reader:
        topics = sorted(reader.get_type_and_topic_info().topics)

    def image_events():
        with rosbag.Bag(str(src_bag), "r") as reader:
            for topic, message, stamp in reader.read_messages():
                if topic != "/imu0":
                    yield int(stamp.to_nsec()), topic, message

    def imu_events():
        for s in imu_samples:
            message_time = Time(
                int(s.t_ns // 1_000_000_000),
                int(s.t_ns % 1_000_000_000),
            )
            msg = Imu()
            msg.header = Header(stamp=message_time, frame_id="imu0")
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity.x = float(s.gyr[0])
            msg.angular_velocity.y = float(s.gyr[1])
            msg.angular_velocity.z = float(s.gyr[2])
            msg.linear_acceleration.x = float(s.acc[0])
            msg.linear_acceleration.y = float(s.acc[1])
            msg.linear_acceleration.z = float(s.acc[2])
            yield int(s.t_ns), "/imu0", msg

    n_img = 0
    with rosbag.Bag(str(bag), "w") as writer:
        for stamp_ns, topic, message in heapq.merge(
            image_events(), imu_events(), key=lambda item: item[0]
        ):
            message_time = Time(
                int(stamp_ns // 1_000_000_000),
                int(stamp_ns % 1_000_000_000),
            )
            writer.write(topic, message, message_time)
            if topic != "/imu0":
                n_img += 1
    print(f"[bag] images={n_img} imu={len(imu_samples)} -> {bag}", flush=True)

    meta = {
        "bag": str(bag),
        "camchain": str(out / "camchain.yaml"),
        "imu_yaml": str(out / "imu.yaml"),
        "target": str(out / "aprilgrid_target.yaml"),
        "imu_signal": "raw",
        "imu_time": "t_unix_ns",
        "imu_meta": imu_meta,
        "imu_still": still_meta,
        "source_bag": str(args.src_bag.resolve()),
        "topics": [name for name in topics if name != "/imu0"] + ["/imu0"],
    }
    (out / "kalibr_meta.json").write_text(json.dumps(meta, indent=2, default=str) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
