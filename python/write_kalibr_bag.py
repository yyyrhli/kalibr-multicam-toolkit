#!/usr/bin/env python3
"""把选出的灰度 images/ 流式写成 ROS1 mono8 bag。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import rosbag
from genpy import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-dir", type=Path, required=True)
    args = ap.parse_args()
    export_dir = args.export_dir.resolve()
    meta = json.loads((export_dir / "kalibr_meta.json").read_text(encoding="utf-8"))
    images_dir = Path(meta["images_dir"])
    bag_path = Path(meta["bag"])
    if bag_path.exists() and bag_path.is_dir():
        raise RuntimeError(f"ROS1 bag 输出路径不能是目录：{bag_path}")

    image_sets: dict[str, list[tuple[int, Path]]] = {}
    for cam_dir in sorted(images_dir.iterdir()):
        if not cam_dir.is_dir():
            continue
        items = []
        for p in sorted(cam_dir.glob("*.png")):
            items.append((int(p.stem), p))
        image_sets[cam_dir.name] = items
        print(f"[scan] {cam_dir.name}: {len(items)} images")

    events = []
    for name, items in image_sets.items():
        for stamp_ns, path in items:
            events.append((stamp_ns, name, path))
    events.sort()

    with rosbag.Bag(str(bag_path), "w") as writer:
        for stamp_ns, name, path in events:
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                continue
            message_time = Time(
                int(stamp_ns // 1_000_000_000),
                int(stamp_ns % 1_000_000_000),
            )
            msg = Image()
            msg.header = Header(stamp=message_time, frame_id=name)
            msg.height, msg.width = gray.shape[:2]
            msg.encoding = "mono8"
            msg.is_bigendian = 0
            msg.step = int(msg.width)
            msg.data = gray.tobytes()
            writer.write(f"/{name}/image_raw", msg, message_time)
    print(f"[bag] wrote {bag_path} format=ros1 events={len(events)} encoding=mono8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
