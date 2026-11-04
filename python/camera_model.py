"""Single source of truth for the multi-camera array model."""
from __future__ import annotations

import math
from pathlib import Path

import yaml


CONFIG_CAMERA_MODEL = "ds"
KALIBR_CAMERA_MODEL = "ds-none"
DS_INTRINSIC_COUNT = 6


def require_config_model(value: object) -> str:
    model = str(value).strip().lower()
    if model != CONFIG_CAMERA_MODEL:
        raise ValueError(f"设备相机标定只允许 {CONFIG_CAMERA_MODEL} 模型")
    return CONFIG_CAMERA_MODEL


def require_model_list(values: object, count: int) -> list[str]:
    models = [str(value).strip().lower() for value in (values or [])]
    expected = [KALIBR_CAMERA_MODEL] * int(count)
    if models != expected:
        raise ValueError(
            f"Kalibr 相机模型必须为 {expected}，实际为 {models}"
        )
    return expected


def require_ds_camchain(path: Path, *, expected_count: int = 4) -> dict:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or not payload:
        raise ValueError(f"camchain 为空或格式错误：{path}")
    cameras = {
        str(name): entry
        for name, entry in payload.items()
        if isinstance(entry, dict) and "intrinsics" in entry
    }
    if not cameras:
        raise ValueError(f"camchain 没有相机条目：{path}")
    if len(cameras) != expected_count:
        raise ValueError(
            f"camchain 必须包含 {expected_count} 路相机，实际为 {len(cameras)}"
        )
    for name, camera in cameras.items():
        projection = str(camera.get("camera_model", "")).strip().lower()
        distortion = str(camera.get("distortion_model", "none")).strip().lower()
        intrinsics = list(camera.get("intrinsics") or [])
        if projection != CONFIG_CAMERA_MODEL or distortion != "none":
            raise ValueError(
                f"{name} 不是 {KALIBR_CAMERA_MODEL}："
                f"camera_model={projection!r}, distortion_model={distortion!r}"
            )
        if len(intrinsics) != DS_INTRINSIC_COUNT:
            raise ValueError(
                f"{name} DS 内参必须为 {DS_INTRINSIC_COUNT} 维，实际为 {len(intrinsics)}"
            )
        try:
            finite = all(math.isfinite(float(value)) for value in intrinsics)
        except (TypeError, ValueError):
            finite = False
        if not finite:
            raise ValueError(f"{name} DS 内参包含非有限数")
    return payload
