#!/usr/bin/env python3
"""设备 IMU 二进制 RKIMU002。

文件布局（小端）：
  72 字节头，含 magic、记录长度 152、采样率约 200 Hz
  每条 152 字节：
    u64 t_unix_ns          驱动打戳（含滤波时延）
    u64 t_comp_ns          采集端记录的补偿时间（仅作诊断对照）
    32 字节元数据
    f64 acc_raw[3]
    f64 gyr_raw[3]
    f64 acc_filt[3]        写入 Kalibr 的加速度，m/s^2
    f64 gyr_filt[3]        写入 Kalibr 的角速度，rad/s
    f64 temperature
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

IMU_MAGIC = b"RKIMU002"
IMU_HEADER = 72
IMU_REC = 152
DEFAULT_FILTER_DELAY_MS = 15.0


@dataclass
class ImuSample:
    t_ns: int
    acc: np.ndarray
    gyr: np.ndarray


def load_imu(
    path: Path,
    *,
    static_end_s: float | None = 8.0,
    static_start_s: float = 0.5,
    static_tail_s: float = 0.0,
    use_raw: bool = True,
    filtered_time_compensation_ms: float = DEFAULT_FILTER_DELAY_MS,
) -> tuple[list[ImuSample], dict]:
    if not math.isfinite(filtered_time_compensation_ms) or filtered_time_compensation_ms < 0:
        raise ValueError("filtered_time_compensation_ms 必须是非负有限数")
    filtered_compensation_ns = int(round(filtered_time_compensation_ms * 1_000_000.0))
    data = path.read_bytes()
    if data[:8] != IMU_MAGIC:
        raise RuntimeError(f"未知 IMU 文件头: {data[:8]!r}，期望 RKIMU002")
    recsize = int(np.frombuffer(data[12:16], dtype="<u4")[0])
    if recsize != IMU_REC:
        raise RuntimeError(f"意外 IMU 记录长度 {recsize}")
    n = (len(data) - IMU_HEADER) // IMU_REC
    samples: list[ImuSample] = []
    acc_f = np.zeros((n, 3), dtype=np.float64)
    gyr_f = np.zeros((n, 3), dtype=np.float64)
    t_raw = np.zeros(n, dtype=np.int64)
    t_comp = np.zeros(n, dtype=np.int64)
    for i in range(n):
        off = IMU_HEADER + i * IMU_REC
        t_raw[i] = int(np.frombuffer(data[off : off + 8], dtype="<u8")[0])
        t_comp[i] = int(np.frombuffer(data[off + 8 : off + 16], dtype="<u8")[0])
        if use_raw:
            acc_f[i] = np.frombuffer(data[off + 48 : off + 72], dtype="<d")
            gyr_f[i] = np.frombuffer(data[off + 72 : off + 96], dtype="<d")
            t_ns = int(t_raw[i])
        else:
            acc_f[i] = np.frombuffer(data[off + 96 : off + 120], dtype="<d")
            gyr_f[i] = np.frombuffer(data[off + 120 : off + 144], dtype="<d")
            # 滤波延迟由当次会话配置显式给出，不隐式依赖设备端固定的 t_comp。
            t_ns = int(t_raw[i]) - filtered_compensation_ns
        if np.linalg.norm(acc_f[i]) < 1.0:
            continue
        samples.append(ImuSample(t_ns=t_ns, acc=acc_f[i].copy(), gyr=gyr_f[i].copy()))

    ok = np.linalg.norm(acc_f, axis=1) > 1.0
    t0 = int(t_raw[ok][0])
    t_last = int(t_raw[ok][-1])
    # 默认取开头 0.5–8 s；独立静置会话把 static_end_s 设为 None。
    # static_tail_s 丢掉结尾（开机/关机晃动）。
    static = ok & ((t_raw - t0) > int(float(static_start_s) * 1e9))
    if static_end_s is not None:
        static = static & ((t_raw - t0) < int(float(static_end_s) * 1e9))
    if static_tail_s and static_tail_s > 0:
        static = static & ((t_last - t_raw) > int(float(static_tail_s) * 1e9))
    dt = np.diff(t_raw[ok].astype(np.float64))
    fs = 1e9 / float(np.median(dt)) if len(dt) else 200.0
    acc_std = acc_f[static].std(axis=0) if static.any() else np.array([0.01, 0.01, 0.01])
    gyr_std = gyr_f[static].std(axis=0) if static.any() else np.array([0.001, 0.001, 0.001])
    recorded_delay_ms = float(np.median(t_raw[ok] - t_comp[ok]) / 1e6)
    meta = {
        "path": str(path),
        "n_raw": n,
        "n_valid": len(samples),
        "rate_hz": fs,
        "signal": "raw" if use_raw else "filt",
        "time_source": (
            "t_unix_ns"
            if use_raw
            else "t_unix_ns - configured_filtered_time_compensation"
        ),
        "filter_delay_ms": 0.0 if use_raw else float(filtered_time_compensation_ms),
        "configured_filtered_time_compensation_ms": (
            None if use_raw else float(filtered_time_compensation_ms)
        ),
        "recorded_t_comp_delay_ms": recorded_delay_ms,
        "t_raw_first_ns": int(t_raw[ok][0]),
        "t_comp_first_ns": int(t_comp[ok][0]),
        "t_raw_last_ns": int(t_raw[ok][-1]),
        "static_s": float((t_raw[static][-1] - t_raw[static][0]) / 1e9) if static.any() else 0.0,
        "static_n": int(static.sum()),
        "acc_filt_std": acc_std.tolist(),
        "gyr_filt_std": gyr_std.tolist(),
        "acc_noise_density": (acc_std / math.sqrt(fs)).tolist(),
        "gyr_noise_density": (gyr_std / math.sqrt(fs)).tolist(),
        "dt_gaps_gt_10ms": int((dt > 10e6).sum()),
        "dt_gaps_gt_20ms": int((dt > 20e6).sum()),
    }
    return samples, meta


def write_imu_yaml(path: Path, imu_meta: dict, topic: str = "/imu0") -> None:
    """白噪声取静置估计的 2 倍并设下限；random_walk 用典型 MEMS。"""
    acc_nd = max(float(np.mean(imu_meta["acc_noise_density"])) * 2.0, 1.5e-3)
    gyr_nd = max(float(np.mean(imu_meta["gyr_noise_density"])) * 2.0, 8e-5)
    path.write_text(
        "\n".join(
            [
                f"# estimated from {imu_meta['static_s']:.2f}s static, rate={imu_meta['rate_hz']:.2f} Hz",
                "# white-noise 2x measured; random walk kept at typical MEMS prior",
                f"accelerometer_noise_density: {acc_nd:.6g}",
                "accelerometer_random_walk: 0.0002",
                f"gyroscope_noise_density: {gyr_nd:.6g}",
                "gyroscope_random_walk: 4.0e-06",
                f"update_rate: {float(imu_meta['rate_hz']):.3f}",
                f"rostopic: {topic}",
                "",
            ]
        ),
        encoding="utf-8",
    )
