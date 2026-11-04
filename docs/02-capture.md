# 采集说明

## 场次目录约定

```text
20260910_175512/
  fisheye0_<id>.mp4
  fisheye1_*.mp4
  fisheye2_*.mp4
  fisheye3_*.mp4
  frames_<id>.csv          # cam_id, inst_fps, unix_ns, imu_nearest, nn_delta_ms
  frame_timestamps.csv     # 含 frame_seq，用于看丢帧
  imu_<id>.bin             # RKIMU002
  recording_report.json
  sess_<id>.json
```

时间基准一律 **unix_ns**。相机 V4L2 原是 `CLOCK_MONOTONIC`，采集端已加 `mono_to_unix_offset_ns`。IMU 驱动是 `CLOCK_REALTIME`，并带 **15 ms** 软件滤波时延；准备脚本写入 bag 时用 `t_unix − 15ms`。

## AprilGrid

与 `config/aprilgrid_6x6.yaml` 一致：

- 家族 tag36h11，6×6，id 0–35
- `tagSize = 0.055` m
- `tagSpacing = 0.3`（缝/边长，不是米）
- `reverseTagCols = true`（实体板列方向）
- `blackTagBorder = 2`（实体 tag 双黑边；OpenCV 默认值 1 不适用）

改板必须同步改 yaml，否则外参尺度全错。

## 纯相机内外参怎么录

- 板可以手持，让多路都能看到、覆盖视场
- 不必长时间静置
- 尽量清晰、少运动模糊

## 相机–IMU 怎么录（必须按这个）

Kalibr 假设 **标定板在世界系静止**，IMU 积分的是设备自己的运动。

1. 板固定在墙或三角架，不要手持晃板。
2. 设备（多路鱼眼 + IMU）放桌上 **静置 30～60 秒**。不必水平，但必须真的不动。
3. 拿起设备，在固定板前绕滚/俯/偏转，并给一些平移和加减速，让多路都能看到板。
4. 鱼眼+IMU 相对位姿在整段里保持刚性（不要拧镜头、拆 IMU）。

静置用来估 yaml 里的白噪声、给优化一个干净起点；`T_cam_imu` 靠后面的运动。另做 Allan（30 分钟以上）可以替换 `imu.yaml` 的 random_walk。

## 丢帧

准备脚本 **不用 frame_id 当时间**。`frame_seq` 跳号只记在 `align_report.json`。csv 比 mp4 多 1 帧时丢掉最后一条时间戳。
