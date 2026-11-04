# 场次目录约定

脚本不预设你的采集方案，但要求输入目录满足下面的结构。所有时间基准统一为 **`unix_ns`**（纳秒时间戳）。

## 最小可用目录

```
<session>/
├── fisheye0_<id>.mp4        # 相机 0 视频（后续 fisheye1…N 同理，N 由 --num-cams 或脚本自动探测）
├── fisheye1_<id>.mp4
├── fisheye2_<id>.mp4
├── fisheye3_<id>.mp4
├── frames_<id>.csv          # 每路一帧一行的时间戳索引
├── frame_timestamps.csv     # 可选：带 frame_seq，用于排查丢帧
├── imu_<id>.bin             # IMU 原始数据（联合标定阶段需要）
└── sess_<id>.json           # 可选：会话元信息
```

## `frames_*.csv` 格式

首行可以是注释（`#` 开头）或表头，脚本会跳过。数据行为逗号分隔：

```
cam_id, inst_fps, unix_ns, imu_nearest, nn_delta_ms
0, 30.0, 1726000000123456789, 1726000000123000000, 0.46
```

| 列 | 含义 |
|---|---|
| `cam_id` | 相机编号，与 `fisheyeN` 的 N 对应 |
| `inst_fps` | 该帧的瞬时帧率（可变帧率视频会浮动） |
| `unix_ns` | 该帧的曝光时刻，纳秒 |
| `imu_nearest` | 最近一条 IMU 样本的时间戳（可选） |
| `nn_delta_ms` | 与最近 IMU 样本的时间差，毫秒（可选） |

## 关于时间戳

这是整套流程里最容易出问题的地方：

- **相机**原始时间如果是 `CLOCK_MONOTONIC`，采集端必须自己加一个 `mono_to_unix_offset_ns` 偏移转成 `unix_ns`
- **IMU** 通常是 `CLOCK_REALTIME`，可以直接用
- 如果 IMU 经过软件滤波，滤波会引入固定延迟。本工具的默认策略（`--imu-signal raw`）是**用原始数据、不做时间补偿**；只有明确知道当次会话的滤波延迟时，才改用 `--imu-signal filtered --filtered-time-compensation-ms <实际值>`

不要把"数据里有补偿后的时间戳"误解为"设备已经完成滤波对齐"。

## 分包录制

如果采集程序按固定时长切包（例如每 30 秒一个 `pack_NNN`），先用 `python/stitch_packs.py` 拼成完整场次：

```bash
python python/stitch_packs.py \
  --session-dir /path/to/session \
  --out-dir /path/to/session/stitched \
  --with-video
```

拼接后的目录结构与上表一致，直接作为下游脚本的 `--session-dir`。

## 标定板

默认 `config/aprilgrid_6x6.yaml` 是 **AprilTag 36h11、6×6、间距比 0.30** 的定义。**尺寸参数（tag 边长）必须换成你自己打印出来的实际值**，否则标定出来的外参尺度是错的，而内参和重投影误差仍然"正常"——这是最隐蔽的一类错误。

如果使用网格不完全一致、或带 `reverseTagCols` / `blackTagBorder` 这类非默认属性，请确保：

1. `config/aprilgrid_6x6.yaml` 里写对
2. 预选用 OpenCV 检测时的参数与它一致

两边不一致会导致检测到的 tag 坐标与 Kalibr 的期望错位。

## 采集动作建议

内参标定和联合标定对动作的要求不同：

- **只标内参**：标定板铺满画面各区域即可，不需要剧烈运动。重点是**覆盖**（中心、四边、四角）和**尺度变化**（近、中、远）。
- **联合标定**：需要给 IMU 足够的激励。绕多个轴旋转 + 三个方向的平移都要有，动作连续平稳，不要只绕单轴摆动，也不要快速晃动导致图像模糊。

`aprilgrid_selection.py` 的门禁会拒绝覆盖不足的数据，所以在现场多花 30 秒比返工重录划算。
