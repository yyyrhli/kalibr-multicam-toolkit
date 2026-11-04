# 鱼眼–IMU：官方 `kalibr_calibrate_imu_camera`

## 前置

- 已有相机 `calib-camchain.yaml`（`ds-none`，topic `/fisheyeN/image_raw`）
- 本场次有 `imu_*.bin` + `frames_*.csv` + 多路 mp4
- 录制遵守 [02-capture.md](02-capture.md)：板固定，设备先静置再运动

相机内外参在这一步 **默认冻结**。本命令估的是 `T_cam_imu` 和 `timeshift_cam_imu`。

## 流程

```text
时间对齐 + 写 bag（图像+IMU）  →  kalibr_calibrate_imu_camera  →  JSON
  20_                                  21_
```

```bash
KR=/path/to/camera_imu_calib_pipeline/kalibr_ros
SESS=.../20260910_175512
CHAIN=/path/to/camera_export/calib-camchain.yaml
ACCEPTED_IMU=/path/to/accepted/imu.yaml

bash $KR/scripts/20_prepare_imu_camera.sh \
  --session-dir "$SESS" \
  --camchain "$CHAIN" \
  --out-dir "$SESS/kalibr_imu_export" \
  --start-frame 900 \
  --stride 10 \
  --accepted-imu-yaml "$ACCEPTED_IMU" \
  --imu-signal raw

bash $KR/scripts/21_run_calibrate_imu_camera.sh --export-dir "$SESS/kalibr_imu_export"
```

`--start-frame`：跳过静置。30 fps 下 30 秒静置约 900 帧。不准也可以先 0，Kalibr 会丢掉看不到板的帧，只是 bag 更大。

`--stride 10`：约 3 Hz 图像 + 全速率 IMU。stride 只影响图像，不会降采样 IMU。

IMU 默认用 `--imu-signal raw`：读取原始 `acc/gyr` 和 `unix_ns`，不做时间补偿。
如确定需要使用滤波数据，必须同时给出当次会话的实际滤波延迟：

```bash
--imu-signal filtered --filtered-time-compensation-ms 15.0
```

此时读取 `acc_filt/gyr_filt`，时间戳为 `unix_ns - 配置延迟`。配置值应来自会话
`imu_filter_delay_ms`，不再隐式假定固定 15 ms。

## 准备脚本在做什么

`python/prepare_kalibr_imu_ros.py`：

1. `frames_*.csv` 的 unix_ns 对齐各路 mp4（处理少 1 帧、seq 丢帧）
2. 对 stride 候选帧检测 AprilGrid，验收画面覆盖、尺度变化和相机共见连通性
3. 读 `RKIMU002`：默认原始 acc/gyro 且无时间补偿；可显式切换滤波数据和延迟
4. 工作台模式校验并逐字节复用已通过静置阶段的 `accepted/imu.yaml`（topic 必须是 `/imu0`）；
   独立使用准备脚本且未传 `--accepted-imu-yaml` 时，才从 `--still-imu` 或当前会话静置段估算
5. `camchain_camn.py` 把 `fisheyeN` 键改成 `cam0`…（**Kalibr 硬编码 cam0**）
6. 按时间戳流式写 ROS1 bag：`/fisheyeN/image_raw`（mono8、非彩色）+ 全频率 `/imu0`

工作台不会递归挑选“最大的一份旧静置数据”。联合阶段只接收本机状态机已经验收并复制到
`accepted/imu.yaml` 的结果，因此静置失败后重新录制不会又误用更大的失败 attempt。

选帧证据在 `frame_selection_report.json`。如任意一路画面覆盖不足，或四相机共见图不连通，
数据准备会失败，不把低质量 bag 交给 Kalibr。

## 官方命令

```bash
xvfb-run -a "$KALIBR_RUNTIME" rosrun kalibr kalibr_calibrate_imu_camera \
  --bag ./calib.bag \
  --cams ./camchain.yaml \
  --imu ./imu.yaml \
  --target ./aprilgrid_target.yaml \
  --timeoffset-padding 0.08 \
  --dont-show-report
```

禁止 `--recover-covariance`：`BSplineMotionError::evaluateJacobiansImplementation` 未实现，进程 abort。

时间偏移约定：`t_imu = t_cam + timeshift_cam_imu`。多路 shift 应接近（例如都在 −12 ms 附近）。

## 误差怎么看

| 项 | 合理范围 | 过大常见原因 |
|----|----------|----------------|
| 纯相机重投影 | 0.3～0.7 px | 板参数错、模糊、模型不对 |
| IMU 联合重投影 | 验收要求每路 median < 1 px | **手持晃板**、激励不足、噪声 yaml 离谱 |
| 多路 timeshift 互差 | < 1 ms 量级 | 某路时间戳错 |

手持晃板 + 冻住的旧相机链时，联合重投影可能达到 2～3 px；这不等同于内参损坏，
但按当前量产门禁仍判联合标定失败，需要改善数据或联合优化后重试。

## 产物

- `calib-camchain-imucam.yaml`：含 `T_cam_imu`、`timeshift_cam_imu`
- `kalibr_imu_calibration.json`：按 fisheye 名整理的外参
- `calib-report-imucam.pdf`
- `align_report.json`：丢帧与 IMU 静置统计
