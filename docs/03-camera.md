# 多路鱼眼：官方 `kalibr_calibrate_cameras`

## 流程

```text
AprilGrid 覆盖选帧  →  写 ROS1 mono8 bag  →  kalibr_calibrate_cameras  →  convert JSON
  10_                 11_                      12_
```

选帧使用本项目内置的 OpenCV AprilGrid 检测与空间覆盖验收。  
标定本体是官方二进制，不是 `Kalibr/run_calibrate.sh`。

## 命令

```bash
KR=/path/to/camera_imu_calib_pipeline/kalibr_ros
SESS=/path/to/session

bash $KR/scripts/10_prepare_cameras.sh --session-dir "$SESS" --cams 0 1 2 3
bash $KR/scripts/11_write_camera_bag.sh --export-dir "$SESS/kalibr_export"
bash $KR/scripts/12_run_calibrate_cameras.sh --export-dir "$SESS/kalibr_export"
```

`12` 内部：

```bash
source kalibr_ros/setup/source_kalibr.sh
export MPLBACKEND=Agg
xvfb-run -a "$KALIBR_RUNTIME" rosrun kalibr kalibr_calibrate_cameras \
  --bag "$EXPORT/calib.bag" \
  --topics /fisheye0/image_raw /fisheye1/image_raw /fisheye2/image_raw /fisheye3/image_raw \
  --models ds-none ds-none ds-none ds-none \
  --target "$EXPORT/aprilgrid_target.yaml" \
  --approx-sync 0.02 \
  --mi-tol -1 \
  --dont-show-report
```

## 参数

| 参数 | 取值 | 说明 |
|------|------|------|
| `--models` | `ds-none` | 设备鱼眼固定模型；非 DS 会在标定前被拒绝 |
| `--approx-sync` | `0.02` | 多路近似同步容差（秒） |
| `--mi-tol` | `-1` | 尽量多留互信息选帧 |
| `--dont-show-report` | 开 | 无显示环境必须 |
| `--show-extraction` | **不要加** | 会退回单线程 |

## 产物（`kalibr_export/`）

- `images/fisheyeN/*.png`
- `calib.bag` ROS1 bag；图像为 `sensor_msgs/Image/mono8`
- `calib-camchain.yaml` 顶层键为 `fisheye0`…（convert 之后）
- `kalibr_calibration.json`
- `calib-report-cam.pdf` / `calib-results-cam.txt`

健康重投影大约 **0.3～0.7 px**（各相机 std）。
