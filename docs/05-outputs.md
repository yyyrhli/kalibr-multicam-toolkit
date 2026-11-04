# 产物与坐标

## 约定

`p_dst = R @ p_src + t`，平移单位 **米**。

Kalibr IMU：

```text
t_imu = t_cam + timeshift_cam_imu
T_cam_imu : p_cam = R @ p_imu + t
```

`kalibr_imu_calibration.json` 里：

- `T_cam_from_imu` = 上述 `T_cam_imu`
- `T_imu_from_cam` = 其逆

## 相机标定输出

| 文件 | 内容 |
|------|------|
| `calib-camchain.yaml` | 内参 + 链外参 `T_cn_cnm1` |
| `kalibr_calibration.json` | 真实相机名、两两外参、重投影 |
| `calib-results-cam.txt` | 文本报告 |
| `calib-report-cam.pdf` | 图 |

Kalibr 原生键是 `cam0`。convert 后顶层为 `fisheye0`。  
**IMU 标定读入前必须再变成 `cam0`**（`20_prepare` 已做）。

## IMU 标定输出

| 文件 | 内容 |
|------|------|
| `camchain.yaml` | 输入（camN 键） |
| `imu.yaml` | 噪声 + `/imu0` |
| `calib.bag` | ROS1 bag；图像 `mono8`，按时间戳流式写入 |
| `calib-camchain-imucam.yaml` | 优化后的 `T_cam_imu` |
| `kalibr_imu_calibration.json` | 给下游用的 JSON |
| `align_report.json` | 时间对齐 / 丢帧 |

## Topic

| Topic | 消息 |
|-------|------|
| `/fisheye0/image_raw` … `/fisheye3/image_raw` | `sensor_msgs/Image` mono8（固定灰度，禁止彩色帧） |
| `/imu0` | `sensor_msgs/Imu`（orientation 协方差 `[0]=-1`） |

Header stamp 必须是真实 unix 时间，不能用「帧号/30」。
