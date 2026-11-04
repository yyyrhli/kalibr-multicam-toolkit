# 本机 Kalibr 环境

本工具不安装、不编译 `kalibr_ros2`，直接复用本机已有的 ROS1 Noetic + TartanCalib 隔离环境。运行入口已收入本项目：

```text
/path/to/camera_imu_calib_pipeline/
  kalibr_ros/setup/tartan_runtime
```

该项目内包装器负责激活本机现有 conda、catkin 和 TartanCalib。桌面流水线的 Python、`rosbag`、
`rosrun kalibr` 均通过它执行。

## 自检

```bash
cd /path/to/camera_imu_calib_pipeline
bash kalibr_ros/scripts/00_check_env.sh
```

首次安装或迁移现有 TartanCalib 后端时，先应用 OpenCV 4.9 的 DS/DLT 兼容补丁：

```bash
bash kalibr_ros/scripts/01_patch_tartancalib_dlt.sh
```

脚本可重复执行，只修改本机 TartanCalib 的 Omni/DS 初始化阈值并重建
`aslam_cameras`/`aslam_cv_python`/`aslam_cv_backend_python`，不连接或修改设备端。

若工程移动，显式指定运行时：

```bash
export KALIBR_RUNTIME=/new/path/camera_imu_calib_pipeline/kalibr_ros/setup/tartan_runtime
bash kalibr_ros/scripts/00_check_env.sh
```

自检内容包括 ROS1 `rosbag`、`sensor_msgs`、OpenCV AprilTag36h11、Kalibr 相机入口和
IMU–相机入口。鱼眼模型固定为 `ds-none`。

## Bag 约定

- 格式：ROS1 `.bag` 文件，不是 ROS2 sqlite 目录。
- 图像：`sensor_msgs/Image`，编码固定 `mono8`；不写彩色图像，不做 JPEG 二次编码。
- IMU：`sensor_msgs/Imu`，保持完整采样率。
- 写入：相机和 IMU 生成器按时间戳归并后流式写入，不在内存缓存整个会话。
- stride：只作用于图像候选帧，默认 `10`；选中帧还必须通过 AprilGrid 覆盖验收。

## 运行

相机标定：

```bash
bash kalibr_ros/scripts/11_dump_session_bag.sh \
  --session-dir /path/to/session --out-dir /path/to/camera_export --stride 10
bash kalibr_ros/scripts/12_run_calibrate_cameras.sh --export-dir /path/to/camera_export
```

IMU–相机联合标定：

```bash
bash kalibr_ros/scripts/20_prepare_imu_camera.sh \
  --session-dir /path/to/session \
  --camchain /path/to/calib-camchain.yaml \
  --out-dir /path/to/imu_camera_export \
  --stride 10 --imu-signal raw
bash kalibr_ros/scripts/21_run_calibrate_imu_camera.sh --export-dir /path/to/imu_camera_export
```

默认 `raw` 使用 `acc/gyr + unix_ns`，不做预补偿。只有明确选择 `filtered` 时，才按配置的
滤波延迟修正时间戳；Kalibr 自身仍估计残余 `timeshift_cam_imu`。
