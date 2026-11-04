# Changelog

本项目遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## [Unreleased]

## [1.0.0] — 2026-11-04

首个公开发布版本。

### 包含

- **选帧门禁**（`python/aprilgrid_selection.py`）：AprilGrid 检测 + 八项覆盖度验收，覆盖象限、网格、尺度变化、多相机共见连通图、时间槽同步跨度
- **bag 构建**（`scripts/11_dump_session_bag.sh`、`20_prepare_imu_camera.sh`）：多 worker 并行解码 + 单 writer 按全局时间戳写入 `mono8` bag
- **camchain 键名转换**（`python/camchain_camn.py`）：解决 `kalibr_calibrate_cameras` 输出 `fisheyeN` 而 `kalibr_calibrate_imu_camera` 硬编码 `cam0` 的不兼容
- **共享时间偏移模式**（`python/kalibr_imucam_shared_timeshift.py`）：monkey-patch `IccCalibrator.buildProblem`，冻结时间优化、只解外参，适用于硬同步多相机
- **IMU 噪声实测**（`python/imu_rk002.py`）：从静置段估计连续时间 noise density，白噪声取实测 2 倍，不伪造加速度计零偏
- **模型硬校验**（`python/camera_model.py`）：拒绝非 Double Sphere 配置，避免静默错标
- **OpenCV 4.9 兼容补丁**（`scripts/01_patch_tartancalib_dlt.sh`）：修复 DS 模型下 5 点 DLT 崩溃，幂等可重入

### 已知限制

- 仅支持 Linux（依赖 ROS1 与 `rosbag`）
- `imu_rk002.py` 绑定一种自定义二进制 IMU 格式，换采集方案需改写
- 共享 timeshift 模式假设多相机硬同步，脚本不验证该假设
- 标定板尺寸参数需使用者按实物填写

[Unreleased]: https://github.com/yyyrhli/kalibr-multicam-toolkit/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/yyyrhli/kalibr-multicam-toolkit/releases/tag/v1.0.0
