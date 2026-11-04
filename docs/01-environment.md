# 环境：本机 ROS1 Noetic + TartanCalib

本工具复用已经验证的隔离运行时，不再构建 `kalibr_ros2`：

```bash
cd /path/to/camera_imu_calib_pipeline
bash kalibr_ros/scripts/00_check_env.sh
```

| 变量 | 含义 |
|------|------|
| `KALIBR_RUNTIME` | `tartan_runtime` 路径；默认指向本项目的 `kalibr_ros/setup/tartan_runtime` |

写 bag和跑标定都通过该包装器执行，避免 ROS1 与系统 Python 环境互相污染。
