#!/usr/bin/env bash
# 使用本机已安装的 ROS1 Noetic + TartanCalib 隔离运行时。
# 真正的 ROS/conda 环境由项目内置 tartan_runtime 为每条命令加载。
set -euo pipefail

SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KALIBR_RUNTIME="${KALIBR_RUNTIME:-${SETUP_DIR}/tartan_runtime}"
if [[ ! -x "${KALIBR_RUNTIME}" ]]; then
  echo "[kalibr] 找不到本机 TartanCalib 运行时：${KALIBR_RUNTIME}" >&2
  echo "  可通过 KALIBR_RUNTIME 指定另一个已安装的 ROS1/TartanCalib 运行入口。" >&2
  return 1 2>/dev/null || exit 1
fi

export KALIBR_RUNTIME
export MPLBACKEND="${MPLBACKEND:-Agg}"
export PYTHONUNBUFFERED=1

echo "[kalibr] backend=ROS1/TartanCalib runtime=${KALIBR_RUNTIME}"
