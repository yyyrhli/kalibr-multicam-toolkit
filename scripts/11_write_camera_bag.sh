#!/usr/bin/env bash
# 步骤 2：把 kalibr_export/images 写成 ROS1 mono8 bag。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/setup/source_kalibr.sh"

EXPORT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --export-dir) EXPORT="$2"; shift 2 ;;
    *) echo "未知参数 $1"; exit 1 ;;
  esac
done
[[ -n "$EXPORT" ]] || { echo "需要 --export-dir"; exit 1; }

WRITE="${ROOT}/python/write_kalibr_bag.py"
[[ -f "$WRITE" ]] || { echo "缺少项目内 bag 生成器: $WRITE"; exit 1; }

"$KALIBR_RUNTIME" python "$WRITE" --export-dir "$EXPORT"
echo "[ok] bag → ${EXPORT}/calib.bag"
echo "下一步: bash ${ROOT}/scripts/12_run_calibrate_cameras.sh --export-dir $EXPORT"
