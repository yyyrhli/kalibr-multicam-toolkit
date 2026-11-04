#!/usr/bin/env bash
# IMU 联合：多路共享 timeshift（硬同步），--no-time-calibration，只优化外参。
# 用法: KALIBR_SHARED_TIMESHIFT_S=-0.03846 bash 21_run_calibrate_imu_camera_shared.sh --export-dir DIR
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/setup/source_kalibr.sh"
export MPLBACKEND=Agg
export PYTHONUNBUFFERED=1

[[ -n "${KALIBR_SHARED_TIMESHIFT_S:-}" ]] || {
  echo "需要 KALIBR_SHARED_TIMESHIFT_S（秒，t_imu=t_cam+shift）"
  exit 1
}

EXPORT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --export-dir) EXPORT="$2"; shift 2 ;;
    *) echo "未知参数 $1"; exit 1 ;;
  esac
done
[[ -n "$EXPORT" ]] || { echo "需要 --export-dir"; exit 1; }
EXPORT="$(cd "$EXPORT" && pwd)"
cd "$EXPORT"

"$KALIBR_RUNTIME" python - "${EXPORT}/camchain.yaml" "${ROOT}/python" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[2])
from camera_model import require_ds_camchain
require_ds_camchain(Path(sys.argv[1]))
print("[ok] 共享时延联合标定输入为 ds-none")
PY

KALIBR_PACKAGE="$($KALIBR_RUNTIME rospack find kalibr)"
export KALIBR_CALIBRATE_IMU_CAMERA="${KALIBR_PACKAGE}/python/kalibr_calibrate_imu_camera"
xvfb-run -a "$KALIBR_RUNTIME" python "${ROOT}/python/kalibr_imucam_shared_timeshift.py" \
  --bag "${EXPORT}/calib.bag" \
  --cams "${EXPORT}/camchain.yaml" \
  --imu "${EXPORT}/imu.yaml" \
  --target "${EXPORT}/aprilgrid_target.yaml" \
  --timeoffset-padding 0.08 \
  --no-time-calibration \
  --dont-show-report

"$KALIBR_RUNTIME" python "${ROOT}/python/convert_kalibr_camchain.py" \
  --export-dir "$EXPORT" \
  --camchain "${EXPORT}/calib-camchain-imucam.yaml" \
  --results-txt "${EXPORT}/calib-results-imucam.txt" \
  --out-json "${EXPORT}/kalibr_calibration.json"
"$KALIBR_RUNTIME" python "${ROOT}/python/convert_kalibr_imucam.py" --export-dir "$EXPORT"
echo "[ok] 共享时延 IMU 联合完成: ${EXPORT}/kalibr_imu_calibration.json  shift=${KALIBR_SHARED_TIMESHIFT_S}"
