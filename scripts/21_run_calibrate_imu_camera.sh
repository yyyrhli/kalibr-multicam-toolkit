#!/usr/bin/env bash
# 官方 kalibr_calibrate_imu_camera。
# 不要加 --recover-covariance：BSplineMotionError 的 Jacobian 未实现，会 abort。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/setup/source_kalibr.sh"
export MPLBACKEND=Agg
export PYTHONUNBUFFERED=1

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
print("[ok] 联合标定输入为 ds-none")
PY

xvfb-run -a "$KALIBR_RUNTIME" rosrun kalibr kalibr_calibrate_imu_camera \
  --bag "${EXPORT}/calib.bag" \
  --cams "${EXPORT}/camchain.yaml" \
  --imu "${EXPORT}/imu.yaml" \
  --target "${EXPORT}/aprilgrid_target.yaml" \
  --timeoffset-padding 0.08 \
  --dont-show-report

"$KALIBR_RUNTIME" python "${ROOT}/python/convert_kalibr_camchain.py" \
  --export-dir "$EXPORT" \
  --camchain "${EXPORT}/calib-camchain-imucam.yaml" \
  --results-txt "${EXPORT}/calib-results-imucam.txt" \
  --out-json "${EXPORT}/kalibr_calibration.json" || true
"$KALIBR_RUNTIME" python "${ROOT}/python/convert_kalibr_imucam.py" --export-dir "$EXPORT"
echo "[ok] IMU 联合标定完成: ${EXPORT}/kalibr_imu_calibration.json"
