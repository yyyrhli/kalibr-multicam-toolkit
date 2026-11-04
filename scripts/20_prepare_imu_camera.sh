#!/usr/bin/env bash
# 相机–IMU：写 ROS1 mono8 bag。默认原始 IMU + unix_ns，无时间补偿。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/setup/source_kalibr.sh"

SESSION=""
OUT=""
CAMCHAIN=""
START=0
STRIDE=10
STILL_IMU=""
ACCEPTED_IMU_YAML=""
IMU_SIGNAL="raw"
FILTERED_TIME_COMPENSATION_MS="15.0"
BAG_CAMERA_WORKERS=4
BAG_QUEUE_DEPTH=2
FRAME_SELECTION_WORKERS=12
PROGRESS_JSON=""
SELECTION_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --session-dir) SESSION="$2"; shift 2 ;;
    --out-dir) OUT="$2"; shift 2 ;;
    --camchain) CAMCHAIN="$2"; shift 2 ;;
    --start-frame) START="$2"; shift 2 ;;
    --stride) STRIDE="$2"; shift 2 ;;
    --still-imu) STILL_IMU="$2"; shift 2 ;;
    --accepted-imu-yaml) ACCEPTED_IMU_YAML="$2"; shift 2 ;;
    --imu-signal) IMU_SIGNAL="$2"; shift 2 ;;
    --filtered-time-compensation-ms) FILTERED_TIME_COMPENSATION_MS="$2"; shift 2 ;;
    --bag-camera-workers) BAG_CAMERA_WORKERS="$2"; shift 2 ;;
    --bag-queue-depth) BAG_QUEUE_DEPTH="$2"; shift 2 ;;
    --frame-selection-workers) FRAME_SELECTION_WORKERS="$2"; shift 2 ;;
    --progress-json) PROGRESS_JSON="$2"; shift 2 ;;
    --min-tags|--min-board-area-ratio|--min-blur-score|--min-border-margin-ratio|--min-perspective-ratio|--coverage-grid-cols|--coverage-grid-rows|--min-coverage-ratio|--min-center-quadrants|--min-scale-ratio|--min-detected-frames-per-camera|--min-detected-cameras-per-slot|--min-covisible-frames|--information-filter|--duplicate-distance|--max-selection-gap-seconds)
      SELECTION_ARGS+=("$1" "$2"); shift 2 ;;
    *) echo "未知参数 $1"; exit 1 ;;
  esac
done
[[ -n "$SESSION" && -n "$CAMCHAIN" ]] || {
  echo "用法: 20_prepare_imu_camera.sh --session-dir DIR --camchain camchain.yaml [--out-dir DIR] [--start-frame N] [--stride 10] [--bag-camera-workers 4] [--bag-queue-depth 2] [--still-imu imu.bin] [--imu-signal raw|filtered] [--filtered-time-compensation-ms MS]"
  exit 1
}
OUT="${OUT:-${SESSION}/kalibr_imu_export}"

PREP=(
  "$KALIBR_RUNTIME" python "${ROOT}/python/prepare_kalibr_imu_ros.py"
  --session-dir "$SESSION"
  --out-dir "$OUT"
  --camchain "$CAMCHAIN"
  --dump-all-after
  --start-frame "$START"
  --stride "$STRIDE"
  --bag-camera-workers "$BAG_CAMERA_WORKERS"
  --bag-queue-depth "$BAG_QUEUE_DEPTH"
  --frame-selection-workers "$FRAME_SELECTION_WORKERS"
  --imu-signal "$IMU_SIGNAL"
  --filtered-time-compensation-ms "$FILTERED_TIME_COMPENSATION_MS"
  "${SELECTION_ARGS[@]}"
)
[[ -n "$PROGRESS_JSON" ]] && PREP+=(--progress-json "$PROGRESS_JSON")
[[ -n "$STILL_IMU" ]] && PREP+=(--still-imu "$STILL_IMU")
[[ -n "$ACCEPTED_IMU_YAML" ]] && PREP+=(--accepted-imu-yaml "$ACCEPTED_IMU_YAML")
"${PREP[@]}"

echo "[ok] IMU bag 准备完成: $OUT"
echo "下一步: bash ${ROOT}/scripts/21_run_calibrate_imu_camera.sh --export-dir $OUT"
