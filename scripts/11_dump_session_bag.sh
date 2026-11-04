#!/usr/bin/env bash
# 按 unix 时间戳选多路鱼眼，写 ROS1 mono8 bag，模型默认 ds-none。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/setup/source_kalibr.sh"

SESSION=""
OUT=""
STRIDE="${CAMERA_STRIDE:-10}"
SYNC_MS="15"
START=0
BAG_CAMERA_WORKERS=4
BAG_QUEUE_DEPTH=2
usage() {
  cat <<'EOF'
用法:
  11_dump_session_bag.sh --session-dir <场次> --out-dir <导出目录>
      [--stride 10] [--sync-ms 15] [--start-frame 0]
      [--bag-camera-workers 4] [--bag-queue-depth 2]

场次目录需含 fisheye0..3 视频、frames_*.csv（unix_ns）。
下一步: bash scripts/12_run_calibrate_cameras.sh --export-dir <导出目录>
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session-dir) SESSION="$2"; shift 2 ;;
    --out-dir) OUT="$2"; shift 2 ;;
    --stride) STRIDE="$2"; shift 2 ;;
    --sync-ms) SYNC_MS="$2"; shift 2 ;;
    --start-frame) START="$2"; shift 2 ;;
    --bag-camera-workers) BAG_CAMERA_WORKERS="$2"; shift 2 ;;
    --bag-queue-depth) BAG_QUEUE_DEPTH="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数 $1"; usage; exit 1 ;;
  esac
done
[[ -n "$SESSION" && -n "$OUT" ]] || { usage; exit 1; }

"$KALIBR_RUNTIME" python "${ROOT}/python/dump_camera_bag.py" \
  --session-dir "$SESSION" \
  --out-dir "$OUT" \
  --stride "$STRIDE" \
  --sync-ms "$SYNC_MS" \
  --start-frame "$START" \
  --bag-camera-workers "$BAG_CAMERA_WORKERS" \
  --bag-queue-depth "$BAG_QUEUE_DEPTH" \
  --min-cams 4

echo "[ok] bag → ${OUT}/calib.bag"
echo "下一步: bash ${ROOT}/scripts/12_run_calibrate_cameras.sh --export-dir $OUT"
