#!/usr/bin/env bash
# 步骤 3：官方 kalibr_calibrate_cameras（ds-none，mi-tol 由环境配置）。
# 与历史 run_kalibr.sh 一致，不走 nros。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/setup/source_kalibr.sh"
export MPLBACKEND=Agg

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

META="${EXPORT}/kalibr_meta.json"
[[ -f "$META" ]] || { echo "缺少 kalibr_meta.json"; exit 1; }

eval "$("$KALIBR_RUNTIME" python - "$META" <<'PY'
import json, shlex, sys
m=json.load(open(sys.argv[1]))
expected=["ds-none"] * 4
if m.get("models") != expected:
    raise SystemExit(f"camera models must be {expected}, got {m.get('models')}")
print("TOPICS="+shlex.quote(" ".join(m["topics"])))
print("MODELS="+shlex.quote(" ".join(m["models"])))
print("TARGET="+shlex.quote(m["target"]))
print("BAG="+shlex.quote(m["bag"]))
PY
)"

CONVERT="${ROOT}/python/convert_kalibr_camchain.py"
[[ -f "$CONVERT" ]] || { echo "缺少项目内 DS 结果转换器: $CONVERT"; exit 1; }

# 直接运行此脚本时默认 -1；工作台会从 config.yaml 传入 0.2。
# -1 可作为全收帧 A/B 基线。
MI_TOL="${KALIBR_MI_TOL:--1}"
# shellcheck disable=SC2206
EXTRA_ARGS=( ${KALIBR_CAM_EXTRA:-} )

set +e
xvfb-run -a "$KALIBR_RUNTIME" rosrun kalibr kalibr_calibrate_cameras \
  --bag "$BAG" \
  --topics $TOPICS \
  --models $MODELS \
  --target "$TARGET" \
  --approx-sync 0.02 \
  --mi-tol "$MI_TOL" \
  "${EXTRA_ARGS[@]}" \
  --dont-show-report
rc=$?
set -e

CHAIN="${EXPORT}/calib-camchain.yaml"
[[ -f "$CHAIN" ]] || { echo "kalibr 没有写出 calib-camchain.yaml (exit=$rc)"; exit 1; }
"$KALIBR_RUNTIME" python - "$CHAIN" "${ROOT}/python" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[2])
from camera_model import require_ds_camchain
require_ds_camchain(Path(sys.argv[1]))
print("[ok] camchain 为多路 ds-none 六维内参")
PY
if [[ "$rc" -ne 0 ]]; then
  echo "[warn] kalibr 退出码 $rc（常见于 PDF/igraph），camchain 已保留"
fi
"$KALIBR_RUNTIME" python "$CONVERT" --export-dir "$EXPORT"
echo "[ok] 相机标定完成: ${EXPORT}/calib-camchain.yaml"
