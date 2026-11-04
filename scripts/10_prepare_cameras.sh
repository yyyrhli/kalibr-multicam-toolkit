#!/usr/bin/env bash
# 兼容旧入口：直接调用项目内置的多路 DS 选帧 + mono8 bag 生成器。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/setup/source_kalibr.sh"
usage() {
  cat <<'EOF'
用法:
  10_prepare_cameras.sh --session-dir <场次目录> [--out-dir DIR]

默认输出: <session>/kalibr_export
标定板默认 6x6 tag36h11, tagSize=5.5cm, spacing 比=0.3
EOF
}

SESSION=""
OUT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --session-dir) SESSION="$2"; shift 2 ;;
    --out-dir) OUT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数 $1"; usage; exit 1 ;;
  esac
done
[[ -n "$SESSION" ]] || { usage; exit 1; }
OUT="${OUT:-${SESSION}/kalibr_export}"

bash "${ROOT}/scripts/11_dump_session_bag.sh" \
  --session-dir "$SESSION" \
  --out-dir "$OUT"

echo "[ok] 多路 DS 选帧与 mono8 bag 完成: $OUT"
echo "下一步: bash ${ROOT}/scripts/12_run_calibrate_cameras.sh --export-dir $OUT"
