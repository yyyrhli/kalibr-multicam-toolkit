#!/usr/bin/env bash
# Make TartanCalib's Omni/DS initializer compatible with OpenCV >= 4.9 DLT.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIPELINE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CALIB_CACHE_ROOT="${HEADSET_CALIB_CACHE_ROOT:-${HOME}/.cache/headset-calib}"
TARTAN_WS="${HEADSET_CALIB_TARTAN_WS:-${CALIB_CACHE_ROOT}/tartan_ws}"
TARTAN_SOURCE="${TARTAN_WS}/src/tartancalib"
OMNI_TARGET="${TARTAN_SOURCE}/aslam_cv/aslam_cameras/include/aslam/cameras/implementation/OmniProjection.hpp"
DS_TARGET="${TARTAN_SOURCE}/aslam_cv/aslam_cameras/include/aslam/cameras/implementation/DoubleSphereProjection.hpp"
PATCH_FILES=(
  "${PIPELINE_ROOT}/scripts/patches/tartancalib-opencv49-dlt-min6.patch"
  "${PIPELINE_ROOT}/scripts/patches/tartancalib-opencv49-solvepnp-min6.patch"
)
JOBS="${JOBS:-$(nproc)}"

if [[ ! -f "${OMNI_TARGET}" || ! -f "${DS_TARGET}" || ! -d "${TARTAN_SOURCE}/.git" ]]; then
  echo "TartanCalib source tree not found: ${TARTAN_SOURCE}" >&2
  exit 2
fi
if [[ ! "${JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "JOBS must be a positive integer" >&2
  exit 2
fi

for patch_file in "${PATCH_FILES[@]}"; do
  if git -C "${TARTAN_SOURCE}" apply --check "${patch_file}" 2>/dev/null; then
    git -C "${TARTAN_SOURCE}" apply "${patch_file}"
    echo "Applied $(basename "${patch_file}")."
  elif git -C "${TARTAN_SOURCE}" apply --reverse --check "${patch_file}" 2>/dev/null; then
    echo "Already applied: $(basename "${patch_file}")."
  else
    echo "Cannot safely apply DLT patch: ${patch_file}" >&2
    exit 2
  fi
done

"${PIPELINE_ROOT}/kalibr_ros/setup/tartan_runtime" \
  catkin build --workspace "${TARTAN_WS}" \
  aslam_cameras aslam_cv_python aslam_cv_backend_python \
  --no-deps --jobs "${JOBS}"

if ! grep -q 'const size_t MIN_CORNERS = 5;' "${OMNI_TARGET}" || \
   ! grep -q 'if (Ps.size() < 6)' "${OMNI_TARGET}" || \
   ! grep -q 'if (Ps.size() < 6)' "${DS_TARGET}"; then
  echo "DLT compatibility patch verification failed." >&2
  exit 2
fi
echo "TartanCalib Omni/DS DLT compatibility patch is active."
