#!/usr/bin/env bash
# 检查本机 ROS1 Noetic + TartanCalib + xvfb 是否可用。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/setup/source_kalibr.sh"

fail=0
need() {
  local name="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    echo "  OK  ${name}"
  else
    echo "  FAIL ${name}"
    fail=1
  fi
}

echo "[check] python / ROS1 / TartanCalib"
need "ROS1 rosbag + sensor_msgs" "$KALIBR_RUNTIME" python -c "import rosbag; from sensor_msgs.msg import Image, Imu"
need "xvfb-run" test -x /usr/bin/xvfb-run
need "cv2 + AprilTag36h11" "$KALIBR_RUNTIME" python -c "import cv2; assert hasattr(cv2.aruco,'DICT_APRILTAG_36h11')"
need "TartanCalib Double Sphere model" "$KALIBR_RUNTIME" python -c \
  "import aslam_cv; assert hasattr(aslam_cv, 'DoubleSphereCameraGeometry'); assert hasattr(aslam_cv, 'DoubleSphereProjection')"
need "production AprilGrid parameters" "$KALIBR_RUNTIME" python -c \
  "from kalibr_common.ConfigReader import CalibrationTargetParameters; p=CalibrationTargetParameters('${ROOT}/config/aprilgrid_6x6.yaml').getTargetParams(); assert p['tagRows']==6 and p['tagCols']==6 and abs(p['tagSize']-0.055)<1e-12 and abs(p['tagSpacing']-0.3)<1e-12 and p['reverseTagCols'] is True and p['blackTagBorder']==2"

KALIBR_PACKAGE="$($KALIBR_RUNTIME rospack find kalibr 2>/dev/null || true)"
need "Kalibr ROS1 package" test -d "$KALIBR_PACKAGE"
need "kalibr_calibrate_cameras" test -f "$KALIBR_PACKAGE/python/kalibr_calibrate_cameras"
need "kalibr_calibrate_imu_camera" test -f "$KALIBR_PACKAGE/python/kalibr_calibrate_imu_camera"

echo "[check] runtime=${KALIBR_RUNTIME}"
echo "[check] kalibr=${KALIBR_PACKAGE}"

exit "${fail}"
