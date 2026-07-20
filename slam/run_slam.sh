#!/usr/bin/env bash
# Twin Engine — ORB-SLAM3 wrapper (requires compiled third_party/ORB_SLAM3).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ORB_ROOT="${REPO_ROOT}/third_party/ORB_SLAM3"

usage() {
  echo "Usage: $0 <tum|euroc> <dataset_root> <output_poses.txt>"
  exit 1
}

[[ $# -ge 3 ]] || usage

MODE="$1"
DATASET_ROOT="$2"
OUTPUT_POSES="$3"

mkdir -p "$(dirname "${OUTPUT_POSES}")"

if [[ ! -d "${ORB_ROOT}" ]]; then
  echo "ERROR: ORB-SLAM3 not found at ${ORB_ROOT}"
  echo "Install ORB-SLAM3 under third_party/ or use --identity-poses in run_pipeline.py"
  exit 1
fi

case "${MODE}" in
  tum)
    EXEC="${ORB_ROOT}/Examples/RGB-D/rgbd_tum"
    VOC="${ORB_ROOT}/Vocabulary/ORBvoc.txt"
    SETTINGS="${ORB_ROOT}/Examples/RGB-D/TUM1.yaml"
    ;;
  euroc)
    EXEC="${ORB_ROOT}/Examples/Monocular/mono_euroc"
    VOC="${ORB_ROOT}/Vocabulary/ORBvoc.txt"
    SETTINGS="${ORB_ROOT}/Examples/Monocular/EuRoC.yaml"
    ;;
  *)
    echo "Unknown mode: ${MODE}"
    usage
    ;;
esac

if [[ ! -x "${EXEC}" ]]; then
  echo "ERROR: ORB-SLAM3 executable not found: ${EXEC}"
  echo "Build ORB-SLAM3 first (see slam/README.md)"
  exit 1
fi

echo "Running ORB-SLAM3 (${MODE}) on ${DATASET_ROOT}..."
"${EXEC}" "${VOC}" "${SETTINGS}" "${DATASET_ROOT}" "${DATASET_ROOT}/associations.txt"

# ORB-SLAM3 writes KeyFrameTrajectory.txt in CWD — copy to requested output
if [[ -f KeyFrameTrajectory.txt ]]; then
  cp KeyFrameTrajectory.txt "${OUTPUT_POSES}"
  echo "Poses saved to ${OUTPUT_POSES}"
else
  echo "WARNING: KeyFrameTrajectory.txt not found — check ORB-SLAM3 output"
  exit 1
fi
