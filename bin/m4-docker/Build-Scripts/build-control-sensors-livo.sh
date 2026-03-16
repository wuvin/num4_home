#!/usr/bin/env bash
set -euo pipefail
# Usage: ./build-controls-sensors-livo.sh [laptop|jetson|image=<base>]

resolve_base() {
  case "${1:-laptop}" in
    laptop) echo "ros:humble-ros-base" ;;
    jetson) echo "dustynv/ros:humble-ros-base-l4t-r36.3.0" ;;
    image=*) echo "${1#image=}" ;;
    *) echo "ros:humble-ros-base-jammy" ;;
  esac
}

detect_arch() {
  local machine_arch="$(uname -m)"
  case "${machine_arch}" in
    x86_64|amd64) echo "amd" ;;
    aarch64|arm64) echo "arm" ;;
    *) echo "Unsupported architecture: ${machine_arch}" >&2; exit 1 ;;
  esac
}

BASE_PARENT="$(resolve_base "${1:-laptop}")"
ARCH="$(detect_arch)"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
IMAGE_NAME="num4:controls-sensors-livo"
DESC="Variant: base + mavtools + realsense + livox + fastlivo2 | parent=${BASE_PARENT}"

TMP_BASE="m4_tmp:base-$(date +%s)"
TMP_MAV="m4_tmp:mavtools-$(date +%s)"
TMP_RS="m4_tmp:realsense-$(date +%s)"
TMP_LIVOX="m4_tmp:livox-$(date +%s)"

docker build \
  --network=host \
  -f "${REPO_ROOT}/Dockerfiles/Dockerfile.base" \
  -t "${TMP_BASE}" \
  --build-arg BASE_IMAGE="${BASE_PARENT}" \
  "${REPO_ROOT}"

docker build \
  --network=host \
  -f "${REPO_ROOT}/Dockerfiles/Dockerfile.mavtools" \
  -t "${TMP_MAV}" \
  --build-arg BASE_IMAGE="${TMP_BASE}" \
  --build-arg ARCH="${ARCH}" \
  "${REPO_ROOT}"

docker build \
  --network=host \
  -f "${REPO_ROOT}/Dockerfiles/Dockerfile.realsense" \
  -t "${TMP_RS}" \
  --build-arg BASE_IMAGE="${TMP_MAV}" \
  "${REPO_ROOT}"

docker build \
  --network=host \
  -f "${REPO_ROOT}/Dockerfiles/Dockerfile.livox" \
  -t "${TMP_LIVOX}" \
  --build-arg BASE_IMAGE="${TMP_RS}" \
  "${REPO_ROOT}"

docker build \
  --network=host \
  -f "${REPO_ROOT}/Dockerfiles/Dockerfile.fastlivo2" \
  -t "${IMAGE_NAME}" \
  --build-arg BASE_IMAGE="${TMP_LIVOX}" \
  --label org.opencontainers.image.title="m4-controls-sensors-livo" \
  --label org.opencontainers.image.description="${DESC}" \
  --label m4.variant="base+mavtools+realsense+livox+fastlivo2" \
  "${REPO_ROOT}"

docker rmi -f "${TMP_LIVOX}" "${TMP_RS}" "${TMP_MAV}" "${TMP_BASE}" 2>/dev/null || true
docker image prune -f >/dev/null
echo "Built ${IMAGE_NAME} (${DESC})"
