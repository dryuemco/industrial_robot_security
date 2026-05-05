#!/usr/bin/env bash
# scripts/build_ursim_image.sh
#
# Build the ENFIELD custom URSim image with External Control URCap baked in.
# Stages the URCap from third_party/urcap/ into docker/ursim/ for the build
# context, then cleans it up via EXIT trap.
#
# Usage:  bash scripts/build_ursim_image.sh
# Output: image tag enfield-ursim:5.12-urcap-1.0.5

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
URCAP_NAME="externalcontrol-1.0.5.urcap"
URCAP_SRC="${REPO_ROOT}/third_party/urcap/${URCAP_NAME}"
URCAP_SHA256="21ee6c9b4d8a64cf47ce2446233627deec0d2999a0440f4261155d79223b4901"
DOCKER_DIR="${REPO_ROOT}/docker/ursim"
STAGED_URCAP="${DOCKER_DIR}/${URCAP_NAME}"
IMAGE_TAG="enfield-ursim:5.12-urcap-1.0.5"
BASE_DIGEST="sha256:b7ad69f5bfa45ffab07788480ad43c753595ce35fcbfe4a3f420725f51764d51"

# --- pre-build guard 1: source URCap exists ---
if [ ! -f "$URCAP_SRC" ]; then
    echo "FATAL: source URCap not found: $URCAP_SRC" >&2
    echo "Vendor it first via Phase 2.A (cp from /opt/ros/humble/share/...)" >&2
    exit 1
fi

# --- pre-build guard 2: source URCap SHA-256 matches expected pin ---
ACTUAL_SHA="$(sha256sum "$URCAP_SRC" | awk '{print $1}')"
if [ "$ACTUAL_SHA" != "$URCAP_SHA256" ]; then
    echo "FATAL: URCap SHA-256 mismatch" >&2
    echo "  expected: $URCAP_SHA256" >&2
    echo "  actual:   $ACTUAL_SHA" >&2
    exit 2
fi

# --- pre-build guard 3: Dockerfile present ---
if [ ! -f "${DOCKER_DIR}/Dockerfile" ]; then
    echo "FATAL: Dockerfile not found: ${DOCKER_DIR}/Dockerfile" >&2
    exit 3
fi

# --- pre-build guard 4: docker daemon reachable ---
if ! docker info >/dev/null 2>&1; then
    echo "FATAL: docker daemon not reachable (is dockerd running?)" >&2
    exit 4
fi

# --- stage URCap into build context, register cleanup trap ---
cleanup() {
    rm -f "$STAGED_URCAP"
}
trap cleanup EXIT INT TERM

cp "$URCAP_SRC" "$STAGED_URCAP"

echo "[build_ursim_image] image      : $IMAGE_TAG"
echo "[build_ursim_image] urcap      : $URCAP_NAME (35287 bytes, sha256 ${URCAP_SHA256:0:16}...)"
echo "[build_ursim_image] base digest: ${BASE_DIGEST:0:30}...764d51"
echo "[build_ursim_image] context    : ${DOCKER_DIR}"
echo

docker build \
    -t "$IMAGE_TAG" \
    -f "${DOCKER_DIR}/Dockerfile" \
    "${DOCKER_DIR}/"

echo
echo "[build_ursim_image] OK"

# --- post-build evidence ---
docker images --format 'table {{.Repository}}:{{.Tag}}\t{{.ID}}\t{{.Size}}' \
    | grep -E '^(REPOSITORY|enfield-ursim)' | head -3

# Sanity: confirm URCap landed at the correct destination inside the image
echo
echo "[build_ursim_image] sanity: URCap location inside image"
docker run --rm --entrypoint /bin/bash "$IMAGE_TAG" -c \
    'ls -la /ursim/GUI/bundle/externalcontrol-1.0.5.jar 2>&1'
