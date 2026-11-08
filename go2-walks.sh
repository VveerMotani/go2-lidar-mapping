#!/usr/bin/env bash
# Process Go2 walks on macOS or Linux, with Docker. No ROS install needed.
#
#   ./go2-walks.sh list
#   ./go2-walks.sh process Lab-loop-3.zip
#   ./go2-walks.sh map Lab-loop-3
#
# Walks live in ~/go2-walks unless GO2_WALKS says otherwise. Put the .zip from
# the robot dashboard's Download button there. --build makes the image from
# this repo instead of downloading it.

WALKS="${GO2_WALKS:-$HOME/go2-walks}"
IMAGE="${GO2_IMAGE:-ghcr.io/vveermotani/go2-lidar-mapping/go2-walks:latest}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$1" = "--build" ]; then
  shift
  IMAGE="go2-walks:local"
  docker build -t "$IMAGE" -f "$HERE/docker/Dockerfile" "$HERE" || exit $?
fi

command -v docker >/dev/null || { echo "Docker is not installed: https://docs.docker.com/get-docker/"; exit 1; }
docker info >/dev/null 2>&1 || { echo "Docker is not running. Start Docker Desktop (or the docker service) first."; exit 1; }

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Downloading the walk processor (about 2 GB, once)..."
  # Apple Silicon: the image is amd64 and runs under Docker Desktop's emulation.
  docker pull --platform linux/amd64 "$IMAGE" \
    || { echo "Could not download $IMAGE. Build it instead:  $0 --build ${*:-list}"; exit 1; }
fi

mkdir -p "$WALKS"
echo "walks folder: $WALKS"
# Run as yourself on Linux, so the results are not owned by root.
USER_FLAG=""
[ "$(uname)" = "Linux" ] && USER_FLAG="--user $(id -u):$(id -g) -e HOME=/tmp"
exec docker run --rm -it --platform linux/amd64 $USER_FLAG -v "$WALKS:/walks" "$IMAGE" "${@:-help}"
