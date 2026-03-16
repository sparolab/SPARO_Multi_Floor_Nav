#!/usr/bin/env bash
set -e

IMAGE_NAME="sparolab/mfnav:latest"
CONTAINER_NAME="mfnav_container"
WORKSPACE_DIR="${HOME}/test_ws"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker run -itd --privileged --gpus all --network host \
  --name ${CONTAINER_NAME} \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --env XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/machine-id:/etc/machine-id \
  -v /usr/lib/nvidia:/usr/lib/nvidia \
  -v /tmp/runtime-root:/tmp/runtime-root \
  -v /usr/lib/x86_64-linux-gnu/libGL.so.1:/usr/lib/x86_64-linux-gnu/libGL.so.1 \
  -v "${PROJECT_ROOT}:/home/test_ws" \
  -v /dev:/dev \
  -w /home/test_ws \
  ${IMAGE_NAME}