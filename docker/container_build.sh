#!/usr/bin/env bash
set -e

IMAGE_NAME="sparolab/mfnav:latest"

docker build -t ${IMAGE_NAME} .