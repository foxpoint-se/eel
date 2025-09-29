#!/usr/bin/env bash
set -e

# Usage: ./build-image2.sh <distro> [--multiarch]
# Example: ./build-image2.sh humble
#          ./build-image2.sh jazzy --multiarch

if [ -z "$1" ]; then
  echo "Usage: $0 <ros_distro> [--multiarch]"
  exit 1
fi

ROS_DISTRO="$1"
IMAGE_NAME="myimage:${ROS_DISTRO}"
MULTIARCH=false

if [ "$2" == "--multiarch" ]; then
  MULTIARCH=true
fi

# Ensure buildx builder exists
if ! docker buildx inspect multiarch-builder > /dev/null 2>&1; then
  docker buildx create --name multiarch-builder --use
else
  docker buildx use multiarch-builder
fi

if [ "$MULTIARCH" = true ]; then
  echo "Building multi-arch image for amd64 and arm64, will push to registry..."
  docker buildx build \
    --platform linux/amd64,linux/arm64 \
    --build-arg ROS_DISTRO=${ROS_DISTRO} \
    -f docker/Dockerfile.ros \
    -t ${IMAGE_NAME} \
    --push \
    .
  echo "\nBuilt and pushed: ${IMAGE_NAME} (multi-arch)"
else
  # Detect current platform
  ARCH=$(uname -m)
  if [ "$ARCH" = "x86_64" ]; then
    PLATFORM="linux/amd64"
  elif [[ "$ARCH" == "arm"* || "$ARCH" == "aarch64" ]]; then
    PLATFORM="linux/arm64"
  else
    echo "Unsupported architecture: $ARCH"
    exit 1
  fi
  echo "Building for current platform: $PLATFORM (local load)"
  docker buildx build \
    --platform $PLATFORM \
    --build-arg ROS_DISTRO=${ROS_DISTRO} \
    -f docker/Dockerfile.ros \
    -t ${IMAGE_NAME} \
    --load \
    .
  echo "\nBuilt and loaded locally: ${IMAGE_NAME} ($PLATFORM)"
fi 