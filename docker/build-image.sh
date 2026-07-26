#!/usr/bin/env bash
set -e

# Usage: ./build-image.sh <distro> [--multiarch | --ci]
# Example: ./build-image.sh humble
#          ./build-image.sh jazzy --multiarch
#          ./build-image.sh humble --ci    # CI: multi-arch verify, no --load

if [ -z "$1" ]; then
  echo "Usage: $0 <ros_distro> [--multiarch | --ci]"
  exit 1
fi

ROS_DISTRO="$1"
IMAGE_NAME="foxpoint/eel:${ROS_DISTRO}"
MODE="local"

if [ "${2:-}" = "--multiarch" ]; then
  MODE="multiarch"
elif [ "${2:-}" = "--ci" ]; then
  MODE="ci"
elif [ -n "${2:-}" ]; then
  echo "Unknown option: $2"
  echo "Usage: $0 <ros_distro> [--multiarch | --ci]"
  exit 1
fi

if [ "$ROS_DISTRO" = "humble" ]; then
  DOCKERFILE="./Dockerfile.ros.humble"
elif [ "$ROS_DISTRO" = "jazzy" ]; then
  DOCKERFILE="./Dockerfile.ros.jazzy"
elif [ "$ROS_DISTRO" = "lyrical" ]; then
  DOCKERFILE="./Dockerfile.ros.lyrical"
else
  echo "Unsupported ROS distro: $ROS_DISTRO"
  exit 1
fi

ensure_buildx_builder() {
  if ! docker buildx inspect multiarch-builder > /dev/null 2>&1; then
    docker buildx create --name multiarch-builder --use
  else
    docker buildx use multiarch-builder
  fi
}

if [ "$MODE" = "ci" ]; then
  echo "CI verify build for linux/amd64 and linux/arm64 (no local load)..."
  docker buildx build \
    --platform linux/amd64,linux/arm64 \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    -f "${DOCKERFILE}" \
    -t "${IMAGE_NAME}" \
    ..
  echo "CI verify build succeeded: ${IMAGE_NAME}"
  exit 0
fi

ensure_buildx_builder

if [ "$MODE" = "multiarch" ]; then
  echo "Building multi-arch image for amd64 and arm64..."
  docker buildx build \
    --platform linux/amd64,linux/arm64 \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    -f "${DOCKERFILE}" \
    -t "${IMAGE_NAME}" \
    --load \
    ..
  echo "Built: ${IMAGE_NAME} (multi-arch)"
else
  ARCH=$(uname -m)
  if [ "$ARCH" = "x86_64" ]; then
    PLATFORM="linux/amd64"
  elif [[ "$ARCH" == "arm"* || "$ARCH" == "aarch64" ]]; then
    PLATFORM="linux/arm64"
  else
    echo "Unsupported architecture: $ARCH"
    exit 1
  fi
  echo "Building for current platform: ${PLATFORM} (local load)"
  docker buildx build \
    --platform "${PLATFORM}" \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    -f "${DOCKERFILE}" \
    -t "${IMAGE_NAME}" \
    --load \
    ..
  echo "Built and loaded locally: ${IMAGE_NAME} (${PLATFORM})"
fi
