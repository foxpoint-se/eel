#!/bin/bash

# detect if using zsh or bash, for sourcing correct files.
if [ -n "$ZSH_VERSION" ]; then
   FILE_EXT="zsh"
elif [ -n "$BASH_VERSION" ]; then
   FILE_EXT="bash"
else
   echo "Cannot source files. Could not detect if bash or zsh :("
   exit 1
fi

echo "Using $FILE_EXT"

# Auto-detect or use ROS_DISTRO
if [ -z "$ROS_DISTRO" ]; then
  # Try to find the latest installed ROS distro
  ROS_DISTRO=$(ls /opt/ros | sort | tail -n 1)
  echo "Auto-detected ROS_DISTRO: $ROS_DISTRO"
fi

# Source ROS
echo "Sourcing ROS $ROS_DISTRO"
if [ -f /opt/ros/$ROS_DISTRO/setup.$FILE_EXT ]; then
  source /opt/ros/$ROS_DISTRO/setup.$FILE_EXT
else
  echo "Could not find /opt/ros/$ROS_DISTRO/setup.$FILE_EXT"
  return 1 2>/dev/null || exit 1
fi

# Source workspace
echo "Sourcing local installation $(pwd)/install/setup.$FILE_EXT"
if [ -f "$(pwd)/install/setup.$FILE_EXT" ]; then
  source "$(pwd)/install/setup.$FILE_EXT"
fi

# Colcon argcomplete (optional)
echo "Sourcing colcon argcomplete"
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.$FILE_EXT ]; then
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.$FILE_EXT
fi

echo "Environment ready!"