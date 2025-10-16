#!/bin/bash
set -e

# Source ROS setup
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Source workspace install setup if it exists
if [ -f "/eel/install/setup.bash" ]; then
  source "/eel/install/setup.bash"
fi

exec "$@"
