#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers
 
# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
 
# Source the base workspace, if built
if [ -f /eel/install/setup.bash ]
then
  source /eel/install/setup.bash
fi
 
# Execute the command passed into this entrypoint
exec "$@"
