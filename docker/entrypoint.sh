#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers
 
# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
 
# Source the base workspace, if built
if [ -f /eel/install/setup.bash ]
then
  source /eel/install/setup.bash
#   export TURTLEBOT3_MODEL=waffle_pi
#   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
fi
 
# Execute the command passed into this entrypoint
exec "$@"
