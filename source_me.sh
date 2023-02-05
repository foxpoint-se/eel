#!/bin/bash

# detect if using zsh or bash, for sourcing correct files.
if [ -n "$ZSH_VERSION" ]; then
   echo "Using zsh"
   FILE_EXT="zsh"
elif [ -n "$BASH_VERSION" ]; then
   # assume Bash
   echo "Using bash"
   FILE_EXT="bash"
else
   # assume something else
   echo "Cannot source files. Could not detect if bash or zsh :("
   exit 1
fi

echo "Sourcing ROS files..."

# this file should always be sourced when running ros2
source $(echo $PWD)/install/setup.$(echo $FILE_EXT) || echo -e "\nHave you run 'colcon build'?"

# source python virtual env to isolate dependencies
source .venv/bin/activate

# source ROS to be able to use it
source /opt/ros/foxy/setup.$(echo $FILE_EXT)

# colcon
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.$(echo $FILE_EXT)

# add site-packages from python venv to PYTHONPATH so that ros sees them
PYTHON_VERSION=$(python -c"import sys; print(str(sys.version_info.major) + '.' + str(sys.version_info.minor))")
LOCAL_PYTHON_PACKAGES=".venv/lib/python$PYTHON_VERSION/site-packages"
export PYTHONPATH=$LOCAL_PYTHON_PACKAGES:$PYTHONPATH

echo "Done!"