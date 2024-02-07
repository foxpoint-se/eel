#!/bin/bash

allowed_ros_distros=("foxy" "humble")

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <ros-distro>"
    exit 1
fi

ros_distro=$1
if [[ ! " ${allowed_ros_distros[@]} " =~ " ${ros_distro} " ]]; then
    echo "Error: Invalid ROS distro. Allowed values: ${allowed_ros_distros[@]}"
    exit 1
fi

echo "Using ROS distro: $ros_distro"

tag=$ros_distro
full_command="docker build --build-arg ROS_DISTRO=$ros_distro --target base -t foxpoint/eel:$tag -f ./Dockerfile .."
echo "Building Docker image"
echo $full_command

$full_command
