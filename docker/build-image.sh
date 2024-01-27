#!/bin/bash

allowed_ros_distros=("foxy" "humble")
allowed_architectures=("amd64" "arm64")

current_architecture=$(dpkg --print-architecture)

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <architecture> <ros-distro>"
    exit 1
fi

architecture=$1
if [[ ! " ${allowed_architectures[@]} " =~ " ${architecture} " ]]; then
    echo "Error: Invalid architecture. Allowed values: ${allowed_architectures[@]}"
    echo "Your current architeture is $current_architecture so you probably want to use that."
    exit 1
fi

ros_distro=$2
if [[ ! " ${allowed_ros_distros[@]} " =~ " ${ros_distro} " ]]; then
    echo "Error: Invalid ROS distro. Allowed values: ${allowed_ros_distros[@]}"
    exit 1
fi

echo "Building image for architecture: $architecture"
echo "Using ROS distro: $ros_distro"

declare -A arch_prefixes
arch_prefixes["amd64"]=""
arch_prefixes["arm64"]="arm64v8/"

arch_prefix=${arch_prefixes["$architecture"]}

tag=$ros_distro-$architecture
full_command="docker build --build-arg ARCH_PREFIX=$arch_prefix --build-arg ROS_DISTRO=$ros_distro --target base -t foxpoint/eel:$tag -f ./Dockerfile .."
echo "Building Docker image"
echo $full_command

$full_command

# docker build --platform linux/amd64 --build-arg ROS_DISTRO=$ros_distro --target base -t foxpoint/eel:test -f ./Dockerfile ..
