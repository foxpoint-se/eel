# ARG ROS_DISTRO=foxy
# ARG UBUNTU_DISTRO=focal

# # Base image
# FROM osrf/ros:${ROS_DISTRO}-desktop as base

# RUN apt-get update
# RUN apt-get upgrade -y
# RUN apt-get install -y python3-pip

# ENV ROS_DISTRO=${ROS_DISTRO}

# SHELL ["/bin/bash", "-c"]

# # Create Colcon workspace
# RUN mkdir -p /ros2_ws/src

# WORKDIR /ros2_ws/src

# COPY ./src/ ./

# ===========
FROM osrf/ros:foxy-desktop

# to get all packages needed to run ros2
RUN apt-get update -y
RUN apt-get install -y git wget python3-pip vim
RUN pip3 install setuptools==58.2.0

SHELL ["/bin/bash", "-c"]

# to be able to run any ros2 commands at all
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# to get ros2 auto completion in bash 
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

RUN mkdir -p /ros2_ws/src

WORKDIR /ros2_ws

COPY ./src/ /ros2_ws/src/

RUN source /opt/ros/foxy/setup.bash && colcon build

RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD source /opt/ros/foxy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run eel imu --ros-args -p simulate:=true

# WORKDIR /ros2_ws/src

# to keep container running
# CMD tail -f /dev/null