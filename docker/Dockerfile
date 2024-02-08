# ROS distribution to use
ARG ROS_DISTRO=foxy


######################
# Base Image for Eel #
######################
FROM --platform=linux/amd64 ros:${ROS_DISTRO} as build-amd64
FROM --platform=linux/arm64 arm64v8/ros:${ROS_DISTRO} as build-arm64

FROM build-$BUILDARCH as base



ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-pip wget unzip && \
    rm -rf /var/lib/apt/lists/*

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install Eel specific python dependencies
RUN pip3 install --no-cache-dir \
 pynmea2 \
 pyserial \
 geopy \
 gpiozero \
 pigpio \
 RPi.GPIO \
 spidev \
 lgpio \
 adafruit-circuitpython-bno055 \
 awsiotsdk

# Install voltage sensor dependency
RUN mkdir -p /pi_ina226
WORKDIR /pi_ina226
RUN wget https://github.com/e71828/pi_ina226/archive/refs/heads/main.zip -O voltage-lib.zip
RUN unzip voltage-lib.zip
RUN pip install pi_ina226-main/

# Create Colcon workspace with external dependencies
RUN mkdir -p /eel/src
WORKDIR /eel/src

COPY ./src/ ./eel/

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /eel
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
#  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

RUN colcon test \
 && colcon test-result --verbose

# Set up the entrypoint
WORKDIR /eel
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM base as dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl && \
    rm -rf /var/lib/apt/lists/*


# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /eel/

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
