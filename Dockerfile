FROM osrf/ros:humble-desktop-jammy

# USE BASH
SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

##### Setup Gazebo #####
# essential install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    binutils \
    mesa-utils \
    kmod \
    x-window-system \
    curl

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN curl -sSL http://get.gazebosim.org | sh

# package dependencies
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-apriltag-msgs \
    ros-humble-apriltag \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    libopencv-dev

# personal packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
   vim \
   gdb

RUN echo "source '/opt/ros/humble/setup.bash'" >> ~/.bashrc \
    && echo "source '/usr/share/gazebo/setup.sh'" >> ~/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/dev_ws/src/simulation_launch/models/" >> ~/.bashrc \
    && echo "alias b='source /home/dev_ws/install/local_setup.bash'" >> ~/.bashrc

WORKDIR /home/dev_ws