#!/bin/bash
# Copyright 2022 Wei-chih Lin
# Apache 2.0 License 

# Install ROS in Nvidia Jetson family from binary distribution
# Reference: http://wiki.ros.org/melodic/Installation/Ubuntu

# Global parameters
UBUNTU_DISTRO=$(grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}')
ROS_DISTRO=$(
    case ${UBUNTU_DISTRO} in
        "20.04" ) echo "foxy" ;;
        "22.04" ) echo "humble" ;;
        * ) echo "humble" ;;

    esac
)


# Set Locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-desktop
sudo apt install -y python3-pip
pip3 install -U argcomplete
    
# Install ROS 2 RQT
sudo apt install -y ros-${ROS_DISTRO}-rqt
    
# Install turtlesim for verification
sudo apt install -y ros-${ROS_DISTRO}-turtlesim
    
# Install ROS 2 build tools
sudo apt install -y python3-colcon-common-extensions python3-vcstool

# RMW for ROS 2
sudo apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# ros1_bridge
sudo apt install -y ros-${ROS_DISTRO}-ros1-bridge

echo "ROS 2 ${ROS_DISTRO} installed successfully"
