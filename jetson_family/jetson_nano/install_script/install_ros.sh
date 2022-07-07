#!/bin/bash
# Copyright 2022 Wei-chih Lin
# Apache 2.0 License 

# Install ROS in Nvidia Jetson family from binary distribution
# Reference: http://wiki.ros.org/melodic/Installation/Ubuntu

# Global parameters
UBUNTU_DISTRO=$(grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}')
ROS_DISTRO=$(
    case ${UBUNTU_DISTRO} in
        "18.04" ) echo "melodic" ;;
        "20.04" ) echo "noetic" ;;
        * ) echo noetic ;;

    esac
)


# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installation
sudo apt update
sudo apt install ros-${ROS_DISTRO}-desktop-full

# Dependencies for building packages
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# Initialize rosdep
sudo apt install python-rosdep
sudo rosdep init
rosdep update
