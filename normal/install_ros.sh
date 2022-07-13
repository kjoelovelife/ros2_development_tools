#!/bin/bash
# Copyright 2022 Wei-chih Lin <weichih.lin@protonmail.com>
# Apache 2.0 License 
# 
# This shell can help user to install ros automatically


# Global parameters
UBUNTU_DISTRO=$(grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}')
ROS_DISTRO=$(
    case ${UBUNTU_DISTRO} in
        "18.04" ) echo "melodic" ;;
        "20.04" ) echo "noetic" ;;
        * ) echo "noetic" ;;

    esac
)

#######################################
# Check ROS
# Globals:
#   ROS_DISTRO
# Arguments:
#   None
# Return:
#   install_status: ROS2 had installed or not
#######################################
check_ros(){
    if [ -d "/opt/ros/${ROS_DISTRO}" ]
    then
        echo "y"
    else
        echo "n"
    fi
}

#######################################
# Install ROS
# Globals:
#   ROS_DISTRO
# Arguments:
#   install_status: ROS2 had installed or not
#######################################
install_ros(){
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
    sudo apt install -y ros-${ROS_DISTRO}-ros1-bridge

    echo "ROS-${ROS_DISTRO}: installed successfully"
}


#######################################
# main function
# Globals:
#   ROS_DISTRO
#######################################
main(){
    install_status=$(check_ros)
    if [ "${install_status}" == "y" ]
    then
        echo -e "ROS-${ROS_DISTRO}: skip installing, because it has already exist."
    else
        install_ros
    fi 
}

# Start progression
main