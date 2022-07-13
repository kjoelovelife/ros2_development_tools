#!/bin/bash
# Copyright 2022 Wei-chih Lin <weichih.lin@protonmail.com>
# Apache 2.0 License 
# 
# This shell can help user to install ros2 automatically


# Global parameters
UBUNTU_DISTRO=$(grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}')
ROS_DISTRO=$(
    case ${UBUNTU_DISTRO} in
        "18.04" ) echo "no_support" ;;
        "20.04" ) echo "foxy" ;;
        "22.04" ) echo "humble" ;;
        * ) echo "humble" ;;

    esac
)

#######################################
# Check ROS2
# Globals:
#   ROS_DISTRO
# Arguments:
#   None
# Return:
#   install_status: ROS2 had installed or not
#######################################
check_ros2(){
    if [ -d "/opt/ros/${ROS_DISTRO}" ]
    then
        echo "y"
    else
        echo "n"
    fi
}

#######################################
# Install ROS2
# Globals:
#   ROS_DISTRO
# Arguments:
#   install_status: ROS2 had installed or not
#######################################
install_ros2(){
# Set Locale
    if [ "${ROS_DISTRO}" == "no_support" ]
    then
        echo -e "ROS2-${ROS_DISTRO}: no support on Ubuntu 18.04. Skip installing."
    else
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

        echo "ROS2-{ROS_DISTRO}: installed successfully"
    fi
}


#######################################
# main function
# Globals:
#   HARDWARE_PLATFORM
#   SYSTEM
#######################################
main(){
    install_status=$(check_ros2)
    if [ "$install_status" == "y" ]
    then
        echo "ROS2-${ROS_DISTRO}: skip installing, because it already exist"
    else
        install_ros2
    fi 
}

# Start progression
main