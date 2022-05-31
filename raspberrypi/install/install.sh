#!/bin/bash
#
#  Install ROS2 base on the Ubuntu system
#  Maintainer: Wei-chih, Lin
#  Email: weichih.lin@protonmail.com
#  Version: 0.0.1

#######################################
# Initialize
# Globals:
#   HARDWARE_PLATFORM
#   SYSTEM
#######################################

HARDWARE_PLATFORM=$(uname -i)
SYSTEM=$(cat /etc/issue | cut -b 8-12 | tr -d '\n')

#######################################
# Update repository
# Globals:
#   None
# Arguments:
#   None
#######################################
function update_repository(){
    sudo apt update
    sudo apt upgrade 
}

#######################################
# Set locale
# Globals:
#   None
# Arguments:
#   None
#######################################
function setup_locale(){
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings
}

#######################################
# Setup Sources
# Globals:
#   None
# Arguments:
#   None
#######################################
function setup_source(){
    #  enable the Universe repository with these instructions
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe

    # add the ROS 2 apt repository to your system
    sudo apt update && sudo apt install curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    # add the repository to your sources list.
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
}

#######################################
# Install ROS2 base on system version
# Globals:
#   None
# Arguments:
#   $1: system version
#######################################
function install_ros2(){
    version=""
    system=$1
    case "$system" in
        20.04) version=foxy
            ;;
        22.04) version=humble
            ;;
        *)  echo "Unexpected system! Please checkout your system!"
            exit 1
            ;; 
    esac
    sudo apt install -y ros-$version-desktop
    if [ -d /opt/ros/$version ]; then
        echo "--------------------"
        echo "Successfully install ROS2-$version-desktop!"
        echo "Please check the website to get more information: https://docs.ros.org/en/humble/index.html"
        echo ""
    else
        echo "Failed to install ROS2-$version! Please checkout your system!"
        echo ""
        exit 1
    fi

}

#######################################
# Sourcing the setup script
# Globals:
#   None
# Arguments:
#   None
#######################################
function environment_setup(){
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
}

# Main function
function main(){
    setup_locale
    setup_source
    update_repository
    install_ros2 $SYSTEM
    environment_setup
    exit 0
}

# Start the procession
main

