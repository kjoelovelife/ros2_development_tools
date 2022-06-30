#!/bin/bash
# Copyright 2022 Wei-chih Lin
# Apache 2.0 License 

# Install ROS 2 Foxy in Nvidia Jetson nano from source code
# Reference: https://docs.ros.org/en/foxy/Installation/Linux-Development-Setup.html
# Mostly from: 
#   https://github.com/jetsonhacks/installROS2/blob/master/installROS2.sh

ROS_DISTRO=foxy
BUILD_ROOT=/opt/ros/$ROS_DISTRO_src
INSTALL_ROOT=/opt/ros/$ROS_DISTRO



#######################################
# Update cmake version from 3.10.2 to 3.20.0
# Globals:
#   None
# Arguments:
#   install_path
#######################################
update_cmake(){
    sudo apt install -y wget checkinstall
    cd $1 && wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz
    tar -zvxf cmake-3.20.0.tar.gz
    cd $1/cmake-3.20.0 && ./bootstrap
    make -j$(nproc)
    sudo checkinstall --pkgname=cmake --pkgversion="3.20-custom" --default
    hash -r
    apt-cache policy cmake
    rm $1/cmake-3.20.0.tar.gz

}

#######################################
# Set locale
# Globals: 
#   None
# Arguments:
#   None
#######################################
set_locale(){
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
}

#######################################
# Add the ROS 2 apt repository
# Globals:
#   None
# Arguments:
#   None
#######################################
add_ros2_apt_repository(){
    sudo apt install software-properties-common
    sudo apt update && sudo apt install -y \
        curl \
        gnupg2 \
        lsb-release \
        wget
    sudo rm -rf /var/lib/apt/lists/*
    wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc 
    sudo apt-key add ros.asc
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
}

#######################################
# Install development tools and ROS tools
# Globals:
#   BUILD_ROOT
#   ROS_DISTRO
# Arguments:
#   install_path
#######################################
install_development_tools(){
    sudo apt update
    sudo apt-get install -y \
		build-essential \
		cmake \
		git \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-pytest-cov \
		python3-rosdep \
		python3-vcstool \
		python3-rosinstall-generator \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev

    # install some pip packages needed for testing
    python3 -m pip install -U \
        argcomplete \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest
    
    # Because of the error "setuptools deprecation warning, setuptools need version 58.2.0"
    python3 -m pip uninstall -y setuptools
    python3 -m pip install -U setuptools==58.2.0



    # install Fast-RTPS dependencies
    sudo apt install -y \
        libasio-dev \
        libtinyxml2-dev

    # install Cyclone DDS dependencies
    sudo apt install -y \
        libcunit1-dev

    # compile yaml-cpp-0.6, which some ROS packages may use (but is not in the 18.04 apt repo)
    git clone --branch yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp $1/yaml-cpp-0.6 && \
    cd $1/yaml-cpp-0.6 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && \
    sudo cp libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/ && \
    sudo ln -s /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6

}

#######################################
# Get ROS 2 code
# Globals:
#   ROS_ROSDISTRO
#   BULD_ROOT
# Arguments:
#   ros_pkg # Please check: https://index.ros.org/packages/
#######################################
get_ros2_code(){
    sudo mkdir -p $BUILD_ROOT/src && cd $BUILD_ROOT
    sudo sh -c "rosinstall_generator --deps --rosdistro $ROS_DISTRO $1 launch_xml launch_yaml example_interfaces > ros2.$ROS_DISTRO.$1.rosinstall"
    sudo sh -c "vcs import src < ros2.$ROS_DISTRO.$1.rosinstall"

    # download unreleased packages     
    sudo sh -c "git clone --branch ros2 https://github.com/Kukanani/vision_msgs ${BUILD_ROOT}/src/vision_msgs && \
    git clone --branch ${ROS_DISTRO} https://github.com/ros2/demos demos && \
    cp -r demos/demo_nodes_cpp ${BUILD_ROOT}/src && \
    cp -r demos/demo_nodes_py ${BUILD_ROOT}/src && \
    rm -r -f demos"

}

#######################################
# Install dependencies with rosdep
# Globals:
#   BULD_ROOT
# Arguments:
#   None
#######################################
rosdep_install(){
    sudo apt-get update
    cd $BUILD_ROOT
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers qt_gui"
    sudo rm -rf /var/lib/apt/lists/*
}


#######################################
# Build ROS 2
# Globals:
#   BULD_ROOT
# Arguments:
#   ros_distro
#######################################
build_ros2(){
    sudo mkdir -p $INSTALL_ROOT
    sudo colcon build --merge-install --install-base $INSTALL_ROOT
    sudo colcon build --merge-install --install-base $INSTALL_ROOT
    echo "source $INSTALL_ROOT/setup.bash" >> ~/.bashrc 
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
    #echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
}


#######################################
# Main function
# Globals:
#   None
# Arguments:
#   None
#######################################
main(){
    update_cmake "/home/$USER/src"
    set_locale
    add_ros2_apt_repository
    install_development_tools "/home/$USER/src"
    get_ros2_code "ros_base"
    rosdep_install
    build_ros2
    exit 0
}


# start the progress
main