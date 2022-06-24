#! /bin/bash
# Copyright 2022 Wei-chih Lin
# ROS 2 Humble in Nvidia Jetson nano with Yocto

#######################################
# Create folder as specific path
# Globals:
#   None
# Arguments:
#   None
# Returns:
#   absolute path of ~/yocto
#######################################
create_folder(){
    if [ ! -d "$HOME/yocto" ]
    then
        mkdir $HOME/yocto   
    fi
    echo "$HOME/yocto"
}

#######################################
# Install dependencies via apt
# Globals:
#   None
# Arguments:
#   None
#######################################
install_dependencies(){
    sudo apt update
    sudo apt install -y \
        chrpath \
        gawk \
        liblz4-tool \
        zstd
}

#######################################
# Fetch meta-tegra BSP and checkout Honister's branch
# Globals:
#   None
# Arguments:
#   install path
#   branch name of https://github.com/OE4T/tegra-demo-distro
# Returns:
#   absolute path of tegr-bsp-$1
#######################################
fetch_bsp(){
    #git clone https://github.com/OE4T/tegra-demo-distro -b $2 $1/tegra-bsp-$2
    #cd $1/tegra-bsp-$2 && git submodule update --init
    echo $1/tegra-bsp-$2
}

#######################################
# Add ROS 2 Humble meta layer
# Globals:
#   None
# Arguments:
#   install path
#   branch name of https://github.com/OE4T/tegra-demo-distro
#   ROS 2 version
#   absolute path of tegra-bsp-$1
#######################################
add_ros2_meta_layer(){
    git clone https://github.com/vmayoral/meta-ros -b $2-$3 $1/repos/meta-ros
    ln -s $1/repos/meta-ros $4/layers/meta-ros
}

#######################################
# Build basic example BSP for NVIDIA Jetson Nano
# Globals:
#   None
# Arguments:
#   install path
#   absolute path of tegra-bsp-$branch_name
#######################################
build_basic_bsp_example(){
    cd $2 && . setup-env \
        --machine jetson-nano-devkit \
        --distro tegrademo build 
    bitbake demo-image-full  # get the basic demo image building

}


#######################################
# Add meta-layers for ROS 2 $VERSION and configure them in Yocto/PetaLinux
# Globals:
#   None
# Arguments:
#   install path
#   branch name of https://github.com/OE4T/tegra-demo-distro
#   ROS 2 version  
#   absolute path of tegra-bsp-$branch_name
#######################################
add_meta_layers_in_peta_linux(){
   sed -i '$i\ \ '"$4"'/layers/meta-ros/meta-ros2\ \\\n\ \ '"$4"'/layers/meta-ros/meta-ros2-'"$3"'\ \\\n\ \ '"$4"'/layers/meta-ros/meta-ros-common\ \\' \
     $1/build/conf/bblayers.conf
   sed -i '4 i \\n#\ define\ the\ ROS\ 2\ Yocto\ target\ release\nROS_OE_RELEASE_SERIES = "'"$2"'"\n\n#\ define\ ROS\ 2\ distro\nROS_DISTRO = "'"$3"'"\n' \
     $1/build/conf/bblayers.conf
}

#######################################
# Create a Yocto recipe image including ROS 2 Humble
# Globals:
#   None
# Arguments:
#   branch name of https://github.com/OE4T/tegra-demo-distro
#   absolute path of tegra-bsp-$branch_name
#######################################
create_yocto_recipe_image(){
    cp $(pwd)/yocto_recipe_images/demo-image-ros2.bb $2/layers/meta-tegrademo/recipes-demo/images/
    sed -i 's/layers\/meta-tegrademo\/recipes-demo\/images/\/home\/'"$USER"'\/yocto\/tegra-bsp-'"$1"'\/layers\/meta-tegrademo\/recipes-demo\/images/g' \
      $2/layers/meta-tegrademo/recipes-demo/images//demo-image-ros2.bb
}

#######################################
# Build the image
# Globals:
#   None
# Arguments:
#   absolute path of tegra-bsp-$branch_name
#######################################
build_ros2_image(){
    . $1/setup-env \
        --machine jetson-nano-devkit \
        --distro tegrademo build 
    bitbake demo-image-ros2
}

#######################################
# Main function
# Globals:
#   None
# Arguments:
#   install path
#   branch name of https://github.com/OE4T/tegra-demo-distro
#   ROS 2 version
#######################################
main(){
    #install_dependencies
    tegra_bsp_path=$(fetch_bsp $1 $2)
    #add_ros2_meta_layer $1 $2 $3 $tegra_bsp_path
    build_basic_bsp_example $1 $tegra_bsp_path
    #add_meta_layers_in_peta_linux $1 $2 $3 $tegra_bsp_path
    #create_yocto_recipe_image $2 $tegra_bsp_path
    #build_ros2_image $tegra_bsp_path
    exit 0
}

# TODO(main): Start the progerssion
current_path=$(pwd)
install_path=$(create_folder) 
branch_name="honister"
ros2_version="humble"
main $install_path $branch_name $ros2_version