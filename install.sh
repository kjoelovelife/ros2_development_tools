#!/bin/bash

# Global Parameters
SHELL=`echo $SHELL | awk -F '/' '{print $NF}'`
UBUNTU_DISTRO=$(grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}')
OS=$(cat /etc/os-release | grep -P "^ID=" | awk -F '=' '{print $NF}')
AUTOMATED="n"
SPECIFICATION=""


#######################################
# Usage information
# Globals:
#   AUTOMATED
#######################################
usage(){
    echo "Use the command: .install.sh [OPTION] "
    echo "  --recomputerj10-ros2 -> Install ROS2-Foxy in reComputer-J10xx series"
    echo "  --vscode             -> Just install VS-Code"
    echo "  -a | --auto          -> Install all things for operating environment with ROS/ROS2"
    echo "  -h | --help          -> For help"

}



#######################################
# Check installation
# Globals:
#   AUTOMATED
#######################################
check_installation(){
    if [ "$1" == "" ]
    then
        echo -n "Do you want to install automatically? (y/N): "
        read AUTOMATED 
    else
        while [ "$1" != "" ]; do
            case $1 in
                --recomputerj10-ros2 )  SPECIFICATION="jetson_family/install_ros2_from_source.sh"
                                        ;;

                --vscode             )  SPECIFICATION="normal/install_vscode.sh"
                                        ;;

                -a | --auto )           AUTOMATED="y"
                                        ;;

                -h | --help )           usage
                                        exit
                                        ;;

                * )                     AUTOMATED="y"
                                        ;;           
            esac
            shift
        done
    fi
}

#######################################
# Check installation
# Globals:
#   SPECIFICATION
#   AUTOMATED
#######################################
install(){
    if [ "$SPECIFICATION" == "" ] 
    then
        case $AUTOMATED in 
            n ) echo "skip installing"
                ;;

            y ) . normal/install_ros.sh
                . normal/install_ros2.sh
                . normal/install_vscode.sh $OS
                ;;

            * ) echo -e "\nLike you want specific installation."
                usage
                ;;
        esac
    else
        . $SPECIFICATION $OS
    fi

}


#######################################
# main function
# Globals:
#   None
# Arguments:
#   default variable
#######################################
main(){
    echo ""
    check_installation $@
    install
    echo ""
    exit
}

# Start progress
main $@
