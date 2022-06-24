#! /bin/bash
# Copyright 2022 Wei-chih Lin
# Install Visual Studio code with C++ and Python support


#######################################
# Update apt source
# Globals:
#   None
# Arguments:
#   None
#######################################
function apt_update(){
     sudo apt update --fix-missing
     sudo apt upgrade
}

#######################################
# Install Visual Studio Code with extension
# Globals:
#   None
# Arguments:
#   VSCode version, please check offical site: https://code.visualstudio.com/updates/v1_68
# Output: 
#   Install status
#######################################
function vscode_install(){
     VERSION=$1
     wget -N -O vscode-linux-deb.arm64.deb https://update.code.visualstudio.com/$VERSION/linux-deb-arm64/stable
     sudo apt install ./vscode-linux-deb.arm64.deb
     code --install-extension ms-python.python \
          --install-extension ms-vscode.cpptools \
          --install-extension ms-vscode.cpptools-extension-pack \
          --install-extension ms-vscode.cpptools-themes \
          --install-extension fleexo.cpp-class-creator \
          --force
     echo "Install done!"
     echo "Also install these extension from Microsoft: Python, C/C++, C/C++ Extension Pack, C/C++ Themes"
}


#######################################
# Main function
# Globals:
#   None
# Arguments:
#   None
#######################################
function main(){
     apt_update
     vscode_install latest
     exit 0
}

# ToDo(): Start the progression 
main




