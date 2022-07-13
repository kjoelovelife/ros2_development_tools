#! /bin/bash
# Copyright 2022 Wei-chih Lin
# Install Visual Studio code with C++ and Python support


# Global parameters
ARCHITECTURE=$(dpkg --print-architecture)


#######################################
# Update apt source
# Globals:
#   ARCHITECTURE
# Arguments:
#   None
#######################################
apt_update(){

     sudo apt update

     if [ "${ARCHITECTURE}" == "amd64" ]
     then
          sudo apt install software-properties-common apt-transport-https wget -y
          curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo gpg --dearmor -o /usr/share/keyrings/ms-vscode-keyring.gpg
          echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ms-vscode-keyring.gpg] https://packages.microsoft.com/repos/vscode stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
     fi

     sudo apt update --fix-missing
     sudo apt upgrade
}

#######################################
# Install VS code in Ubuntu
# Globals:
#   ARCHITECTURE
# Arguments:
#   VSCode version, please check offical site: https://code.visualstudio.com/updates/v1_68
#######################################
install_with_ubuntu(){
     case $ARCHITECTURE in 

     amd64 )  sudo apt install -y code          
              ;;

     arm64 )  wget -N -O normal/vscode-linux-deb.arm64.deb https://update.code.visualstudio.com/$1/linux-deb-arm64/stable
              sudo apt install normal/vscode-linux-deb.arm64.deb
              ;;

     * )      echo "This project no support for structure $ARCHITECTURE witn Ubuntu."
              ;;

     esac
}

#######################################
# Install Visual Studio Code with extension
# Globals:
#   None
# Arguments:
#   operating system  
# Output: 
#   Install status
#######################################
vscode_install(){
     case $1 in

          ubuntu )    install_with_ubuntu latest
                      ;;

          raspbian )  sudo apt install code
                      ;;

          * )         echo "Error! Please check system(support for ubuntu or raspbian)."
                      ;;
     esac

     if [ -n $(which code) ]
     then
          code --install-extension ms-python.python \
          --install-extension ms-vscode.cpptools \
          --install-extension ms-vscode.cpptools-extension-pack \
          --install-extension ms-vscode.cpptools-themes \
          --install-extension fleexo.cpp-class-creator \
          --force
          echo "Install done!"
          echo "Also install these extension from Microsoft: Python, C/C++, C/C++ Extension Pack, C/C++ Themes"
     else
          echo "VS Code: faild to install."
     fi


}


#######################################
# Main function
# Globals:
#   None
# Arguments:
#   operating system
#######################################
main(){

     if [ -n $(which code) ] 
     then
          echo "VS Code: skip installing, because it already exist."
     else
          apt_update
          vscode_install $1
     fi

}

# ToDo(): Start the progression 
main $@




