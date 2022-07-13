# Useful tools for ROS and ROS2
![LICENSE](https://img.shields.io/badge/license-Apache%202.0-blue)
![LICENSE](https://img.shields.io/badge/version-v1.0.0-success)


This project is about setting operating environment for ROS/ROS2 on specific SBC/platforms quickly. 


| SBC/platform | Test status |
| ------------ |:-----------:|
| [Raspberry pi 4B](https://www.icshop.com.tw/product-page.php?27538)                                 |   &#9745;   |
| [Jetson Nano Developer Kit](https://www.icshop.com.tw/product-page.php?27812)                       |   &#9745;   |
| [reComputer-J1010](https://www.icshop.com.tw/product-page.php?28703)                                |   &#9745;   |
| [Adlink I-Pi SMARC Kit](https://www.adlinktech.com/en/ipi-smarc-devkit-for-industrial-applications) |   &#9745;   |

And now can install tools below:

| Tools    | Content |
| -------- | -------- |
| Framwork | [ROS](http://wiki.ros.org/ROS/Installation), [ROS2](https://docs.ros.org/en/humble/Installation.html)|
| [IDE](https://en.wikipedia.org/wiki/Integrated_development_environment)| [VSCode](https://code.visualstudio.com/)|

## Download this project

Make sure the tool "git" in the SBC user use, please type the command below to install git:

```bash=
$ sudo apt install git
```

After installing git, use can type the command below to download this project:

```bash=
$ mkdir -p ~/src
$ git clone https://github.com/kjoelovelife/ros2_development_tools ~/src
```


## Usage

First time use this procject, user can type the command below in terminal:

```bash=
$ ./install.sh -h
```

![Ask for help](https://i.imgur.com/Zu2394V.png)

The default option is "-a" that means user can install automatically with the scripts. Use the command below to install automatically:

```bash=
$ ./install.sh
```

The image shown below represent all things in this project had installed.

![Run install automatically](https://i.imgur.com/xefaTH1.png)




## Next

Please feel free to check the project: [ROSKY2](https://github.com/CIRCUSPi/ROSKY2/wiki)

Or if user want to add new tools in this project, please don't 
hesitate to contact me :)





