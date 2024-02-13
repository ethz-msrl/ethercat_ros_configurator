# EtherCAT ROS Configurator

This package acts as a configurator for EtherCAT device SDKs built on top of the (ethercat_sdk_master)[https://github.com/leggedrobotics/ethercat_sdk_master] package. This package provides a seamless software architecture to add new ethercat slave device classes and configure them with a ROS backend for easy integration and multhreading for asynchronous update of the command and feedback data through ROS messages. Furthermore, this package provides a simple way to configure an EtherCAT network with multiple slave devices through a single yaml configuration file.

# Installation

This package is supposed to be used as a ROS package for a catkin work space. Therefore, knowledge of the catkin build system and workspaces is assumed for the rest of the sections. If you are not familiar with catkin, please refer to the [ROS wiki](http://wiki.ros.org/catkin) and [Catkin Tutorials](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for more information. Note that the package is tested under the following conditions:
- Ubuntu Focal (20.04 LTS)
- ROS Noetic
- `catkin build` as the build system for the catkin workspace. Requires catkin_tools.

## Installing Dependencies

The following packages are required to be installed in your system in order to use this package:
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
- [ethercat_sdk_master](https://github.com/leggedrobotics/ethercat_sdk_master/tree/6b420bc1785cf26324aab62c79347b2a6e07924d)
- [soem_interface](https://github.com/leggedrobotics/soem_interface/tree/7b7bed29d8dfe1d0ef17c8e42a4aab07b6b393df)
- [message_logger](https://github.com/leggedrobotics/message_logger/tree/bdb867e57059f21d22f9454f6910920bfe5caac2)
- yaml-cpp (Ubuntu 20.04 LTS system installed for [version 0.6](https://packages.ubuntu.com/focal/libyaml-cpp0.6))

### ROS Noetic

### Catkin Tools
To use the `catkin build` command, you need to install the `catkin_tools` package. You can install it by running the following command:
```bash
sudo pip3 install -U catkin_tools
```
For more information and alternative installation methods, please refer to the [catkin_tools Installation Guide](https://catkin-tools.readthedocs.io/en/latest/installing.html).

## Supported Device SDKs

# How to use ?

# How it works ?

# API Documentation

