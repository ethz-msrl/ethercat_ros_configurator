# EtherCAT ROS Configurator

This package acts as a configurator for EtherCAT device SDKs built on top of the [ethercat_sdk_master](https://github.com/leggedrobotics/ethercat_sdk_master) package. This package provides a seamless software architecture to add new ethercat slave device classes and configure them with a ROS backend for easy integration and multhreading for asynchronous update of the command and feedback data through ROS messages. Furthermore, this package provides a simple way to configure an EtherCAT network with multiple slave devices through a single yaml configuration file.

# Table of Contents

- [EtherCAT ROS Configurator](#ethercat-ros-configurator)
- [Table of Contents](#table-of-contents)
- [Installation](#installation)
  - [Step 1: Installing Dependencies](#step-1-installing-dependencies)
    - [ROS Noetic](#ros-noetic)
    - [Catkin Tools](#catkin-tools)
    - [Message Logger](#message-logger)
    - [SOEM Interface](#soem-interface)
    - [EtherCAT SDK Master](#ethercat-sdk-master)
    - [EtherCAT Motor Messages](#ethercat-motor-messages)
    - [Yaml-CPP](#yaml-cpp)
  - [Step 2: Install Device SDKs](#step-2-install-device-sdks)
    - [Supported Device SDKs](#supported-device-sdks)
- [How to use ?](#how-to-use-)
- [How it works ?](#how-it-works-)
- [API Documentation](#api-documentation)


# Installation

This package is supposed to be used as a ROS package for a catkin work space. Therefore, knowledge of the catkin build system and workspaces is assumed for the rest of the sections. If you are not familiar with catkin, please refer to the [ROS wiki](http://wiki.ros.org/catkin) and [Catkin Tutorials](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for more information. Note that the package is tested under the following conditions:
- Ubuntu Focal (20.04 LTS)
- ROS Noetic
- `catkin build` as the build system for the catkin workspace. Requires catkin_tools.

## Step 1: Installing Dependencies

The following packages are required to be installed in your system in order to use this package:
- [Ubuntu Focal (20.04 LTS)](https://releases.ubuntu.com/20.04/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
- [ethercat_sdk_master](https://github.com/leggedrobotics/ethercat_sdk_master/tree/6b420bc1785cf26324aab62c79347b2a6e07924d)
- [soem_interface](https://github.com/leggedrobotics/soem_interface/tree/7b7bed29d8dfe1d0ef17c8e42a4aab07b6b393df)
- [message_logger](https://github.com/leggedrobotics/message_logger/tree/bdb867e57059f21d22f9454f6910920bfe5caac2)
- [ethercat_motor_msgs](Dead link, will be updated soon)
- yaml-cpp (Ubuntu 20.04 LTS system installed for [version 0.6](https://packages.ubuntu.com/focal/libyaml-cpp0.6))

### ROS Noetic
For installing ROS Noetic in Ubuntu 20.04 LTS, please refer to the [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu). The packages have not been tested for ROS installations in other environments like WSL, Raspberry Pi, etc, therefore, user discretion is advised. Feel free to open an issue if you encounter any problems with other distributions, but receiving support for other distributions is not guaranteed.
### Catkin Tools
To use the `catkin build` command, you need to install the `catkin_tools` package. You can install it by running the following command:
```bash
sudo pip3 install -U catkin_tools
```
For more information and alternative installation methods, please refer to the [catkin_tools Installation Guide](https://catkin-tools.readthedocs.io/en/latest/installing.html).

### Message Logger

The message logger package provides a simple way to log messages to the console and to a file. Add the package to your workspace by running the following commands:
```bash
cd /path/to/your/catkin/workspace/src
git clone https://github.com/leggedrobotics/message_logger.git
```

For the specific version of the package used while testing alongwith the installation instructions, goto [message_logger](https://github.com/leggedrobotics/message_logger/tree/bdb867e57059f21d22f9454f6910920bfe5caac2). At the time of writing this readme, the latest version of the package should work, if it fails, checkout the specific commit hash in the link after cloning the repository.
 
### SOEM Interface
The SOEM Interface package provides a ROS wrapper for the [SOEM Library](https://github.com/OpenEtherCATsociety/SOEM.git). Add the package to your workspace by running the following commands:
```bash
cd /path/to/your/catkin/workspace/src
git clone https://github.com/leggedrobotics/soem_interface.git
```
The specific version of the package used while testing can be found at [soem_interface](https://github.com/leggedrobotics/soem_interface/tree/7b7bed29d8dfe1d0ef17c8e42a4aab07b6b393df). At the time of writing this readme, the latest version of the package should work, if it fails, checkout the specific commit hash in the link after cloning the repository.

### EtherCAT SDK Master
This package presents a nice base class for implementation of EtherCAT slave devices through the [SOEM Library](https://github.com/OpenEtherCATsociety/SOEM.git). Add the package to your workspace by running the following commands:
```bash
cd /path/to/your/catkin/workspace/src
git clone https://github.com/leggedrobotics/ethercat_sdk_master.git
```

The specific version of the package used while testing can be found at [ethercat_sdk_master](https://github.com/leggedrobotics/ethercat_sdk_master/tree/6b420bc1785cf26324aab62c79347b2a6e07924d). At the time of writing this readme, the latest version of the package should work, if it fails, checkout the specific commit hash in the link after cloning the repository.

### EtherCAT Motor Messages
The EtherCAT Motor Messages package provides a set of ROS messages for EtherCAT motor controllers. Add the package to your workspace by running the following commands:
```bash
cd /path/to/your/catkin/workspace/src
git clone <link to the github page for ethercat_motor_msgs>
```

### Yaml-CPP
The package uses the yaml-cpp library for parsing the YAML configuration files. The library is installed by default in Ubuntu 20.04 LTS. However, you should be able to install it by running the following command:
```bash
sudo apt-get install libyaml-cpp0.6
```

## Step 2: Install Device SDKs
Several supported device SDKs are available for use with this package. The device SDKs are built on top of the `ethercat_sdk_master` package. The device SDKs are available as separate packages and can be installed by following the instructions in the respective package's readme. Installing them just boils down to cloning the respective device SDK's repository and installing any dependencies mentioned in their installation instructions. Make sure that the device SDKs can be found by the cmake [`find_package()`](https://cmake.org/cmake/help/latest/command/find_package.html#command:find_package) command in `CMakeLists.txt`.

The following device SDKs are supported:



### Supported Device SDKs

# How to use ?

# How it works ?

# API Documentation

