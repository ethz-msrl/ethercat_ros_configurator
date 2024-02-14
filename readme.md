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
  - [Step 3: Install EtherCAT ROS Configurator](#step-3-install-ethercat-ros-configurator)
- [Usage Instructions](#usage-instructions)
    - [`ethercat_master` Section](#ethercat_master-section)
    - [`ethercat_devices` Section](#ethercat_devices-section)
    - [Important Note On EtherCAT Address](#important-note-on-ethercat-address)
    - [Command and Feedback Data](#command-and-feedback-data)
    - [Wiring the EtherCAT Network](#wiring-the-ethercat-network)
    - [Running the ROS Node](#running-the-ros-node)
- [API Documentation and Implementation Details](#api-documentation-and-implementation-details)


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
This package presents a nice base class for implementation of EtherCAT slave devices by providing high level EtherCAT functionalities over the [SOEM Library](https://github.com/OpenEtherCATsociety/SOEM.git). Add the package to your workspace by running the following commands:
```bash
cd /path/to/your/catkin/workspace/src
git clone https://github.com/leggedrobotics/ethercat_sdk_master.git
```

The specific version of the package used while testing can be found at [ethercat_sdk_master](https://github.com/leggedrobotics/ethercat_sdk_master/tree/6b420bc1785cf26324aab62c79347b2a6e07924d). At the time of writing this readme, the latest version of the package should work, if it fails, checkout the specific commit hash in the link after cloning the repository.

### EtherCAT Motor Messages
The [EtherCAT Motor Messages]() package provides a set of ROS messages for EtherCAT motor controllers. Add the package to your workspace by running the following commands:
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
Several supported device SDKs are available for use with this package. The device SDKs are built on top of the `ethercat_sdk_master` package. The device SDKs are available as separate packages and can be installed by following the instructions in the respective package's readme. Installing them usually just boils down to cloning the respective device SDK's repository and installing any dependencies mentioned in their installation instructions. Make sure that the device SDKs can be found by the cmake [`find_package()`](https://cmake.org/cmake/help/latest/command/find_package.html#command:find_package) command in `CMakeLists.txt`.


### Supported Device SDKs
The following device SDKs are supported:

| SDK Name    | URL    | Type    | Description    | License | Registration Name |
|---------------- | --------------- | --------------- | --------------- | --------------- | --------------- |
| Nanotec EtherCAT SDK | [Insert URL here] | Motor Controller | Designed for Nanotec C5E-1-21 motor controller | [Insert License here] | Nanotec |
| Maxon EPOS EtherCAT SDK | [Insert URL here] | Motor Controller | Designed for Maxon EPOS4 motor controller | [Insert License here] | Maxon |

This list will be updated as more device SDKs are added to the package. Follow the installation instructions in the respective device SDK's readme to add it to your catkin workspace. Note that the package only compiles the class object for a device if its SDK is found by cmake. Check out the warning messages in the build log to see which SDKs were found by cmake. For example: if the Nanotec and the Maxon SDKs are found, the following is the output of the build log:
```bash
Starting  >>> ethercat_ros_configurator                                                                                                                                        
_______________________________________________________________________________________________________________________________________________________________________________
Warnings   << ethercat_ros_configurator:cmake /home/neelaksh/ethercat_ws/logs/ethercat_ros_configurator/build.cmake.000.log                                                    
Found EtherCAT device sdk: maxon_epos_ethercat_sdk
Found EtherCAT device sdk: nanotec_ethercat_sdk
cd /home/neelaksh/ethercat_ws/build/ethercat_ros_configurator; catkin build --get-env ethercat_ros_configurator | catkin env -si  /usr/bin/cmake /home/neelaksh/ethercat_ws/src/ethercat_ros_configurator --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/neelaksh/ethercat_ws/devel/.private/ethercat_ros_configurator -DCMAKE_INSTALL_PREFIX=/home/neelaksh/ethercat_ws/install; cd -

...............................................................................................................................................................................
_______________________________________________________________________________________________________________________________________________________________________________

```

## Step 3: Install EtherCAT ROS Configurator
Finally, clone this repository into your catkin workspace and build the workspace. You can do this by running the following commands:
```bash
cd /path/to/your/catkin/workspace/src
git clone <link to this repo>
cd ..
catkin build
```

# Usage Instructions
Before following the instructions below, make sure you have built your catkin workspace alongwith the SDKs of all the EtherCAT devices you intend to use. The following steps will guide you through the process of using the package to configure your EtherCAT network. The EtherCAT ros configurator prepares a network of devices through the `config/setup.yaml` file. An example of the setup file for an EtherCAT bus with 2 Nanotec C5E-1-21 motor drivers followed by a Maxon EPOS4 driver is given below:
```yaml
ethercat_master:
  time_step:                          0.002 # in seconds
  update_rate_too_low_warn_threshold: 50 # in Hz
  ros_namespace:                      ethercat_master

ethercat_devices:
  - type:               Nanotec # Registration Name of the device SDK
    name:               Nanotec_Motor_1 # Name of the device, should be unique for each device. Used for ROS topics.
    configuration_file: device_configurations/nanotec_1.yaml # Path to the configuration file for the device
    ethercat_bus:       eth0 # eth0 usually
    ethercat_address:   1 # Address of the device on the bus, should be unique for each device
    thread_frequency:   100 # in Hz
    initial_mode_of_operation: 8 # Check mode maps in the SDK docs or device .cpp file

  - type:               Nanotec
    name:               Nanotec_Motor_2
    configuration_file: device_configurations/nanotec_2.yaml
    ethercat_bus:       eth0
    ethercat_address:   2
    thread_frequency:   100
    initial_mode_of_operation: 8
  
  - type:               Maxon
    name:               Maxon_Motor_1
    configuration_file: device_configurations/maxon_1.yaml
    ethercat_bus:       eth0
    ethercat_address:   3
    thread_frequency:   100
    initial_mode_of_operation: 8
```

### `ethercat_master` Section
The `ethercat_master` section contains the configuration for the EtherCAT master. The `time_step` parameter is the time step for the EtherCAT master in seconds. In the example YAML the master is configured to run at 500 Hz, the package has been tested at 1000 Hz (time step of 0.001 secs) too. The `update_rate_too_low_warn_threshold` parameter is the threshold time for the warning message when the update rate of the EtherCAT master is too low. The `ros_namespace` parameter is the namespace for the EtherCAT master. All the device topics will be published or subscribed to under this namespace.

### `ethercat_devices` Section
This section lists the devices connected to the EtherCAT network. Each device is represented as a dictionary with the following keys:
- `type`: The type of the device. This should be the name of the device SDK package. It is the name by which the device class is registered in the `EthercatDeviceFactory`. Check the respective device SDK's documentation for the correct name, or refer to the "Registration Name" column in the table in [Supported Device SDKs](#supported-device-sdks) section.
- `name`: The name of the device. This is the name by which the device will be referred to in the ROS topics and services. It should be unique for each device.
- `configuration_file`: The path to the configuration file for the device. This file should be in the `config` directory (or a subdirectory) of the package. The configuration file should contain the parameters required to configure the device. The parameters are specific to the device and should be documented in the respective device SDK's documentation.
- `ethercat_bus`: The name of the EtherCAT bus to which the device is connected. This is usually `eth0`. For each ethernet port on the computer, a separate bus is created. Bus names can be found by running the `ifconfig` command in the terminal. Note that, for each different bus, a separate instance of the `EthercatMaster` class is created. Therefore, devices on different buses will not be able to communicate with each other and a separate master is created for each bus.
- `ethercat_address`: The address of the device on the EtherCAT bus. The address should be unique for each device on the bus. It serves as the identifier for the device on the bus. Unlike several industrial PLCs this package doesn't support automatic assignment of address. See the important note after this list for more information related to the address.
- `thread_frequency`: The frequency at which the device's own worker thread should run. This is the frequency at which the device's command and feedback data is updated. The frequency should be in Hz.
- `initial_mode_of_operation`: The initial mode of operation for the device. The mode of operation is a parameter specific to the device and should be documented in the respective device SDK's documentation. The mode of operation is the mode in which the device operates. The mode of operation can be changed through the ROS interface. The initial mode of operation should be set to the mode in which the device should start operating. The value of 8 in the example script corresponds to the "Cyclic Synchronous Position Mode" for both Nanotec C5E-1-21 and Maxon EPOS4 motor controllers.

### Important Note On EtherCAT Address
The address 0 is reserved for the EtherCAT master. The EtherCAT address value for all slaves is supposed to be in the range of 1 to 65535 (16 bits). Therefore, one master can support upto 65535 devices. Note that, addresses define which parameters get associated with which device. Addresses are always assigned in a way that the device closest to the master gets the smallest address, the next device gets the smallest address after that and so on. This means, that in the list of device dictionaries in the `ethercat_devices` section, the address value order of the devices should follow the physical wiring order (topological position) of the devices on the bus. Otherwise, you may end up assigning the wrong device class, and the wrong settings to the wrong device which may lead to failure or unexpected behavior. For example, if the address of the Maxon motor is changed to 2 while that of the 2nd Nanotec motor is changed to 3, then the master node will try to create a slave of type Maxon at address 2 for the 2nd Nanotec motor, which will lead to failure. Therefore, it is important to ensure that the address values are assigned according to the correct topological order.

### Command and Feedback Data
At the time of writing, the package is only configured for motor controllers. The package provides a simple way of updating the command and receiving the feedback data through ROS messages. The command and feedback data is updated asynchronously through the worker threads of the device classes. For each device, 2 ROS topics are created:
- `/<ethercat_master_ros_namespace>/<device_name>/command`: The command topic for the device. The command topic is used to send commands to the device. The message type for the command topic is specific to the device and should be documented in the respective device SDK's documentation. For the currently supported motor controllers, it is `ethercat_motor_msgs::MotorCtrlMessage`. The user can send commands to this topic at any rate which updates a local buffer in the device class. The worker thread of the device class then updates the command data to the device at the frequency specified in the `thread_frequency` parameter in the `ethercat_devices` section of the setup file.
- `/<ethercat_master_ros_namespace>/<device_name>/reading`: The feedback topic for the device. The feedback topic is used to receive feedback from the device. The message type for the feedback topic is specific to the device and should be documented in the respective device SDK's documentation. For the currently supported motor controllers, it is `ethercat_motor_msgs::MotorStatusMessage`. The device class updates the feedback data to this topic at the frequency specified in the `thread_frequency` parameter in the `ethercat_devices` section of the setup file.

### Wiring the EtherCAT Network
Check the [EtherCAT installation guide](https://www.ethercat.org/download/documents/ETG1600_V1i0i4_G_R_InstallationGuideline.pdf) for more details on setting up the physical connections between the devices and a ROS enabled PC.Please note a Ethernet port should be available on the ROS machine. A basic linear bus connection topoloy is achieved as follows: One ethernet wire is connected from the ROS machine's Ethernet port to the "input" EtherCAT port of the first device. Then the "output" EtherCAT port of the first device is connected to the "input" EtherCAT port of the second device and so on. This is continued until the last device is reached whose "output" port is left unconnected. The dangling "output" port is handled internally by the EtherCAT communication protocol. A sample diagram of such a connection topology is given below:

<img src="images/Functional_Principal_hd_60fps_v4.gif?raw=true"/>

Image Source: [Acontis Technologies](https://www.acontis.com/en/what-is-ethercat-communication-protocol.html). The reader is encouraged to refer to this link for gaining a better understanding of the EtherCAT communication protocol.

Note that the network might benefit from circular connection topologies because of one layer of redundancy against phyical connection faults. However, this would require two ports in the ROS machine to serve as a part of the same EtherCAT bus; however, this is not supported by the package at the time of writing.

### Running the ROS Node
After setting up the `config/setup.yaml` file, you can run the ROS node by running the following command:
```bash
roslaunch ethercat_ros_configurator ethercat_ros_configurator.launch <path_to_config_file>
```
Replace `<path_to_config_file>` with the path to the `config/setup.yaml` file. The path can either be absolute or relative to the working directory when launching the rosnode.

<span style="color:red">IMPORTANT: </span> Please note that running the EtherCAT master node requires root privilages because of the low level access to the Ethernet port. One way to achieve this is to launch the node while logged in as root in the terminal. This will require sourcing the ROS environment variables in the root shell. Sourcing the ROS environment in root shell is not recommended. A better way to achieve this will be through implementing a [ethercat_grant](https://github.com/shadow-robot/ethercat_grant) like functionality in the package. This is currently not a feature of the package.

# API Documentation and Implementation Details
Refer to the following url for detailed information on how to integrate a new device SDK with the package, and to understand important implementation details as a device SDK developer: [EtherCAT ROS Configurator API Documentation]().