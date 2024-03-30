# EtherCAT ROS Configurator: API Documentation

This is the API documentation of the EtherCAT ROS Configruator package. This page lists the important points related to the software architecture used for adding the devices and is intended as a brief guide for developers to add new device SDKs or create compatibility classes for new device types.

- [EtherCAT ROS Configurator: API Documentation](#ethercat-ros-configurator-api-documentation)
  - [Adding New Device Types](#adding-new-device-types)
    - [Adding Motor Controllers](#adding-motor-controllers)
    - [Adding Other Device Types](#adding-other-device-types)

## Adding New Device Types
The basic idea behind this package is to provide an object factory based interface for easy addition of new devices without requiring any interaction with the code responsible for launching and managing the slaves. This is achieved through the usual C++ runtime polymorphism. For this to work we make a distinction between the device type (for eg: motor controller, sensors, etc) and the device driver specific code itself. Check the `EthercatDeviceRosBase` class in `include/EthercatDeviceRos.hpp` which specifies all the basic methods which all the devices (or their type classes) must implement. The next step is to create a C++ class definition for the same class of EtherCAT devices. This class' job is to implement all the common functionalities between the device drivers and provide an easy interface to the driver developers to integrate their devices with the package. For now only the type class for EtherCAT Motor Controllers has been implemented, and more type classes shall be added later if required.

### Adding Motor Controllers
For this section, the reader is advised to go through the type class for EtherCAT Motor Controllers called `EthercatDeviceRos` in this release, in the file `include/EthercatDeviceRos.hpp`. This object encapsulates the data which is common to motor controllers thanks to a sufficient level of standardization with respect to operation modes and the required data. The idea is to fully specialize the templated `EthercatDeviceRos` class with the device driver class. *Please note that the device constructor must inherit from the `ecat_master::EthercatDevice` class since it needs to implement some mandatory functions used by the configurator for starting up the connection with the device and managing its event loop.* Following is an example skeleton code for adding some device driver called `XYZMotorController`, a lot of variables mentioned are encapsulated in `EthercatDeviceRos`:
```C++
#include <ethercat_ros_configurator/EthercatDeviceRos.hpp>
#include <xyz_device_sdk/xyz.hpp>

ETHERCAT_ROS_NAMESPACE_BEGIN
template <>
void EthercatDeviceRos<xyz::XYZMotorController>::worker(){
    ros::Rate loop_rate(device_info_.thread_frequency);
    worker_loop_running_ = true; // Refer to the base class for this flag.
    std::unique_lock<std::recursive_mutex> lock(*command_msg_mutex_ptr_);
    lock.unlock();

    /**
     * Perform some action before the main event loop begins. For
     * example setting the initial reading in order to allow the motors
     * to start from their current state. Otherwise they'll jump to the
     * zero position on startup.
    */

    while(!abrt){
        if(!device_enabled_){
                /**
                 * Actions to perform in order to set the device to the OperationEnabled state for example.
                 * This name of the enable state and the behavior here may
                 * change depending on the specific Ethercat profile being
                 * followed by the device.
                */
            }

            /**
             * Received feedback from the Reading message and then update 
             * the device' command structures based on the last received
             * command (see last_command_msg_ptr). Don't forget to lock the
             * command message mutex in order to avoid data race.
            */
     
    }
    // worker loop shut down at this point, set any required flags.
    worker_loop_running_ = false;
    return;
}

template<>
void EthercatDeviceRos<xyz::XYZMotorController>::createDevice(){
    // driver SDK and user parameter input specific code for creating the 
    // device class. For example:
    device_ptr_ = xyz::XYZCreators::deviceFromFile(device_info_.config_file_path, device_info_.name, device_info_.ethercat_address);
}

template<>
EthercatDeviceClass EthercatDeviceRos<xyz::XYZMotorController>::getDeviceClass(){
    /**
     * Return the correct device type from the EthercatDeviceClass enum
     * In this case it'll almost always be motor controller.
    */
    return EthercatDeviceClass::MotorController;
}

typedef EthercatDeviceRos<xyz::XYZMotorController> XYZDeviceRos;

// Finally, register the class, this adds its constructor to the object factory.
ETHERCAT_ROS_REGISTER_DEVICE("XYZ", XYZDeviceRos); 
ETHERCAT_ROS_NAMESPACE_END
```

For existing implementations refer to the files `src/Maxon.cpp` and `src/Nanotec.cpp` in conjunction with their respective SDKs. Links to the SDK repositories can be found in the main readme file for this package.

Note that the registration name of the class also becomes the name by which it will be created through the `setup.yaml` file. So if one wanted to add a XYZ Motor Controller to the array of their EtherCAT devices they'll add the following block to their `setup.yaml` file's `ethercat_devices` section:
```yaml
  - type:               XYZ # Same as the class registration name.
    name:               XYZ_Motor_Controller
    configuration_file: device_configurations/xyz.yaml # or any other input file type for parameters
    ethercat_bus:       eth0
    ethercat_address:   N # Any unique address N > 0
    thread_frequency:   100 # Tune to the desired value
    initial_mode_of_operation: 8 # Depends on the motor controller. 8 is Cyclic Sync Position mode for Maxon and Nanotec.
```

### Adding Other Device Types
`EthercatDeviceRos` class cannot be used for other devices types than motor controllers because it is hard bound to the `MotorControlMessage` and `MotorStatusMessage` ROS message types. The class was intended to be more generic; however, going through the template specialization route would have meant adding another template. It'll then be logical to go for a specizalition over the message template to create a class for motor controller named something like EthercatMotorController. However, this will require a partial specialization which means a complete redefinition of all the methods from the base class which defets the purpose of inheriting to minimize code duplication in the first place. This is the reason behind the generic name of the motor controller class. 

Suggestions and improvements of the inheritence pipeline are greatly welcomed in order to implement this new level of abstraction. But then it might be relatively low effort to just add another device type class which is 50% the same as the EthercatDeviceRos class, in which case one can go ahead with the current design. By the current design if one wants to make, say, EtherCAT Sensors compatible with the package then they'll have to inherit from `EthercatDeviceRosBase` and create the type class just like `EthercatDeviceRos`. The worker loop handling functions may be repititive and may just be taken as is but the amount of code duplication shouldn't be a lot, unless the base classes get fairly complex in the future.