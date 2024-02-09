#include <ethercat_ros_configurator/EthercatDeviceRos.hpp>


ETHERCAT_ROS_NAMESPACE_BEGIN

template <>
void EthercatDeviceRos<nanotec::Nanotec>::worker() {
    ROS_INFO_STREAM("Nanotec '" << device_ptr_->getName() << "': Worker thread started.");
    ros::Rate loop_rate(device_info_.thread_frequency);
    worker_loop_running_ = true;
    std::unique_lock<std::recursive_mutex> lock(*command_msg_mutex_ptr_);
    lock.unlock();

    // This step is important to prevent the motor from rushing
    // to the 0 when the master is started. Note that 0 rush prevention
    // is only done if reading is updated during startup in the device SDK.
    nanotec::Reading reading;
    device_ptr_->getReading(reading);
    last_command_msg_ptr_->targetPosition = reading.getActualPositionRaw();
    last_command_msg_ptr_->targetVelocity = reading.getActualVelocityRaw();
    last_command_msg_ptr_->targetTorque = reading.getActualTorqueRaw();
    last_command_msg_ptr_->positionOffset = 0;
    last_command_msg_ptr_->velocityOffset = 0;
    last_command_msg_ptr_->torqueOffset = 0;
    last_command_msg_ptr_->motionProfileType = 0;
    last_command_msg_ptr_->profileAcceleration = 0;
    last_command_msg_ptr_->profileDeceleration = 0;   

    while(!abrt){
        if(!device_enabled_){
            device_ptr_->setDriveStateViaPdo(nanotec::DriveState::OperationEnabled, false);
            // Small delay to allow the PDO state change flag to be set. Due to the min number
            // of succesful PDO state readings check taking some time. Increase delay with min
            // number of succesful PDO state readings (param).
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        nanotec::Reading reading;
        device_ptr_->getReading(reading);
        reading_msg_.header.stamp = ros::Time::now();
        reading_msg_.actualPosition = reading.getActualPositionRaw();
        reading_msg_.actualVelocity = reading.getActualVelocityRaw();
        reading_msg_.statusword = reading.getRawStatusword();
        reading_msg_.demandTorque = reading.getDemandTorqueRaw();
        reading_msg_.actualTorque = reading.getActualTorqueRaw();
        reading_msg_.actualFollowingError = reading.getActualFollowingErrorRaw();
        reading_pub_ptr_->publish(reading_msg_);


        // set commands if we can
        if (device_ptr_->lastPdoStateChangeSuccessful() &&
                device_ptr_->getReading().getDriveState() == nanotec::DriveState::OperationEnabled)
        {
            // @todo: make mode of operation configurable

            device_enabled_ = true;
            nanotec::Command cmd;
            cmd.setModeOfOperation(NanotecUtils::getModeOfOperation(last_command_msg_ptr_->operationMode));
            lock.lock();
            cmd.setTargetPositionRaw(last_command_msg_ptr_->targetPosition);
            cmd.setTargetVelocityRaw(last_command_msg_ptr_->targetVelocity);
            cmd.setTargetTorqueRaw(last_command_msg_ptr_->targetTorque);
            cmd.setPositionOffsetRaw(last_command_msg_ptr_->positionOffset);
            cmd.setTorqueOffsetRaw(last_command_msg_ptr_->torqueOffset);
            cmd.setVelocityOffsetRaw(last_command_msg_ptr_->velocityOffset);
            lock.unlock();

            device_ptr_->stageCommand(cmd);
        }
        else
        {
            device_enabled_ = false;
            ROS_WARN_STREAM("Nanotec '" << device_ptr_->getName()
                                                                << "': " << device_ptr_->getReading().getDriveState());
        }
        loop_rate.sleep();
    }
    worker_loop_running_ = false;
    return;
}

template<>
void EthercatDeviceRos<nanotec::Nanotec>::createDevice(){
    device_ptr_ = nanotec::Nanotec::deviceFromFile(device_info_.config_file_path, device_info_.name, device_info_.ethercat_address);
}

template<>
EthercatDeviceClass EthercatDeviceRos<nanotec::Nanotec>::getDeviceClass(){
    return EthercatDeviceClass::MotorController;
}

typedef EthercatDeviceRos<nanotec::Nanotec> NanotecDeviceRos;

ETHERCAT_ROS_REGISTER_DEVICE("Nanotec", NanotecDeviceRos);
ETHERCAT_ROS_NAMESPACE_END