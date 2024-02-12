#include <ethercat_ros_configurator/EthercatDeviceRos.hpp>
#include <maxon_epos_ethercat_sdk/Maxon.hpp>

ETHERCAT_ROS_NAMESPACE_BEGIN

class MaxonUtils{
    public:
        /**
         * @brief operation mode mapping for maxon. It is recommended to change the way changing
         * the mode of operation is handled. This is a temporary solution because this is what 
         * I thought was feasible with the current class heirarchies and template specializations
         * while avoiding inheritances from specialized classes.
        */
        static maxon::ModeOfOperationEnum getModeOfOperation(int mode_of_operation){
            std::map<int, maxon::ModeOfOperationEnum> mode_of_operation_map = {
                    {0, maxon::ModeOfOperationEnum::NA},
                    {1, maxon::ModeOfOperationEnum::ProfiledPositionMode}, // default for empty msg
                    {3, maxon::ModeOfOperationEnum::ProfiledVelocityMode},
                    {6, maxon::ModeOfOperationEnum::HomingMode},
                    {8, maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode},
                    {9, maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode},
                    {10, maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode}
            };
            if(mode_of_operation_map.find(mode_of_operation) != mode_of_operation_map.end()){
                return mode_of_operation_map[mode_of_operation];
            }
            ROS_ERROR_STREAM("[EthercatDeviceRos::getModeOfOperation] Mode of operation not found. Returning NA[0]");
            return maxon::ModeOfOperationEnum::NA;
        }
};

template <>
void EthercatDeviceRos<maxon::Maxon>::worker(){
    ROS_INFO_STREAM("Maxon '" << device_ptr_->getName() << "': Worker thread started.");
    ros::Rate loop_rate(device_info_.thread_frequency);
    worker_loop_running_ = true;
    std::unique_lock<std::recursive_mutex> lock(*command_msg_mutex_ptr_);
    lock.unlock();

    maxon::Reading reading;
    device_ptr_->getReading(reading);
    last_command_msg_ptr_->targetPosition = reading.getActualPositionRaw();
    last_command_msg_ptr_->targetVelocity = reading.getActualVelocityRaw();
    last_command_msg_ptr_->targetTorque = reading.getActualCurrentRaw();
    last_command_msg_ptr_->positionOffset = 0;
    last_command_msg_ptr_->velocityOffset = 0;
    last_command_msg_ptr_->torqueOffset = 0;
    last_command_msg_ptr_->motionProfileType = 0;
    last_command_msg_ptr_->profileAcceleration = 0;
    last_command_msg_ptr_->profileDeceleration = 0;

    while(!abrt){
            if(!device_enabled_){
                device_ptr_->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, false);
                // Small delay to allow the PDO state change flag to be set. Due to the min number
                // of succesful PDO state readings check taking some time.
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            maxon::Reading reading;
            device_ptr_->getReading(reading);
            reading_msg_.header.stamp = ros::Time::now();
            reading_msg_.actualPosition = reading.getActualPositionRaw();
            reading_msg_.actualVelocity = reading.getActualVelocityRaw();
            reading_msg_.statusword = reading.getRawStatusword();
            reading_msg_.analogInput = reading.getAnalogInputRaw();
            reading_msg_.busVoltage = reading.getBusVoltageRaw();
            reading_msg_.actualTorque   = reading.getActualCurrent(); // see txPDO defn in maxon's SDK
            reading_msg_.errorCode = reading.getErrorCode();
            reading_pub_ptr_->publish(reading_msg_);


            // set commands if we can
            if (device_ptr_->lastPdoStateChangeSuccessful() &&
                    device_ptr_->getReading().getDriveState() == maxon::DriveState::OperationEnabled)
            {
                // @todo: make mode of operation configurable : Add to last_command_msg_ptr_ smartly
                // Maybe a map of int to OpModes in different device classes.

                // Initially a pointer to command was maintained, but was removed for uneccessary
                // template specializations. This should have a slight allocation overhead but compiler
                // optimizations should reduce it a bit.

                maxon::Command cmd;
                cmd.setModeOfOperation(MaxonUtils::getModeOfOperation(last_command_msg_ptr_->operationMode));
                lock.lock();
                cmd.setTargetPositionRaw(last_command_msg_ptr_->targetPosition);
                cmd.setTargetVelocityRaw(last_command_msg_ptr_->targetVelocity);
                cmd.setTargetTorqueRaw(last_command_msg_ptr_->targetTorque);
                cmd.setPositionOffsetRaw(last_command_msg_ptr_->positionOffset);
                cmd.setTorqueOffsetRaw(last_command_msg_ptr_->torqueOffset);
                cmd.setVelocityOffsetRaw(last_command_msg_ptr_->velocityOffset);
                lock.unlock();
                device_ptr_->stageCommand(cmd);

                device_enabled_ = true;
            }
            else
            {
                device_enabled_ = false;
                ROS_WARN_STREAM("Maxon '" << device_ptr_->getName()
                                                                    << "': " << device_ptr_->getReading().getDriveState());
            }

            loop_rate.sleep();
    }
    worker_loop_running_ = false;
    return;
}

template<>
void EthercatDeviceRos<maxon::Maxon>::createDevice(){
    device_ptr_ = maxon::Maxon::deviceFromFile(device_info_.config_file_path, device_info_.name, device_info_.ethercat_address);
}

template<>
EthercatDeviceClass EthercatDeviceRos<maxon::Maxon>::getDeviceClass(){
    return EthercatDeviceClass::MotorController;
}

typedef EthercatDeviceRos<maxon::Maxon> MaxonDeviceRos;

// Don't forget to register the class
ETHERCAT_ROS_REGISTER_DEVICE("Maxon", MaxonDeviceRos);
ETHERCAT_ROS_NAMESPACE_END