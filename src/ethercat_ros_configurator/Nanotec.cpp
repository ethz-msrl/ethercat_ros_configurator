/*
 ** Copyright (c) 2024, Multi-Scale Robotics Lab - ETH Zürich
 ** Neelaksh Singh
 **
 ** Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are met:
 ** 
 ** 1. Redistributions of source code must retain the above copyright notice, this
 **    list of conditions and the following disclaimer.
 ** 
 ** 2. Redistributions in binary form must reproduce the above copyright notice,
 **    this list of conditions and the following disclaimer in the documentation
 **    and/or other materials provided with the distribution.
 ** 
 ** 3. Neither the name of the copyright holder nor the names of its
 **    contributors may be used to endorse or promote products derived from
 **    this software without specific prior written permission.
 ** 
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 ** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 ** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 ** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 ** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 ** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 ** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 ** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 ** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ethercat_ros_configurator/EthercatDeviceRos.hpp>
#include <nanotec_ethercat_sdk/Nanotec.hpp>


ETHERCAT_ROS_NAMESPACE_BEGIN

class NanotecUtils{
    public:
        /**
         * @brief operation mode mapping for Nanotec C5E-1-21. It is recommended to change the way changing
         * the mode of operation is handled. This is a temporary solution because this is what 
         * I thought was feasible with the current class heirarchies and template specializations
         * while avoiding inheritances from specialized classes.
        */
        static nanotec::ModeOfOperationEnum getModeOfOperation(int mode_of_operation){
            std::map<int, nanotec::ModeOfOperationEnum> mode_of_operation_map = {
                {-2, nanotec::ModeOfOperationEnum::AutoSetup},
                {-1, nanotec::ModeOfOperationEnum::ClockDirectionMode},
                {0, nanotec::ModeOfOperationEnum::NA},
                {1, nanotec::ModeOfOperationEnum::ProfilePositionMode},
                {2, nanotec::ModeOfOperationEnum::VelocityMode},
                {3, nanotec::ModeOfOperationEnum::ProfileVelocityMode},
                {4, nanotec::ModeOfOperationEnum::ProfileTorqueMode},
                {6, nanotec::ModeOfOperationEnum::HomingMode},
                {7, nanotec::ModeOfOperationEnum::InterpolatedPositionMode},
                {8, nanotec::ModeOfOperationEnum::CyclicSynchronousPositionMode},
                {9, nanotec::ModeOfOperationEnum::CyclicSynchronousVelocityMode},
                {10, nanotec::ModeOfOperationEnum::CyclicSynchronousTorqueMode}
            };

            if(mode_of_operation_map.find(mode_of_operation) != mode_of_operation_map.end()){
                return mode_of_operation_map[mode_of_operation];
            }
            ROS_ERROR_STREAM("[EthercatDeviceRos::getModeOfOperation] Mode of operation not found. Returning NA[0]");
            return nanotec::ModeOfOperationEnum::NA;
        }
        
};

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

          
            device_ptr_->stageCommand(cmd); // update command for update_write
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