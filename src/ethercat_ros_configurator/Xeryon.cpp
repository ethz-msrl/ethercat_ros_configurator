/*
 ** Copyright (c) 2024, Multi-Scale Robotics Lab - ETH Zürich
 ** Claas Ehmkee
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
#include <xeryon_ethercat_sdk/Xeryon.hpp>


ETHERCAT_ROS_NAMESPACE_BEGIN

class XeryonUtils{
    public:
        static xeryon::ModeOfOperationEnum getModeOfOperation(int mode_of_operation){
            std::map<int, xeryon::ModeOfOperationEnum> mode_of_operation_map = {
                {0, xeryon::ModeOfOperationEnum::NA},
                {1, xeryon::ModeOfOperationEnum::StandardMode},
            };

            if(mode_of_operation_map.find(mode_of_operation) != mode_of_operation_map.end()){
                return mode_of_operation_map[mode_of_operation];
            }
            ROS_ERROR_STREAM("[EthercatDeviceRos::getModeOfOperation] Mode of operation not found. Returning NA[0]");
            return xeryon::ModeOfOperationEnum::NA;
        }
        
};

template <>
void EthercatDeviceRos<xeryon::Xeryon>::worker() {
    ROS_INFO_STREAM("Xeryon '" << device_ptr_->getName() << "': Worker thread started.");
    ros::Rate loop_rate(device_info_.thread_frequency);
    worker_loop_running_ = true;
    std::unique_lock<std::recursive_mutex> lock(*command_msg_mutex_ptr_);
    lock.unlock();

    // This step is important to prevent the motor from rushing
    // to the 0 when the master is started. Note that 0 rush prevention
    // is only done if reading is updated during startup in the device SDK.
    xeryon::Reading reading;
    device_ptr_->getReading(reading);
    last_command_msg_ptr_->targetPosition = reading.getActualPositionRaw();
    // last_command_msg_ptr_->targetVelocity = reading.getActualVelocityRaw();
    // last_command_msg_ptr_->profileAcceleration = reading.getActualAccelerationRaw();
    // last_command_msg_ptr_->profileDeceleration = reading.getActualDecelerationRaw();  

    while(!abrt){
        if(!device_enabled_){
            MELO_INFO_STREAM("Xeryon '" << device_ptr_->getName() << "': Enabling device.");
            device_ptr_->setDriveStateViaPdo(xeryon::DriveState::OperationEnabled, false);
            // Small delay to allow the PDO state change flag to be set. Due to the min number
            // of succesful PDO state readings check taking some time. Increase delay with min
            // number of succesful PDO state readings (param).
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        xeryon::Reading reading;
        device_ptr_->getReading(reading);   
        reading_msg_.header.stamp = ros::Time::now();
        //todo: add the following fields
        reading_msg_.actualPosition = reading.getActualPositionRaw(); //int32

        reading_pub_ptr_->publish(reading_msg_);


        // set commands if we can
        if (device_ptr_->lastPdoStateChangeSuccessful() &&  device_ptr_->getReading().getDriveState() == xeryon::DriveState::OperationEnabled)
        {
            device_enabled_ = true;
            xeryon::Command cmd;
            cmd.setModeOfOperation(XeryonUtils::getModeOfOperation(last_command_msg_ptr_->operationMode));
            lock.lock();
            // MELO_WARN("Xeryon: New target position: %d, velocity: %d, acceleration: %d, deceleration: %d", last_command_msg_ptr_->targetPosition, last_command_msg_ptr_->targetVelocity, last_command_msg_ptr_->profileAcceleration, last_command_msg_ptr_->profileDeceleration);
            cmd.setTargetPositionRaw(last_command_msg_ptr_->targetPosition);
            cmd.setTargetVelocityRaw(last_command_msg_ptr_->targetVelocity);
            cmd.setProfileAccelerationRaw(last_command_msg_ptr_->profileAcceleration);
            cmd.setProfileDecelerationRaw(last_command_msg_ptr_->profileDeceleration);
            lock.unlock();  
            
            device_ptr_->stageCommand(cmd);
        }
        else
        {
            device_enabled_ = false;
            ROS_WARN_STREAM("Xeryon '" << device_ptr_->getName() << "': " << device_ptr_->getReading().getDriveState());
        }
        loop_rate.sleep();
    }
    worker_loop_running_ = false;
    return;
}

template<>
void EthercatDeviceRos<xeryon::Xeryon>::createDevice(){
    device_ptr_ = xeryon::Xeryon::deviceFromFile(device_info_.config_file_path, device_info_.name, device_info_.ethercat_address);
}

template<>
EthercatDeviceClass EthercatDeviceRos<xeryon::Xeryon>::getDeviceClass(){
    return EthercatDeviceClass::MotorController;
}

typedef EthercatDeviceRos<xeryon::Xeryon> XeryonDeviceRos;

ETHERCAT_ROS_REGISTER_DEVICE("Xeryon", XeryonDeviceRos);
ETHERCAT_ROS_NAMESPACE_END