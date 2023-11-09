
#include <ethercat_ros_configurator/EthercatDeviceRos.hpp>

namespace EthercatRos{
    template <class Device>
    void EthercatDeviceRos<Device>::commandCallback(const ethercat_motor_msgs::MotorCtrlMessage::ConstPtr& msg){
        std::lock_guard<std::recursive_mutex> lock(staged_command_mutex_);
        staged_command_ptr_->setTargetPositionRaw(msg->targetPosition);
        staged_command_ptr_->setTargetVelocityRaw(msg->targetVelocity);
        staged_command_ptr_->setTargetTorqueRaw(msg->targetTorque);
        staged_command_ptr_->setPositionOffsetRaw(msg->positionOffset);
        staged_command_ptr_->setTorqueOffsetRaw(msg->torqueOffset);
        staged_command_ptr_->setVelocityOffsetRaw(msg->velocityOffset);
    }
}; // namespace EthercatRos