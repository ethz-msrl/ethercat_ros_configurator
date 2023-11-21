
#include <ethercat_ros_configurator/EthercatDeviceConfigurator.hpp>
#include <ethercat_motor_msgs/MotorCtrlMessage.h>
#include <ethercat_motor_msgs/MotorStatusMessage.h>
#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#include <nanotec_ethercat_sdk/Nanotec.hpp>
#include <ros/ros.h>

#include <thread>
#include <chrono>
#include <mutex>

namespace EthercatRos{
    
enum class EthercatSlaveType
{
    Maxon,
    Nanotec,
    NA
};
    
struct EthercatSlaveInfo
{
    EthercatSlaveType type = EthercatSlaveType::NA;
    std::string ros_ns = "";
    std::string name = "";
    std::string config_file_path = "";

    uint32_t ethercat_address = 0;
    std::string ethercat_bus = "";
    std::string ethercat_pdo_type = "";
};

template <class DeviceClass>
/**
 * @brief The EthercatDeviceRos class, provides a ROS interface to an EthercatDevice.
*/
class EthercatDeviceRos{
    public:
        typedef std::shared_ptr<EthercatDeviceRos> SharedPtr;

        /**
         * @brief EthercatDeviceRos ctor
        */
        EthercatDeviceRos(ros::NodeHandle& nh, 
                          EthercatDeviceConfigurator::SharedPtr configurator, 
                          const std::shared_ptr<ecat_master::EthercatDevice> &slave, 
                          EthercatSlaveInfo &device_info){

            device_info_ = device_info;
            configurator_ = configurator;
            device_ptr_ = std::dynamic_pointer_cast<DeviceClass>(slave);

            command_sub_ptr_ = std::make_unique<ros::Subscriber>(
                nh.subscribe<ethercat_motor_msgs::MotorCtrlMessage>(device_info_.ros_ns + "/" + device_info.name + "/command", 1000, &EthercatDeviceRos::commandCallback, this)
                );
            
            reading_pub_ptr_ = std::make_unique<ros::Publisher>(
                nh.advertise<ethercat_motor_msgs::MotorStatusMessage>(device_info_.ros_ns + "/" + device_info.name + "/reading", 1000)
                );
            

            staged_command_ptr_ = std::make_unique<Device::Command>();

            device_enabled_ = true;

        }

        /**
         * @brief Move ctor for EthercatDeviceRos
        */
        EthercatDeviceRos(EthercatDeviceRos&& other) noexcept {
            device_ptr_ = std::move(other.device_ptr_);
            device_info_ = std::move(other.device_info_);
            command_sub_ptr_ = std::move(other.command_sub_ptr_);
            reading_pub_ptr_ = std::move(other.reading_pub_ptr_);
            worker_thread_ptr_ = std::move(other.worker_thread_ptr_);
            staged_command_ptr_ = std::move(other.staged_command_ptr_);
            other.device_ptr_ = nullptr;
            other.device_info_ = EthercatSlaveInfo();
        }

        /**
         * @brief Move assignment ctor for EthercatDeviceRos
        */
       EthercatDeviceRos& operator=(EthercatDeviceRos&& other) noexcept {
            if(this == &other) return *this;
            device_ptr_ = std::move(other.device_ptr_);
            device_info_ = std::move(other.device_info_);
            command_sub_ptr_ = std::move(other.command_sub_ptr_);
            reading_pub_ptr_ = std::move(other.reading_pub_ptr_);
            worker_thread_ptr_ = std::move(other.worker_thread_ptr_);
            staged_command_ptr_ = std::move(other.staged_command_ptr_);
            other.device_ptr_ = nullptr;
            other.device_info_ = EthercatSlaveInfo();
            return *this;
       }

       /**
        * @brief EthercatDeviceRos dtor
       */
        virtual ~EthercatDeviceRos(){
            if(worker_thread_ptr_ != nullptr){
                worker_thread_ptr_->join();
            }
        }

        // Disabling copy constructor and copy assignment because
        // we never want multiple copies of same ethercat device.
        EthercatDeviceRos(const EthercatDeviceRos&) = delete;
        EthercatDeviceRos& operator=(const EthercatDeviceRos&) = delete;

        /**
         * @brief startWorkerThread - starts the worker thread
        */
        void startWorkerThread(){
            worker_thread_ptr_ = std::make_unique<std::thread>(&EthercatDeviceRos::worker, this);
        }
        
    protected:
        std::unique_ptr<std::thread> worker_thread_ptr_;

        void worker(){
            
            // Worker thread retains the style of standalone.cpp because of software
            // architecture reasons. Slave type specific operations are possible here.

            if (configurator->getInfoForSlave(
                std::static_pointer_cast<ecat_master::EthercatDevice>(device_ptr_),
                ).type == EthercatDeviceConfigurator::EthercatSlaveType::Maxon) {
                if(!device_enabled_){
                    device_ptr_->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, false);
                }

                // set commands if we can
                if (device_ptr_->lastPdoStateChangeSuccessful() &&
                        device_ptr_->getReading().getDriveState() == maxon::DriveState::OperationEnabled)
                {
                    // @todo: make mode of operation configurable
                    command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode);

                    reading_msg_.header.stamp = ros::Time::now();
                    reading_msg_.actualPosition = device_ptr_->getReading().getActualPositionRaw();
                    reading_msg_.actualVelocity = device_ptr_->getReading().getActualVelocityRaw();
                    reading_msg_.actualTorque   = device_ptr_->getReading().getActualCurrent(); // see txPDO defn in maxon's SDK
                }
                else
                {
                    MELO_WARN_STREAM("Maxon '" << device_ptr_->getName()
                                                                         << "': " << device_ptr_->getReading().getDriveState());
                }
            }
            else if (configurator->getInfoForSlave(
                std::static_pointer_cast<ecat_master::EthercatDevice>(device_ptr_),
                ).type == EthercatDeviceConfigurator::EthercatSlaveType::Nanotec) {
                if(!device_enabled_){
                    device_ptr_->setDriveStateViaPdo(nanotec::DriveState::OperationEnabled, false);
                }

                // set commands if we can
                if (device_ptr_->lastPdoStateChangeSuccessful() &&
                        device_ptr_->getReading().getDriveState() == nanotec::DriveState::OperationEnabled)
                {
                    // @todo: make mode of operation configurable
                    command.setModeOfOperation(nanotec::ModeOfOperationEnum::CyclicSynchronousPositionMode);
                }
            }


        }

        /**
         * @brief commandCallback - callback for the command topic. Updates
         * the staged command of the device used by the worker thread.
        */
        void commandCallback(const ethercat_motor_msgs::MotorCtrlMessage::ConstPtr& msg){
            staged_command_ptr_->setTargetPositionRaw(msg->targetPosition);
            staged_command_ptr_->setTargetVelocityRaw(msg->targetVelocity);
            staged_command_ptr_->setTargetTorqueRaw(msg->targetTorque);
            staged_command_ptr_->setPositionOffsetRaw(msg->positionOffset);
            staged_command_ptr_->setTorqueOffsetRaw(msg->torqueOffset);
            staged_command_ptr_->setVelocityOffsetRaw(msg->velocityOffset);
            // The following line is only compatible with nanotec and maxon for now.
            device_ptr_->stageCommand(*staged_command_ptr_);
        }
    
        std::shared_ptr<DeviceClass> device_ptr_; // Shared pointer to slave.
                                         // because Ethercat master needs it too.
        EthercatDeviceConfigurator::SharedPtr configurator_;
        EthercatSlaveInfo device_info_;
        std::unique_ptr<ros::Subscriber> command_sub_ptr_;
        std::unique_ptr<ros::Publisher> reading_pub_ptr_;
        std::unique_ptr<DeviceClass::Command> staged_command_ptr_;
        ethercat_motor_msgs::MotorStatusMessage reading_msg_;
        mutable std::recursive_mutex staged_command_mutex_;
        bool device_enabled_ = false;

        
};

} // namespace EthercatRos