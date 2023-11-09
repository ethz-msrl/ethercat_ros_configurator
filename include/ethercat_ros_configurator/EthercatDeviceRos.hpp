
#include <ethercat_ros_configurator/EthercatDeviceConfigurator.hpp>
#include <ethercat_motor_msgs/MotorCtrlMessage.h>
#include <ethercat_motor_msgs/MotorStatusMessage.h>
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

template <class Device>
/**
 * @brief The EthercatDeviceRos class, provides a ROS interface to an EthercatDevice.
*/
class EthercatDeviceRos{
    public:
        typedef std::shared_ptr<EthercatDeviceRos> SharedPtr;

        /**
         * @brief EthercatDeviceRos ctor
        */
        EthercatDeviceRos(ros::NodeHandle& nh, Device &device, EthercatSlaveInfo &device_info){
            device_ = std::make_shared<Device>(device);
            device_info_ = device_info;
            command_sub_ptr_ = std::make_unique<ros::Subscriber>(
                nh.subscribe<ethercat_motor_msgs::MotorCtrlMessage>(device_info_.ros_ns + "/command", 1000, &EthercatDeviceRos::commandCallback, this)
                );
            
            reading_pub_ptr_ = std::make_unique<ros::Publisher>(
                nh.advertise<ethercat_motor_msgs::MotorStatusMessage>(device_info_.ros_ns + "/reading", 1000)
                );
            

            staged_command_ptr_ = std::make_unique<Device::Command>();

        }

        /**
         * @brief Move ctor for EthercatDeviceRos
        */
        EthercatDeviceRos(EthercatDeviceRos&& other) noexcept {
            device_ = std::move(other.device_);
            device_info_ = std::move(other.device_info_);
            command_sub_ptr_ = std::move(other.command_sub_ptr_);
            reading_pub_ptr_ = std::move(other.reading_pub_ptr_);
            worker_thread_ptr_ = std::move(other.worker_thread_ptr_);
            staged_command_ptr_ = std::move(other.staged_command_ptr_);
            other.device_ = nullptr;
            other.device_info_ = EthercatSlaveInfo();
        }

        /**
         * @brief Move assignment ctor for EthercatDeviceRos
        */
       EthercatDeviceRos& operator=(EthercatDeviceRos&& other) noexcept {
            if(this == &other) return *this;
            device_ = std::move(other.device_);
            device_info_ = std::move(other.device_info_);
            command_sub_ptr_ = std::move(other.command_sub_ptr_);
            reading_pub_ptr_ = std::move(other.reading_pub_ptr_);
            worker_thread_ptr_ = std::move(other.worker_thread_ptr_);
            staged_command_ptr_ = std::move(other.staged_command_ptr_);
            other.device_ = nullptr;
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

        void worker();

        /**
         * @brief commandCallback - callback for the command topic. Updates
         * the staged command of the device used by the worker thread.
        */
        void commandCallback(const ethercat_motor_msgs::MotorCtrlMessage::ConstPtr& msg);
    
        std::shared_ptr<Device> device_; // Shared pointer to slave.
                                         // because Ethercat master needs it too.
        EthercatSlaveInfo device_info_;
        std::unique_ptr<ros::Subscriber> command_sub_ptr_;
        std::unique_ptr<ros::Publisher> reading_pub_ptr_;
        std::unique_ptr<Device::Command> staged_command_ptr_;
        mutable std::recursive_mutex staged_command_mutex_;
};

} // namespace EthercatRos