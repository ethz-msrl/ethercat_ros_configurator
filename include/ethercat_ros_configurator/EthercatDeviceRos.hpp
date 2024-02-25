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

#include <ethercat_sdk_master/EthercatMaster.hpp>

#include <ethercat_motor_msgs/MotorCtrlMessage.h>
#include <ethercat_motor_msgs/MotorStatusMessage.h>

#include <ros/ros.h>

#include <thread>
#include <csignal>
#include <chrono>
#include <mutex>

#define ETHERCAT_ROS_NAMESPACE_BEGIN namespace EthercatRos {
#define ETHERCAT_ROS_NAMESPACE_END }

namespace EthercatRos{

typedef std::string EthercatSlaveType;

/**
 * @brief An enum to identify what the type of device is, for eg: MotorController, Sensor,
 * etc. A utility enum which might be useful in the future expansions to add new devices.
*/
enum class EthercatDeviceClass{
    NA,
    MotorController
};

struct EthercatSlaveEntry
{
    EthercatSlaveType type = "NA";
    std::string name = "";
    std::string config_file_path = "";

    uint32_t ethercat_address = 0;
    std::string ethercat_bus = "";
    std::string ethercat_pdo_type = "";
    uint32_t thread_frequency = 100;
    int8_t initial_mode_of_operation = 0; // Mode "NA" in most drivers
};

class EthercatDeviceRosBase{
    public:
        virtual ~EthercatDeviceRosBase() = default;
        virtual void startWorkerThread() = 0;
        virtual void joinWorkerThread() = 0;
        virtual std::shared_ptr<ecat_master::EthercatDevice> getSlaveObjPtr() const = 0;
        virtual std::string getName() const = 0;
        virtual void abort() = 0;
        virtual void createDevice() = 0;
        virtual EthercatDeviceClass getDeviceClass() = 0;
};

template <class DeviceClass>
/**
 * @brief The EthercatDeviceRos class, provides a ROS interface to an EthercatDevice.
*/
class EthercatDeviceRos : public EthercatDeviceRosBase{
    public:
        typedef std::shared_ptr<EthercatDeviceRos> SharedPtr;

        /**
         * @brief EthercatDeviceRos ctor
        */
        EthercatDeviceRos(const std::shared_ptr<ros::NodeHandle> &nh_ptr,
                          const EthercatSlaveEntry &device_info){
            nh_ptr_ = nh_ptr;
            device_info_ = device_info;
            this->createDevice();

            command_sub_ptr_ = std::make_unique<ros::Subscriber>(
                nh_ptr_->subscribe<ethercat_motor_msgs::MotorCtrlMessage>(device_info.name + "/command", 1000, &EthercatDeviceRos::commandCallback, this)
                );
            
            reading_pub_ptr_ = std::make_unique<ros::Publisher>(
                nh_ptr_->advertise<ethercat_motor_msgs::MotorStatusMessage>(device_info.name + "/reading", 1000)
                );
            
            last_command_msg_ptr_ = std::make_unique<ethercat_motor_msgs::MotorCtrlMessage>();
            command_msg_mutex_ptr_ = std::make_unique<std::recursive_mutex>();

            // set initial mode of operation
            last_command_msg_ptr_->operationMode = device_info.initial_mode_of_operation;

            device_enabled_ = true;

        }

        /**
         * @brief Move ctor for EthercatDeviceRos
        */
        EthercatDeviceRos(EthercatDeviceRos&& other) noexcept {
            device_ptr_ = std::move(other.device_ptr_);
            device_info_ = std::move(other.device_info_);
            nh_ptr_ = std::move(other.nh_ptr_);
            command_sub_ptr_ = std::move(other.command_sub_ptr_);
            reading_pub_ptr_ = std::move(other.reading_pub_ptr_);
            last_command_msg_ptr_ = std::move(other.last_command_msg_ptr_);
            command_msg_mutex_ptr_ = std::move(other.command_msg_mutex_ptr_);
            worker_thread_ptr_ = std::move(other.worker_thread_ptr_);
            device_enabled_ = other.device_enabled_;
            abrt = other.abrt;
            other.device_ptr_ = nullptr;
            other.device_info_ = EthercatSlaveEntry(); // not needed because trivial type
        }

        /**
         * @brief Move assignment ctor for EthercatDeviceRos
        */
       EthercatDeviceRos& operator=(EthercatDeviceRos&& other) noexcept {
            if(this == &other) return *this;
            device_ptr_ = std::move(other.device_ptr_);
            device_info_ = std::move(other.device_info_);
            nh_ptr_ = std::move(other.nh_ptr_);
            command_sub_ptr_ = std::move(other.command_sub_ptr_);
            reading_pub_ptr_ = std::move(other.reading_pub_ptr_);
            last_command_msg_ptr_ = std::move(other.last_command_msg_ptr_);
            command_msg_mutex_ptr_ = std::move(other.command_msg_mutex_ptr_);
            worker_thread_ptr_ = std::move(other.worker_thread_ptr_);
            device_enabled_ = other.device_enabled_;
            abrt = other.abrt;
            other.device_ptr_ = nullptr;
            other.device_info_ = EthercatSlaveEntry();
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

        virtual void abort() override {
            ROS_INFO("EthercatDevice %s aborting.", device_info_.name.c_str());
            abrt = true;
        }

        /**
         * @brief startWorkerThread - starts the worker thread
        */
        virtual void startWorkerThread() override {
            worker_thread_ptr_ = std::make_unique<std::thread>(&EthercatDeviceRos::worker, this);
        }

        virtual void joinWorkerThread() override {
            worker_thread_ptr_->join();
        }

        /**
         * @brief getSlave - returns the slave
         * @return shared_ptr on slave
        */
        virtual std::shared_ptr<ecat_master::EthercatDevice> getSlaveObjPtr() const override {
            return std::static_pointer_cast<ecat_master::EthercatDevice>(device_ptr_);
        }

        virtual std::string getName() const override {
            return device_info_.name;
        }

        virtual void createDevice() override {
            ROS_ERROR("EthercatDeviceRos::createDevice() not implemented for this device type.");
        }

        virtual EthercatDeviceClass getDeviceClass() override {
            return EthercatDeviceClass::NA;
        }
        
    protected:
        std::unique_ptr<std::thread> worker_thread_ptr_;

        /**
         * @brief worker - worker thread function
         * This is the main thread which reads the local command buffer and sends them
         * to the command class of the device. It also reads the readings from the device
         * and publishes them.
        */
        void worker() {
            
            // Worker thread retains the style of standalone.cpp because of software
            // architecture reasons. Slave type specific operations are possible here.
            // Specify this in device specific specializations of workerDispatch().

            ROS_ERROR("EthercatDeviceRos::workerDispatch() not implemented for this device type.");
        }

        /**
         * @brief commandCallback - callback for the command topic. Updates
         * the staged command of the device used by the worker thread.
        */
        void commandCallback(const ethercat_motor_msgs::MotorCtrlMessage::ConstPtr& msg) {
            std::lock_guard<std::recursive_mutex> lock(*command_msg_mutex_ptr_);
            last_command_msg_ptr_->header.stamp = msg->header.stamp;
            last_command_msg_ptr_->header.frame_id = msg->header.frame_id;
            last_command_msg_ptr_->header.seq = msg->header.seq;
            last_command_msg_ptr_->targetPosition = msg->targetPosition;
            last_command_msg_ptr_->targetVelocity = msg->targetVelocity;
            last_command_msg_ptr_->targetTorque = msg->targetTorque;
            last_command_msg_ptr_->positionOffset = msg->positionOffset;
            last_command_msg_ptr_->velocityOffset = msg->velocityOffset;
            last_command_msg_ptr_->torqueOffset = msg->torqueOffset;
            last_command_msg_ptr_->motionProfileType = msg->motionProfileType;
            last_command_msg_ptr_->timestamp = msg->timestamp;
            last_command_msg_ptr_->version = msg->version;
            last_command_msg_ptr_->profileAcceleration = msg->profileAcceleration;
            last_command_msg_ptr_->profileDeceleration = msg->profileDeceleration;
            last_command_msg_ptr_->operationMode = msg->operationMode;
        }
    
        std::shared_ptr<DeviceClass> device_ptr_; // Shared pointer to slave.
                                         // because Ethercat master needs it too.
        std::shared_ptr<ros::NodeHandle> nh_ptr_; // A shared pointer to node handle for pubs and subs
        EthercatSlaveEntry device_info_; // Slave info
        std::unique_ptr<ros::Subscriber> command_sub_ptr_; // A unique pointer to command subscriber
        std::unique_ptr<ros::Publisher> reading_pub_ptr_; // A unique pointer to reading publisher
        std::unique_ptr<ethercat_motor_msgs::MotorCtrlMessage> last_command_msg_ptr_; // A unique pointer to latest command message received
        std::unique_ptr<std::recursive_mutex> command_msg_mutex_ptr_; // A unique pointer to mutex for command message callback rw locks.
        ethercat_motor_msgs::MotorStatusMessage reading_msg_; // make this a pointer too?
        bool device_enabled_ = false;
        volatile std::atomic<bool> abrt = false;
        bool worker_loop_running_ = false;

        // NOTE: One can also make the command message an atomic type since all the ROS msg fields are
        // generally trivially copyable structs. Not implemented for now, but maybe in the future to avoid mutex locking.
};

/**
 * @brief An object factory for allowing easy addition of new device types
 * through class registrations. It also simplifies the process of device instance creations
 * for the configurator program thus removing the need to loop over all device types and
 * choosing a constructor manually. Design for this class was inspired by the factory design pattern
 * implementation in the nori2 renderer.
*/
class EthercatDeviceFactory {
    public:
        typedef std::function<std::shared_ptr<EthercatDeviceRosBase> (const std::shared_ptr<ros::NodeHandle> &,
                                                 const EthercatSlaveEntry &)> DeviceCreator;
        
        static void registerDevice(const EthercatSlaveType &type, const DeviceCreator &constructor){
            if(device_constructors_ == nullptr){
                device_constructors_ = new std::map<EthercatSlaveType, DeviceCreator>();
            }
            
            (*device_constructors_)[type] = constructor;
        }

        static std::shared_ptr<EthercatDeviceRosBase> createDevice(
            const std::shared_ptr<ros::NodeHandle> &nh_ptr,
            const EthercatSlaveEntry &device_info){

            EthercatSlaveType type = device_info.type;
                
            if(!device_constructors_ || device_constructors_->find(type) == device_constructors_->end()){
                ROS_ERROR_STREAM("[EthercatDeviceFactory::createDevice] Tried to create a device type that was not registered.");
                return nullptr;
            }
            else{
                return (*device_constructors_)[type](nh_ptr, device_info);
            }
        }
    protected:
        static std::map<EthercatSlaveType, DeviceCreator> *device_constructors_;
};

// std::map<EthercatSlaveType, EthercatDeviceFactory::DeviceCreator> *EthercatDeviceFactory::device_constructors_ = nullptr;

#define ETHERCAT_ROS_REGISTER_DEVICE(type, cls) \
    std::shared_ptr<cls> cls ##_create (const std::shared_ptr<ros::NodeHandle> &nh_ptr, \
                                        const EthercatSlaveEntry &device_info) { \
        return std::make_shared<cls>(nh_ptr, device_info); \
    } \
    static struct cls ##_register{\
        cls ##_register() {\
            EthercatDeviceFactory::registerDevice(type, cls ##_create);\
        } \
    } cls ##_register_instance;

} // namespace EthercatRos