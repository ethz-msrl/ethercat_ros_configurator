
#include <ethercat_ros_configurator/EthercatDeviceConfigurator.hpp>
#include <ethercat_motor_msgs/MotorCtrlMessage.h>
#include <ethercat_motor_msgs/MotorStatusMessage.h>
#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#include <nanotec_ethercat_sdk/Nanotec.hpp>
#include <ros/ros.h>

#include <thread>
#include <csignal>
#include <chrono>
#include <mutex>

namespace EthercatRos{

enum class EthercatSlaveType
{
    NA,
    Maxon,
    Nanotec
};

struct EthercatSlaveEntry
{
    EthercatSlaveType type = EthercatSlaveType::NA;
    std::string name = "";
    std::string config_file_path = "";

    uint32_t ethercat_address = 0;
    std::string ethercat_bus = "";
    std::string ethercat_pdo_type = "";
};


class EthercatDeviceRosBase{
    public:
        virtual ~EthercatDeviceRosBase() = default;
        virtual void startWorkerThread() = 0;
        virtual void joinWorkerThread() = 0;
        virtual std::shared_ptr<ecat_master::EthercatDevice> getSlaveObjPtr() const = 0;
        virtual std::string getName() const = 0;
        virtual void abort() = 0;
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
        EthercatDeviceRos(std::shared_ptr<ros::NodeHandle> &nh_ptr, 
                          const std::shared_ptr<ecat_master::EthercatDevice> &slave, 
                          EthercatSlaveEntry &device_info){
            nh_ptr_ = nh_ptr;
            device_info_ = device_info;
            device_ptr_ = std::dynamic_pointer_cast<DeviceClass>(slave);

            command_sub_ptr_ = std::make_unique<ros::Subscriber>(
                nh_ptr_->subscribe<ethercat_motor_msgs::MotorCtrlMessage>(device_info.name + "/command", 1000, &EthercatDeviceRos::commandCallback, this)
                );
            
            reading_pub_ptr_ = std::make_unique<ros::Publisher>(
                nh_ptr_->advertise<ethercat_motor_msgs::MotorStatusMessage>(device_info.name + "/reading", 1000)
                );
            
            last_command_msg_ptr_ = std::make_unique<ethercat_motor_msgs::MotorCtrlMessage>();
            command_msg_mutex_ptr_ = std::make_unique<std::recursive_mutex>();

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
            other.device_info_ = EthercatSlaveEntry();
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
            ROS_DEBUG("EthercatDevice %s aborting.", device_info_.name.c_str());
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
        
    protected:
        std::unique_ptr<std::thread> worker_thread_ptr_;

        void worker() {
            
            // Worker thread retains the style of standalone.cpp because of software
            // architecture reasons. Slave type specific operations are possible here.
            // Specify this in device specific specializations of workerDispatch().

            ROS_ERROR("EthercatDeviceRos::workerDispatch() not implemented for this device type.");
        }

        void checkSlaveTypeAvailability(){
            if (device_info_.type == EthercatSlaveType::Maxon) {
                if (device_ptr_ == nullptr) {
                    ROS_ERROR_STREAM("Maxon '" << device_info_.name << "' pointer not available to device class.");
                    return;
                }
            }
            else if (device_info_.type == EthercatSlaveType::Nanotec) {
                if (device_ptr_ == nullptr) {
                    ROS_ERROR_STREAM("Nanotec '" << device_info_.name << "' pointer not available to device class.");
                    return;
                }
            }
            else {
                ROS_ERROR_STREAM("[EthercatDeviceRos::checkSlaveTypeAvailability()] Ethercat ROS configurator not implemented for this device type.");
            }
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
        }
    
        std::shared_ptr<DeviceClass> device_ptr_; // Shared pointer to slave.
                                         // because Ethercat master needs it too.
        std::shared_ptr<ros::NodeHandle> nh_ptr_; // A shared pointer to node handle for pubs and subs
        EthercatSlaveEntry device_info_; // Slave info
        std::unique_ptr<ros::Subscriber> command_sub_ptr_; // A unique pointer to command subscriber
        std::unique_ptr<ros::Publisher> reading_pub_ptr_; // A unique pointer to reading publisher
        std::unique_ptr<ethercat_motor_msgs::MotorCtrlMessage> last_command_msg_ptr_; // A unique pointer to latest command message received
        // Maybe the command_msg_mutex doesn't need to be a pointer.
        std::unique_ptr<std::recursive_mutex> command_msg_mutex_ptr_; // A unique pointer to mutex for command message callback rw locks.
        ethercat_motor_msgs::MotorStatusMessage reading_msg_; // make this a pointer too?
        bool device_enabled_ = false;
        volatile std::atomic<bool> abrt = false;
        bool worker_loop_running_ = false;
        uint32_t reading_pub_freq = 500; // Hz

        // NOTE: One can also make the command message an atomic type since all the ROS msg fields are
        // generally trivially copyable structs. Not implemented for now, but maybe in the future to avoid mutex locking.
};

// --------> DEVICE SPECIFIC SPECIALIZATIONS <--------

// --------> Maxon
template <>
void EthercatDeviceRos<maxon::Maxon>::worker(){
    ROS_INFO_STREAM("Maxon '" << device_ptr_->getName() << "': Worker thread started.");
    ros::Rate loop_rate(reading_pub_freq);
    worker_loop_running_ = true;
    std::unique_lock<std::recursive_mutex> lock(*command_msg_mutex_ptr_);
    lock.unlock();
    while(!abrt){
        if (device_info_.type == EthercatSlaveType::Maxon) {
            if(!device_enabled_){
                device_ptr_->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, false);
                // Small delay to allow the PDO state change flag to be set. Due to the min number
                // of succesful PDO state readings check taking some time.
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            maxon::Reading reading;
            reading_msg_.header.stamp = ros::Time::now();
            reading_msg_.actualPosition = reading.getActualPositionRaw();
            reading_msg_.actualVelocity = reading.getActualVelocityRaw();
            reading_msg_.statusword = reading.getRawStatusword();
            reading_msg_.analogInput = reading.getAnalogInputRaw();
            reading_msg_.busVoltage = reading.getBusVoltageRaw();
            reading_msg_.actualTorque   = reading.getActualCurrent(); // see txPDO defn in maxon's SDK
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
                cmd.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode);
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
        else{
            ROS_ERROR("[EthercatDeviceRos::workerDispatch] Device type and template specialization mismatch.");
            checkSlaveTypeAvailability();
        }
    }
    worker_loop_running_ = false;
    return;
}
// <-------- Maxon

// --------> Nanotec
template <>
void EthercatDeviceRos<nanotec::Nanotec>::worker() {
    ROS_INFO_STREAM("Nanotec '" << device_ptr_->getName() << "': Worker thread started.");
    ros::Rate loop_rate(reading_pub_freq);
    worker_loop_running_ = true;
    std::unique_lock<std::recursive_mutex> lock(*command_msg_mutex_ptr_);
    lock.unlock();
    while(!abrt){
        if (device_info_.type == EthercatSlaveType::Nanotec) {
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
                cmd.setModeOfOperation(nanotec::ModeOfOperationEnum::CyclicSynchronousPositionMode);
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
        else{
            ROS_ERROR("[EthercatDeviceRos::workerDispatch] Device type and template specialization mismatch.");
            checkSlaveTypeAvailability();
        }
    }
    worker_loop_running_ = false;
    return;
}
// <-------- Nanotec

} // namespace EthercatRos