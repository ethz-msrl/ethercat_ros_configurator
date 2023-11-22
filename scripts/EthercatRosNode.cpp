#include <ethercat_ros_configurator/EthercatDeviceConfigurator.hpp>

#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#include <nanotec_ethercat_sdk/Nanotec.hpp>

#include <thread>
#include <csignal>

std::unique_ptr<std::thread> worker_thread;
std::unique_ptr<ros::AsyncSpinner> spinner;
int nthreads = 4; // Number of threads to use for the spinner.
bool abrt = false;

EthercatRos::EthercatDeviceConfigurator::SharedPtr configurator;

void worker()
{
    bool rtSuccess = true;
    long long int iteration_count = 0;
    for(const auto & master: configurator->getMasters())
    {
        rtSuccess &= master->setRealtimePriority(99);
    }
    std::cout << "Setting RT Priority: " << (rtSuccess? "successful." : "not successful. Check user privileges.") << std::endl;

    // Start the worker threads for all the devices
    for(const auto & slave: configurator->getSlaves())
    {
        slave->startWorkerThread();
    }

    if(ros::param::has("nthreads")){
        ros::param::get("nthreads", nthreads);
    }

    spinner = std::make_unique<ros::AsyncSpinner>(nthreads);

    spinner->start();

    while(!abrt) {
        for(const auto & master: configurator->getMasters() )
        {
            master->update(ecat_master::UpdateMode::StandaloneEnforceRate); // TODO fix the rate compensation (Elmo reliability problem)!!
        }
    }
}

/*
** Handle the interrupt signal.
** This is the shutdown routine.
** Note: This logic is executed in a thread separated from the communication update!
 */
void signal_handler(int sig)
{
    /*
    ** Pre shutdown procedure.
    ** The devices execute procedures (e.g. state changes) that are necessary for a
    ** proper shutdown and that must be done with PDO communication.
    ** The communication update loop (i.e. PDO loop) continues to run!
    ** You might thus want to implement some logic that stages zero torque / velocity commands
    ** or simliar safety measures at this point using e.g. atomic variables and checking them
    ** in the communication update loop.
     */
    for(const auto & master: configurator->getMasters())
    {
        master->preShutdown();
    }

    // stop the PDO communication at the next update of the communication loop
    abrt = true;
    worker_thread->join();
    for(const auto &slave : configurator->getSlaves())
    {
        slave->joinWorkerThread();
    }


    /*
    ** Completely halt the EtherCAT communication.
    ** No online communication is possible afterwards, including SDOs.
     */
    for(const auto & master: configurator->getMasters())
    {
        master->shutdown();
    }

    spinner->stop();
    spinner.reset();

    // Exit this executable
    std::cout << "Shutdown" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}



/*
** Program entry.
** Pass the path to the setup.yaml file as first command line argument.
 */
int main(int argc, char**argv)
{
    // Set the abrt_ flag upon receiving an interrupt signal (e.g. Ctrl-c)
    std::signal(SIGINT, signal_handler);
    // ROS Initialization
    ros::init(argc, argv, "ethercat_ros_configurator");

    if(argc < 2)
    {
        std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
        return EXIT_FAILURE;
    }
    // a new EthercatDeviceConfigurator object (path to setup.yaml as constructor argument)
    // The contructor initailizes all the slaves, the masters, and the ROS Nodehandle.
    configurator = std::make_shared<EthercatRos::EthercatDeviceConfigurator>(argv[1]);

    /*
    ** Start all masters.
    ** There is exactly one bus per master which is also started.
    ** All online (i.e. SDO) configuration is done during this call.
    ** The EtherCAT interface is active afterwards, all drives are in Operationaslavel
    ** EtherCAT state and PDO communication may begin.
     */
    for(auto & master: configurator->getMasters())
    {
        if(!master->startup())
        {
            std::cerr << "Startup not successful." << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Start the PDO loop in a new thread.
    worker_thread = std::make_unique<std::thread>(&worker);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "Startup finished" << std::endl;

    // nothing further to do in this thread.
    pause();
}
