#pragma once
#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#include <nanotec_ethercat_sdk/Nanotec.hpp>
#include <ros/ros.h>

namespace EthercatRos{
    
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

} // namespace EthercatRos