
cmake_minimum_required(VERSION 3.0)
project(ethercat_ros_configurator)

# Set RPATH to include necessary library paths
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# List of directories to include in RPATH
list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
list(APPEND CMAKE_INSTALL_RPATH "/opt/ros/noetic/lib")
list(APPEND CMAKE_INSTALL_RPATH "$ENV{HOME}/tesla_ws/devel/lib")


find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
endif()

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror)
add_definitions(-DMELO_USE_COUT)


find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED)
find_package(roscpp REQUIRED)
find_package(rospy REQUIRED)
find_package(ethercat_motor_msgs REQUIRED)

find_package(maxon_epos_ethercat_sdk )
find_package(nanotec_ethercat_sdk )


set(SRC_FILES
  # include files
  include/ethercat_ros_configurator/EthercatDeviceConfigurator.hpp
  include/ethercat_ros_configurator/EthercatDeviceRos.hpp

  # source files
  src/ethercat_ros_configurator/EthercatDeviceConfigurator.cpp
)

if(maxon_epos_ethercat_sdk_FOUND)
  add_definitions(-D_MAXON_FOUND_)
  set(PACKAGE_DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
    maxon_epos_ethercat_sdk
  )
  set(SRC_FILES
    ${SRC_FILES}
    src/ethercat_ros_configurator/Maxon.cpp)
endif()

if(nanotec_ethercat_sdk_FOUND)
  add_definitions(-D_NANOTEC_FOUND_)
  set(PACKAGE_DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
    nanotec_ethercat_sdk
  )
  set(SRC_FILES
    ${SRC_FILES}
    src/ethercat_ros_configurator/Nanotec.cpp)
endif()

foreach(sdk ${PACKAGE_DEPENDENCIES})
  message("Found EtherCAT device sdk: " ${sdk})
endforeach()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    ethercat_motor_msgs
)

include_directories(
  include
  ${maxon_epos_ethercat_sdk_INCLUDE_DIRS}
  ${nanotec_ethercat_sdk_INCLUDE_DIRS}
  ${ethercat_motor_msgs_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${maxon_epos_ethercat_sdk_LIBRARIES}
  ${nanotec_ethercat_sdk_LIBRARIES}
  ${ethercat_motor_msgs_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  stdc++fs
)

add_executable(
  ethercat_ros_node
  scripts/EthercatRosNode.cpp
)
add_dependencies(
  ethercat_ros_node
  ${PROJECT_NAME}
  ${maxon_epos_ethercat_sdk_EXPORTED_TARGETS}
  ${nanotec_ethercat_sdk_EXPORTED_TARGETS}
  ${ethercat_motor_msgs_EXPORTED_TARGETS}
)

# For CMake 3.13 and later
target_link_options(ethercat_ros_node PRIVATE -Wl,--disable-new-dtags)

# If using an older version of CMake, use set_target_properties instead
set_target_properties(ethercat_ros_node PROPERTIES
  LINK_FLAGS "-Wl,--disable-new-dtags"
)

target_link_libraries(
  ethercat_ros_node
  ${PROJECT_NAME}
  ${maxon_epos_ethercat_sdk_LIBRARIES}
  ${nanotec_ethercat_sdk_LIBRARIES}
  ${ethercat_motor_msgs_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  -pthread
  stdc++fs
)

catkin_install_python(PROGRAMS scripts/test_reference_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} ethercat_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)