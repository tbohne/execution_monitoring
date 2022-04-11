cmake_minimum_required(VERSION 3.0.2)
project(execution_monitoring)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  message_generation
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  WiFi.msg
  Internet.msg
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES Scan.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES execution_monitoring
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/high_level_smach.py
  scripts/operation.py
  scripts/scanning/dummy_scanner.py
  scripts/monitoring/sensor_monitoring.py
  scripts/monitoring/data_monitoring.py
  scripts/monitoring/connection_monitoring.py
  scripts/resolution/resolver.py
  scripts/scanning/republish_velodyne.py
  scripts/monitoring/operator_communication.py
  scripts/monitoring/internet_monitoring.py
  scripts/monitoring/weather_monitoring.py
  scripts/monitoring/localization_monitoring.py
  scripts/simulation/gnss_simulator.py
  scripts/monitoring/os_specific/wifi_monitor.py
  scripts/experiments/experiments.py
  scripts/monitoring/plan_deployment_monitoring.py
  scripts/monitoring/navigation_monitoring.py
  scripts/simulation/obstacle_spawner.py
  scripts/monitoring/charging_failure_monitoring.py
  scripts/experiments/data_accumulator.py
  scripts/monitoring/power_management_monitoring.py
  scripts/simulation/battery_failure_simulator.py
  scripts/monitoring/mission_monitoring.py
  scripts/simulation/charging_failure_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
