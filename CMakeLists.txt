cmake_minimum_required(VERSION 3.0.2)
# MODIFIED: The project name must match your package folder name
project(allergen_scanner)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   TriggerScan.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

## catkin specific configuration
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
