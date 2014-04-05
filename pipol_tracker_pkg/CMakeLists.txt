cmake_minimum_required(VERSION 2.8.3)
project(pipol_tracker_pkg)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
      roscpp rospy std_msgs geometry_msgs image_transport cv_bridge message_generation dynamic_reconfigure)

## System dependencies are found with CMake's conventions
find_package(pipol_tracker_lib REQUIRED)

#SRCS files
SET(SRCS 
      src/xxx.cpp
)

## Add messages to be generated
add_message_files(FILES xx.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

#dynamic reconfigure cfg file
generate_dynamic_reconfigure_options(cfg/xx.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
      CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs image_transport cv_bridge message_runtime dynamic_reconfigure
      DEPENDS pipol_tracker_lib
)

## Specify additional locations of header files
include_directories(${pipol_tracker_lib_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(pipol_tracker_node ${SRCS})
add_dependencies(pipol_tracker_node ${PROJECT_NAME}_EXPORTED_TARGETS)
target_link_libraries(pipol_tracker_node ${pipol_tracker_lib_LIBRARIES} ${catkin_LIBRARIES})
