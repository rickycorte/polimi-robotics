cmake_minimum_required(VERSION 3.0.2)
project(rb1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  robotics_hw1
  message_filters
  tf2
  tf2_ros
  message_generation
  dynamic_reconfigure
)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   MOdometry.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ResetPose.srv
  ResetToZeroZero.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(

CATKIN_DEPENDS 
  roscpp
  std_msgs 
  geometry_msgs 
  nav_msgs 
  robotics_hw1 
  message_filters
  message_runtime
)

###########
## Build ##
###########


include_directories(
 include ${catkin_INCLUDE_DIRS}
)

# velocity (linear and angular) estimator component
add_executable(rb1_velocity src/velocity.cpp)
add_dependencies(rb1_velocity  ${catkin_EXPORTED_TARGETS})
target_link_libraries(rb1_velocity  ${catkin_LIBRARIES})

# estimator of baseline
add_executable(rb1_baseline src/baseline.cpp)
add_dependencies(rb1_baseline  ${catkin_EXPORTED_TARGETS})
target_link_libraries(rb1_baseline  ${catkin_LIBRARIES})


# odometry component
add_executable(rb1_odometry src/odometry.cpp)
add_dependencies(rb1_odometry rb1_generate_messages_cpp)
add_dependencies(rb1_odometry ${PROJECT_NAME}_gencfg)
add_dependencies(rb1_odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(rb1_odometry  ${catkin_LIBRARIES})


