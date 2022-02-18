cmake_minimum_required(VERSION 3.0.2)
project(rb2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(

CATKIN_DEPENDS 
  roscpp
  std_msgs 
)

###########
## Build ##
###########


include_directories(
 include ${catkin_INCLUDE_DIRS}
)




