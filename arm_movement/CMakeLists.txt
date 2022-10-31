cmake_minimum_required(VERSION 3.0.2)
project(arm_movement)

find_package(catkin REQUIRED COMPONENTS
  rospy
  spot_arm_description
  spot_moveit_config
  std_msgs
)


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm_movement
#  CATKIN_DEPENDS rospy spot_arm_description spot_moveit_config std_msgs
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


