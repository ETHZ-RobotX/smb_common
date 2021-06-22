cmake_minimum_required(VERSION 3.0.2)
project(smb_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  roslaunch
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES smb_control
#  CATKIN_DEPENDS controller_manager joint_state_controller robot_state_publisher
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
