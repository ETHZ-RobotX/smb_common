cmake_minimum_required(VERSION 3.5.1)
project(smb)

add_definitions(-std=c++17)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

#############
## Install ##
#############

install(DIRECTORY launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
