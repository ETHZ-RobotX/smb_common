cmake_minimum_required(VERSION 3.1.3)
project(smb_msf)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    msf_updates

)

install(DIRECTORY
  param
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
