cmake_minimum_required(VERSION 3.1.3)
project(smb_msf)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    msf_updates
    msf_core
)

catkin_python_setup()

#############
## Install ##
#############

install(DIRECTORY
  param
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  smb_msf/plotting_node.py
  smb_msf/pose_noisifier.py
  smb_msf/imu_noisifier.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
