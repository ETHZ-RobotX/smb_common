cmake_minimum_required(VERSION 3.1.3)
project(smb_msf_graph)

set(CATKIN_PACKAGE_DEPENDENCIES
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)

catkin_python_setup()

#############
## Install ##
#############

install(
  TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Python nodes
catkin_install_python(PROGRAMS
  smb_msf_graph/smb_msf_graph/plotting_node.py
  smb_msf_graph/smb_msf_graph/imu_noisifier.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
