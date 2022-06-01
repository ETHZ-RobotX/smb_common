cmake_minimum_required(VERSION 3.0.1)
project(smb_mpc)

set(CATKIN_DEPENDENCIES
  roslib
  ocs2_core
  ocs2_robotic_tools
  ocs2_ros_interfaces
  ocs2_ddp
  tf2_ros
  eigen_conversions
)


find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  )

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_DEPENDENCIES}
  DEPENDS
    Boost
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/SmbSystemDynamics.cpp
  src/cost/SmbCost.cpp
  #src/SmbOperatingPoint.cpp
  src/SmbInterface.cpp
  src/SmbConversions.cpp
  src/SmbDefinitions.cpp
  src/SmbController.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
  )
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)


add_executable(${PROJECT_NAME}_node
  src/smb_mpc_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}  
  ${catkin_LIBRARIES}
)

add_executable(smb_controller_node
  src/smb_controller_node.cpp
)
target_link_libraries(smb_controller_node
  ${PROJECT_NAME}  
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gtest(smb_mpc_test
  test/TestSmbInterface.cpp
  )
add_dependencies(smb_mpc_test
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(smb_mpc_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
