cmake_minimum_required(VERSION 3.0.2)
project(smb_control)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  controller_manager
  joint_state_controller
  robot_state_publisher
  roslaunch
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  config/PIDConfig.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure controller_manager joint_state_controller robot_state_publisher roscpp std_msgs geometry_msgs sensor_msgs dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pid_controller_node src/PIDControllerNode.cpp)
add_dependencies(pid_controller_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})  # Ensure the node is built after generating dynamic reconfigure headers

target_link_libraries(pid_controller_node
   ${EXTERNAL_LIBS}
   ${catkin_LIBRARIES}
)

roslaunch_add_file_check(launch)

install(DIRECTORY config launch include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

