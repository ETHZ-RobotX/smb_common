cmake_minimum_required(VERSION 3.10.2)
project(smb_slam)


set(CMAKE_CXX_STANDARD 14)


set(PACKAGE_DEPENDENCIES
        roscpp
        nav_msgs
        pointmatcher_ros
        roslib
        )


set(SRC_FILES
  src/PointcloudStitcher.cpp
)

find_package(libpointmatcher REQUIRED)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
        INCLUDE_DIRS
          include 
        LIBRARIES
          ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  SYSTEM
    ${libpointmatcher_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

link_directories( #only needed for pcl
  ${PCL_LIBRARY_DIRS}
)

add_definitions( #only needed for pcl
  ${PCL_DEFINITIONS}
)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

add_executable(pointcloud_stitcher_node
  src/pointcloud_stitcher_node.cpp
)

target_link_libraries(pointcloud_stitcher_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)