cmake_minimum_required(VERSION 3.10.2)
project(smb_slam)


set(CMAKE_CXX_STANDARD 14)

set(PACKAGE_DEPENDENCIES)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${PACKAGE_DEPENDENCIES}
)
