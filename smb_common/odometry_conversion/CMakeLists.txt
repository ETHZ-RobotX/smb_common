cmake_minimum_required(VERSION 3.10.2)
project(odometry_conversion)

set(PACKAGE_DEPENDENCIES
        roscpp
        nav_msgs
        tf2_ros
        eigen_conversions
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Eigen3 REQUIRED NO_MODULE)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${PACKAGE_DEPENDENCIES}
)

add_definitions("--std=c++17 -Wall")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}}
)

add_library(${PROJECT_NAME}
        src/OdometryConversion.cpp
        )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(${PROJECT_NAME}_node
        src/odometry_conversion_node.cpp
        )
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
