cmake_minimum_required(VERSION 3.10.2)
project(smb_exploration)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} src/ExplorationMap.cpp)

cs_add_executable(exploration_map_node src/exploration_map_node.cpp)
target_link_libraries(exploration_map_node ${PROJECT_NAME})

cs_install()
cs_export(LIBRARIES)