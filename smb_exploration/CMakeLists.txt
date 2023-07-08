cmake_minimum_required(VERSION 3.10.2)
project(smb_exploration)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_install()
cs_export(LIBRARIES)