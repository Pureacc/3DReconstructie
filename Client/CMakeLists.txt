cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(RESTClient2)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (RESTClient2 rest_client2.cpp)
target_link_libraries (RESTClient2 ${PCL_LIBRARIES})