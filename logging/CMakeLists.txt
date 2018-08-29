cmake_minimum_required(VERSION 2.8)
add_definitions(-std=c++11)

include_directories("${PROJECT_BINARY_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(logging SHARED Logger.cpp)
SET(LIBS ${LIBS} logging PARENT_SCOPE)
