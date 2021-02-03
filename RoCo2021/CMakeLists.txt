cmake_minimum_required(VERSION 3.17)
project(RoCo)

set(CMAKE_CXX_STANDARD 14)

add_executable(RoCo main.cpp)
include_directories(Src)
add_subdirectory(Src)
add_subdirectory(GoogleTest)

add_definitions(-DBUILD_FOR_TESTING)