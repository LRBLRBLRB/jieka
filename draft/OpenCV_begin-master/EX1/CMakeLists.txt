cmake_minimum_required(VERSION 3.13)
project(Ex1)

set(CMAKE_CXX_STANDARD 14)

add_executable(Ex1 main.cpp)

find_package(OpenCV REQUIRED)
target_link_libraries(Ex1 ${OpenCV_LIBS})