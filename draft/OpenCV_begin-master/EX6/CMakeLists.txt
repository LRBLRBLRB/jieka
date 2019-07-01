cmake_minimum_required(VERSION 3.13)
project(EX6)

set(CMAKE_CXX_STANDARD 14)

add_executable(ex6 main.cpp)
find_package(OpenCV REQUIRED)
target_link_libraries(ex6 ${OpenCV_LIBS})