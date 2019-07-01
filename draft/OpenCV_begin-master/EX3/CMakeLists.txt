cmake_minimum_required(VERSION 3.13)
project(EX3)

set(CMAKE_CXX_STANDARD 14)

add_executable(EX3 main.cpp)

find_package(OpenCV REQUIRED)
target_link_libraries(EX3 ${OpenCV_LIBS})#Proj_name是工程名