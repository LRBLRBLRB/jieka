cmake_minimum_required(VERSION 3.13)
project(EX4)

set(CMAKE_CXX_STANDARD 14)

add_executable(EX4 main.cpp)
find_package(OpenCV REQUIRED)
target_link_libraries(EX4 ${OpenCV_LIBS})