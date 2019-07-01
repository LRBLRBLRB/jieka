cmake_minimum_required(VERSION 3.13)
project(EX5)

set(CMAKE_CXX_STANDARD 14)

add_executable(EX5 main.cpp)
find_package(OpenCV REQUIRED)
target_link_libraries(EX5 ${OpenCV_LIBS})