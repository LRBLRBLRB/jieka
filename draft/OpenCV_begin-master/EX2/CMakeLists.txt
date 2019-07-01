cmake_minimum_required(VERSION 3.13)
project(EX2)

set(CMAKE_CXX_STANDARD 14)

add_executable(EX2 main.cpp)
find_package(OpenCV REQUIRED)
target_link_libraries(EX2 ${OpenCV_LIBS})