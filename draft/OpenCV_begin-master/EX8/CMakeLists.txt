cmake_minimum_required(VERSION 3.13)
project(EX8)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(myResize myResize.cpp)
target_link_libraries(myResize ${OpenCV_LIBS})
add_executable(comprehensive comprehensive.cpp)
target_link_libraries(comprehensive ${OpenCV_LIBS})