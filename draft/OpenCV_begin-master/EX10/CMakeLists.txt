cmake_minimum_required(VERSION 3.13)
project(EX10)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS})
add_executable(advanced advanced.cpp)
target_link_libraries(advanced ${OpenCV_LIBS})