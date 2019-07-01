cmake_minimum_required(VERSION 3.13)
project(EX7)

set(CMAKE_CXX_STANDARD 14)

Find_package(OpenCV REQUIRED)

add_executable(canny Canny.cpp )
target_link_libraries(canny ${OpenCV_LIBS})

add_executable(sobel Sobel.cpp )
target_link_libraries(sobel ${OpenCV_LIBS})
add_executable(laplace Laplace.cpp )
target_link_libraries(laplace ${OpenCV_LIBS})
add_executable(combine Combine.cpp )
target_link_libraries(combine ${OpenCV_LIBS})