cmake_minimum_required(VERSION 3.10)
project(opevcv)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(calib main.cpp )
target_link_libraries(calib ${OpenCV_LIBS})
include_directories(${EIGEN3_INCLUDE_DIRS})
