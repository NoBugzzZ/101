cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)
find_package(OpenCV REQUIRED)
include_directories(OPENCV2_INCLUDE_DIR)

MESSAGE( STATUS "EIGEN3_INCLUDE_DIR = ${EIGEN3_INCLUDE_DIR}.")
MESSAGE( STATUS "OpenCV_INCLUDE_DIR = ${OpenCV2_INCLUDE_DIR}.")
MESSAGE( STATUS "OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}.")

set(CMAKE_CXX_STANDARD 17)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})

