cmake_minimum_required(VERSION 2.8)
 
project(undistort_image)
 
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
 
#include_directories("/usr/include/opencv2")
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#find_package(Eigen3 3.1.0 REQUIRED)
#include_directories(${Eigen3_INCLUDE_DIRS})

#set(Sophus_LIBRARIES libSophus.so)
 

 
add_executable(undistort_image undistort_image.cpp)
 
#target_link_libraries(undistort_image )
target_link_libraries(undistort_image ${OpenCV_LIBS} )
