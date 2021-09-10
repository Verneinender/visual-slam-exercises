cmake_minimum_required(VERSION 2.8)
 
project(disparity)
 
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
 
#include_directories("/usr/include/eigen3")
find_package(Eigen3 3.1.0 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
 
add_executable(disparity disparity.cpp)
 
target_link_libraries(disparity 
                    ${Pangolin_LIBRARIES}  
                    ${Eigen3_INCLUDE_DIRS}
                    ${OpenCV_LIBS})

