cmake_minimum_required(VERSION 2.8)
 
project(ICP)
 
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
 
include_directories("/usr/include/eigen3")
#find_package(Eigen3 3.1.0 REQUIRED)
#include_directories(${Eigen3_INCLUDE_DIRS})

#set(Sophus_LIBRARIES libSophus.so)
 
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
 
add_executable(ICP ICP.cpp)
 
target_link_libraries(ICP 
                    ${Pangolin_LIBRARIES} 
                    ${Sophus_INCLUDE_DIRS} 
                    ${Eigen3_INCLUDE_DIRS}
		            ${Sophus_LIBRARIES}
                    ${OpenCV_LIBS})

