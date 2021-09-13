cmake_minimum_required( VERSION 2.8 )
project( E2Rt )
# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 为使用 sophus，您需要使用find_package命令找到它
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( E2Rt E2Rt.cpp )
target_link_libraries( E2Rt ${Sophus_LIBRARIES} )
