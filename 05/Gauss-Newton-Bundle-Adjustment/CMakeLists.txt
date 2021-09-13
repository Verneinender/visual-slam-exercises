cmake_minimum_required( VERSION 2.8 )
project( GN-BA )
# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 为使用 sophus，您需要使用find_package命令找到它
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( GN-BA GN-BA.cpp )
target_link_libraries( GN-BA ${Sophus_LIBRARIES} )
