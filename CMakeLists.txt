cmake_minimum_required( VERSION 2.8 )
project( DenseMapping )

set( CMAKE_BUILD_TYPE "Debug" )
#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O1" )

# OpenCV
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization )
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( DenseMapping DenseMapping.cpp )
target_link_libraries( DenseMapping ${OpenCV_LIBS} ${PCL_LIBRARIES} )
