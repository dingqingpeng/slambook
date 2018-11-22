cmake_minimum_required( VERSION 2.8 )
project( VO1 )

# Add C++ 11 Standard support
set( CMAKE_CXX_FLAGS "-std=c++11" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# Eigen include directory
#include_directories( "/usr/include/eigen3" )

# g2o include directory
#find_package( G2O REQUIRED )
#include_directories( ${G2O_INCLUDE_DIRS} )

# OpenCV include directory
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add an executable file
add_executable( feature_extraction feature_extraction.cpp )
target_link_libraries( feature_extraction ${OpenCV_LIBS} )

add_executable( pose_2d2d pose_estimation_2d2d.cpp vo1_funcs.cpp )
target_link_libraries( pose_2d2d ${OpenCV_LIBS} )

add_executable( triangulation triangulation.cpp vo1_funcs.cpp )
target_link_libraries( triangulation ${OpenCV_LIBS} )
