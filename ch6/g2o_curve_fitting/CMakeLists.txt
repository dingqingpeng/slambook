cmake_minimum_required( VERSION 2.8 )
project( g2o_curve_fitting )

# Add C++ 11 Standard support
set( CMAKE_CXX_FLAGS "-std=c++11" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# Eigen include directory
include_directories( "/usr/include/eigen3" )

# g2o include directory
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

# OpenCV include directory
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add an executable file
add_executable( g2oFitting g2oFitting.cpp )
target_link_libraries( g2oFitting g2o_core g2o_stuff ${OpenCV_LIBS} )
