cmake_minimum_required( VERSION 2.8 )
project( ceres_curve_fitting )

# Add C++ 11 Standard support
set( CMAKE_CXX_FLAGS "-std=c++11" )

#list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# Eigen include directory
include_directories( "/usr/include/eigen3" )

# Ceres include directory
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV include directory
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add an executable file
add_executable( ceresFitting ceresFitting.cpp )
target_link_libraries( ceresFitting ${CERES_LIBRARIES} ${OpenCV_LIBS} )
