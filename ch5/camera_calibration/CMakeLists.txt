cmake_minimum_required( VERSION 2.8 )
project( cameraCalib )

# Add C++ 11 Standard support
set( CMAKE_CXX_FLAGS "-std=c++11" )


# OpenCV include directory
set( OpenCV_DIR /usr/local/lib )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add an executable file
add_executable( cameraCalib cameraCalib.cpp )
target_link_libraries( cameraCalib ${OpenCV_LIBS} )

