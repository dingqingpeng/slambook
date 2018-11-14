cmake_minimum_required( VERSION 2.8 )
project( cameraCalib )

# Add C++ 11 Standard support
set( CMAKE_CXX_FLAGS "-std=c++11" )

# OpenCV include directory
set( OpenCV_DIR /usr/local/lib )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Two ways to compile multiple source files
# 1. add_executable( fileName source1.cpp  source2.cpp )
#
# 2. aux_source_directory( . source_list )
#    add_executable( fileName ${source_list} )

# Add an executable file
#aux_source_directory( . source_list )
add_executable( cameraCalib cameraCalib.cpp class_Settings.cpp )
add_executable( cameraCalibTutorial camera_calib_opencv_tutorial.cpp )
target_link_libraries( cameraCalib ${OpenCV_LIBS} )
target_link_libraries( cameraCalibTutorial ${OpenCV_LIBS} )
