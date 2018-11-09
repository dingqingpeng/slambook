cmake_minimum_required( VERSION 2.8 )
project( imageBasics )

# Add C++ 11 Standard support
set( CMAKE_CXX_FLAGS "-std=c++11" )

set( OpenCV_DIR /usr/local/lib )

# Look for OpenCV Library
find_package( OpenCV REQUIRED )

# Add OpenCV header file
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add an executable file
add_executable( imageBasics imageBasics.cpp )
target_link_libraries( imageBasics ${OpenCV_LIBS} )

add_executable( test test.cpp )
