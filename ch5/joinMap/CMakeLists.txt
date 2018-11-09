cmake_minimum_required( VERSION 2.8 )
project( joinMap )

# Add C++ 11 Standard support
set( CMAKE_CXX_FLAGS "-std=c++11" )


# OpenCV include directory
set( OpenCV_DIR /usr/local/lib )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Eigen include directory
include_directories( "/usr/include/eigen3/" )

# PCL include directory
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Add an executable file
add_executable( joinMap joinMap.cpp )
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )

