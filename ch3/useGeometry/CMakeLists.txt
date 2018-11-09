cmake_minimum_required( VERSION 2.8 )
project( useGeometry )

# Add Eigen header file
include_directories( "/usr/include/eigen3" )

# Add an executable file
add_executable( useGeometry useGeometry.cpp )
add_executable( extractBlock extractBlock.cpp )
