cmake_minimum_required( VERSION 2.8 )
project( useGeometry )

# Add Eigen header file
include_directories( "/usr/include/eigen3" )

# Add an executable file
add_executable( robot robot_1_and_2.cpp )
