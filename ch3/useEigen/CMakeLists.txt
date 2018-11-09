cmake_minimum_required( VERSION 2.8 )
project( useEigen )

# Add Eigen header file
include_directories( "/usr/include/eigen3" )

# Add an executable file
add_executable(eigenMatrix eigenMatrix.cpp)
