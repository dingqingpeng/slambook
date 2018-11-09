cmake_minimum_required( VERSION 2.8 )
project( useSophus )

# Add Eigen header file
include_directories( "/usr/include/eigen3" )

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# Add an executable file
add_executable( useSophus useSophus.cpp )
target_link_libraries( useSophus ${Sophus_LIBRARIES} )
