cmake_minimum_required( VERSION 2.8 )
project( myslam )

# Add C++ 11 Standard support
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )
find_package( OpenCV 3.3 REQUIRED )
find_package( Sophus REQUIRED )

include_directories( 
    "/usr/include/eigen3/"
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

# Add an executable file
#add_executable( pose_2d2d pose_estimation_2d2d.cpp vo1_funcs.cpp )
#target_link_libraries( pose_2d2d 
#   ${OpenCV_LIBS}
#   ${CSPARSE_LIBRARY}
#   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
#)
