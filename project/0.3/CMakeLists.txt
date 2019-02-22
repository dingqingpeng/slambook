cmake_minimum_required( VERSION 2.8 )
project( myslam )

# Add C++ 11 Standard support
set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

find_package( G2O REQUIRED )
#find_package( CSparse REQUIRED )
find_package( OpenCV 3.3 REQUIRED )
find_package( Sophus REQUIRED )

include_directories( 
    "/usr/include/eigen3/"
    ${G2O_INCLUDE_DIRS}
#    ${CSPARSE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)

add_subdirectory( src )
add_subdirectory( test )
