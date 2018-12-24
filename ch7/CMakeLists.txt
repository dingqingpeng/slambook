cmake_minimum_required( VERSION 2.8 )
project( VO1 )

# Add C++ 11 Standard support
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )
find_package( OpenCV 3.3 REQUIRED )

include_directories( 
    "/usr/include/eigen3/"
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# Add an executable file
add_executable( feature_extraction feature_extraction.cpp )
target_link_libraries( feature_extraction ${OpenCV_LIBS} )

add_executable( pose_2d2d pose_estimation_2d2d.cpp vo1_funcs.cpp )
target_link_libraries( pose_2d2d 
   ${OpenCV_LIBS}
   ${CSPARSE_LIBRARY}
   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)

add_executable( triangulation triangulation.cpp vo1_funcs.cpp )
target_link_libraries( triangulation
   ${OpenCV_LIBS}
   ${CSPARSE_LIBRARY}
   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)

add_executable( pose_3d2d pose_estimation_3d2d.cpp vo1_funcs.cpp )
target_link_libraries( pose_3d2d 
    ${OpenCV_LIBS}
    ${CSPARSE_LIBRARY}
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)


