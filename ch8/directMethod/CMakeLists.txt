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

set( G2O_LIBS
    g2o_core
    g2o_stuff
    g2o_types_sba
    g2o_solver_csparse
    g2o_csparse_extension
)

# Add an executable file
add_executable( direct_sparse direct_sparse.cpp )
target_link_libraries( direct_sparse 
   ${OpenCV_LIBS}
   ${CSPARSE_LIBRARY}
   ${G2O_LIBS}
)

add_executable( direct_semidense direct_semidense.cpp )
target_link_libraries( direct_semidense 
   ${OpenCV_LIBS}
   ${CSPARSE_LIBRARY}
   ${G2O_LIBS}
)

