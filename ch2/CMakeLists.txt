# Declare minimum version of cmake required
cmake_minimum_required( VERSION 2.8 )

# Declare a cmake project
project( HelloSLAM )

# Add an executable file
add_executable( helloSLAM helloSLAM.cpp )

# Add a library
add_library( hello_shared SHARED libHelloSLAM.cpp )

add_executable( useHello useHello.cpp )
# Link a lib to a executable file
target_link_libraries( useHello hello_shared )
