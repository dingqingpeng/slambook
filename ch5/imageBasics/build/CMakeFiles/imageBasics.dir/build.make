# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dingqingpeng/Documents/SLAM/SRC/ch5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dingqingpeng/Documents/SLAM/SRC/ch5/build

# Include any dependencies generated for this target.
include CMakeFiles/imageBasics.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/imageBasics.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/imageBasics.dir/flags.make

CMakeFiles/imageBasics.dir/imageBasics.cpp.o: CMakeFiles/imageBasics.dir/flags.make
CMakeFiles/imageBasics.dir/imageBasics.cpp.o: ../imageBasics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dingqingpeng/Documents/SLAM/SRC/ch5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/imageBasics.dir/imageBasics.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imageBasics.dir/imageBasics.cpp.o -c /home/dingqingpeng/Documents/SLAM/SRC/ch5/imageBasics.cpp

CMakeFiles/imageBasics.dir/imageBasics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imageBasics.dir/imageBasics.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dingqingpeng/Documents/SLAM/SRC/ch5/imageBasics.cpp > CMakeFiles/imageBasics.dir/imageBasics.cpp.i

CMakeFiles/imageBasics.dir/imageBasics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imageBasics.dir/imageBasics.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dingqingpeng/Documents/SLAM/SRC/ch5/imageBasics.cpp -o CMakeFiles/imageBasics.dir/imageBasics.cpp.s

CMakeFiles/imageBasics.dir/imageBasics.cpp.o.requires:

.PHONY : CMakeFiles/imageBasics.dir/imageBasics.cpp.o.requires

CMakeFiles/imageBasics.dir/imageBasics.cpp.o.provides: CMakeFiles/imageBasics.dir/imageBasics.cpp.o.requires
	$(MAKE) -f CMakeFiles/imageBasics.dir/build.make CMakeFiles/imageBasics.dir/imageBasics.cpp.o.provides.build
.PHONY : CMakeFiles/imageBasics.dir/imageBasics.cpp.o.provides

CMakeFiles/imageBasics.dir/imageBasics.cpp.o.provides.build: CMakeFiles/imageBasics.dir/imageBasics.cpp.o


# Object files for target imageBasics
imageBasics_OBJECTS = \
"CMakeFiles/imageBasics.dir/imageBasics.cpp.o"

# External object files for target imageBasics
imageBasics_EXTERNAL_OBJECTS =

imageBasics: CMakeFiles/imageBasics.dir/imageBasics.cpp.o
imageBasics: CMakeFiles/imageBasics.dir/build.make
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
imageBasics: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
imageBasics: CMakeFiles/imageBasics.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dingqingpeng/Documents/SLAM/SRC/ch5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable imageBasics"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imageBasics.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/imageBasics.dir/build: imageBasics

.PHONY : CMakeFiles/imageBasics.dir/build

CMakeFiles/imageBasics.dir/requires: CMakeFiles/imageBasics.dir/imageBasics.cpp.o.requires

.PHONY : CMakeFiles/imageBasics.dir/requires

CMakeFiles/imageBasics.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imageBasics.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imageBasics.dir/clean

CMakeFiles/imageBasics.dir/depend:
	cd /home/dingqingpeng/Documents/SLAM/SRC/ch5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dingqingpeng/Documents/SLAM/SRC/ch5 /home/dingqingpeng/Documents/SLAM/SRC/ch5 /home/dingqingpeng/Documents/SLAM/SRC/ch5/build /home/dingqingpeng/Documents/SLAM/SRC/ch5/build /home/dingqingpeng/Documents/SLAM/SRC/ch5/build/CMakeFiles/imageBasics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imageBasics.dir/depend

