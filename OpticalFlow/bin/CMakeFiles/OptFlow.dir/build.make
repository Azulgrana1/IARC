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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yuhan/IARC/OpticalFlow

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuhan/IARC/OpticalFlow/bin

# Include any dependencies generated for this target.
include CMakeFiles/OptFlow.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/OptFlow.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OptFlow.dir/flags.make

CMakeFiles/OptFlow.dir/optflow.cpp.o: CMakeFiles/OptFlow.dir/flags.make
CMakeFiles/OptFlow.dir/optflow.cpp.o: ../optflow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuhan/IARC/OpticalFlow/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/OptFlow.dir/optflow.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OptFlow.dir/optflow.cpp.o -c /home/yuhan/IARC/OpticalFlow/optflow.cpp

CMakeFiles/OptFlow.dir/optflow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OptFlow.dir/optflow.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuhan/IARC/OpticalFlow/optflow.cpp > CMakeFiles/OptFlow.dir/optflow.cpp.i

CMakeFiles/OptFlow.dir/optflow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OptFlow.dir/optflow.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuhan/IARC/OpticalFlow/optflow.cpp -o CMakeFiles/OptFlow.dir/optflow.cpp.s

CMakeFiles/OptFlow.dir/optflow.cpp.o.requires:

.PHONY : CMakeFiles/OptFlow.dir/optflow.cpp.o.requires

CMakeFiles/OptFlow.dir/optflow.cpp.o.provides: CMakeFiles/OptFlow.dir/optflow.cpp.o.requires
	$(MAKE) -f CMakeFiles/OptFlow.dir/build.make CMakeFiles/OptFlow.dir/optflow.cpp.o.provides.build
.PHONY : CMakeFiles/OptFlow.dir/optflow.cpp.o.provides

CMakeFiles/OptFlow.dir/optflow.cpp.o.provides.build: CMakeFiles/OptFlow.dir/optflow.cpp.o


# Object files for target OptFlow
OptFlow_OBJECTS = \
"CMakeFiles/OptFlow.dir/optflow.cpp.o"

# External object files for target OptFlow
OptFlow_EXTERNAL_OBJECTS =

OptFlow: CMakeFiles/OptFlow.dir/optflow.cpp.o
OptFlow: CMakeFiles/OptFlow.dir/build.make
OptFlow: /usr/local/lib/libopencv_videostab.so.2.4.13
OptFlow: /usr/local/lib/libopencv_ts.a
OptFlow: /usr/local/lib/libopencv_superres.so.2.4.13
OptFlow: /usr/local/lib/libopencv_stitching.so.2.4.13
OptFlow: /usr/local/lib/libopencv_contrib.so.2.4.13
OptFlow: /usr/lib/x86_64-linux-gnu/libGLU.so
OptFlow: /usr/lib/x86_64-linux-gnu/libGL.so
OptFlow: /usr/local/lib/libopencv_nonfree.so.2.4.13
OptFlow: /usr/local/lib/libopencv_ocl.so.2.4.13
OptFlow: /usr/local/lib/libopencv_gpu.so.2.4.13
OptFlow: /usr/local/lib/libopencv_photo.so.2.4.13
OptFlow: /usr/local/lib/libopencv_objdetect.so.2.4.13
OptFlow: /usr/local/lib/libopencv_legacy.so.2.4.13
OptFlow: /usr/local/lib/libopencv_video.so.2.4.13
OptFlow: /usr/local/lib/libopencv_ml.so.2.4.13
OptFlow: /usr/local/lib/libopencv_calib3d.so.2.4.13
OptFlow: /usr/local/lib/libopencv_features2d.so.2.4.13
OptFlow: /usr/local/lib/libopencv_highgui.so.2.4.13
OptFlow: /usr/local/lib/libopencv_imgproc.so.2.4.13
OptFlow: /usr/local/lib/libopencv_flann.so.2.4.13
OptFlow: /usr/local/lib/libopencv_core.so.2.4.13
OptFlow: /usr/local/cuda/lib64/libnppc.so
OptFlow: /usr/local/cuda/lib64/libnppi.so
OptFlow: /usr/local/cuda/lib64/libnpps.so
OptFlow: /usr/local/cuda/lib64/libcublas.so
OptFlow: /usr/local/cuda/lib64/libcufft.so
OptFlow: CMakeFiles/OptFlow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yuhan/IARC/OpticalFlow/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable OptFlow"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OptFlow.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OptFlow.dir/build: OptFlow

.PHONY : CMakeFiles/OptFlow.dir/build

CMakeFiles/OptFlow.dir/requires: CMakeFiles/OptFlow.dir/optflow.cpp.o.requires

.PHONY : CMakeFiles/OptFlow.dir/requires

CMakeFiles/OptFlow.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/OptFlow.dir/cmake_clean.cmake
.PHONY : CMakeFiles/OptFlow.dir/clean

CMakeFiles/OptFlow.dir/depend:
	cd /home/yuhan/IARC/OpticalFlow/bin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuhan/IARC/OpticalFlow /home/yuhan/IARC/OpticalFlow /home/yuhan/IARC/OpticalFlow/bin /home/yuhan/IARC/OpticalFlow/bin /home/yuhan/IARC/OpticalFlow/bin/CMakeFiles/OptFlow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/OptFlow.dir/depend

