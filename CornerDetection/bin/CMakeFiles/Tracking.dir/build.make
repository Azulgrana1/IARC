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
CMAKE_SOURCE_DIR = /home/yuhan/IARC/CornerDetection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuhan/IARC/CornerDetection/bin

# Include any dependencies generated for this target.
include CMakeFiles/Tracking.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Tracking.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Tracking.dir/flags.make

CMakeFiles/Tracking.dir/VideoFlow.cpp.o: CMakeFiles/Tracking.dir/flags.make
CMakeFiles/Tracking.dir/VideoFlow.cpp.o: ../VideoFlow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuhan/IARC/CornerDetection/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Tracking.dir/VideoFlow.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Tracking.dir/VideoFlow.cpp.o -c /home/yuhan/IARC/CornerDetection/VideoFlow.cpp

CMakeFiles/Tracking.dir/VideoFlow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Tracking.dir/VideoFlow.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuhan/IARC/CornerDetection/VideoFlow.cpp > CMakeFiles/Tracking.dir/VideoFlow.cpp.i

CMakeFiles/Tracking.dir/VideoFlow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Tracking.dir/VideoFlow.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuhan/IARC/CornerDetection/VideoFlow.cpp -o CMakeFiles/Tracking.dir/VideoFlow.cpp.s

CMakeFiles/Tracking.dir/VideoFlow.cpp.o.requires:

.PHONY : CMakeFiles/Tracking.dir/VideoFlow.cpp.o.requires

CMakeFiles/Tracking.dir/VideoFlow.cpp.o.provides: CMakeFiles/Tracking.dir/VideoFlow.cpp.o.requires
	$(MAKE) -f CMakeFiles/Tracking.dir/build.make CMakeFiles/Tracking.dir/VideoFlow.cpp.o.provides.build
.PHONY : CMakeFiles/Tracking.dir/VideoFlow.cpp.o.provides

CMakeFiles/Tracking.dir/VideoFlow.cpp.o.provides.build: CMakeFiles/Tracking.dir/VideoFlow.cpp.o


CMakeFiles/Tracking.dir/HoughCorner.cpp.o: CMakeFiles/Tracking.dir/flags.make
CMakeFiles/Tracking.dir/HoughCorner.cpp.o: ../HoughCorner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuhan/IARC/CornerDetection/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Tracking.dir/HoughCorner.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Tracking.dir/HoughCorner.cpp.o -c /home/yuhan/IARC/CornerDetection/HoughCorner.cpp

CMakeFiles/Tracking.dir/HoughCorner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Tracking.dir/HoughCorner.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuhan/IARC/CornerDetection/HoughCorner.cpp > CMakeFiles/Tracking.dir/HoughCorner.cpp.i

CMakeFiles/Tracking.dir/HoughCorner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Tracking.dir/HoughCorner.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuhan/IARC/CornerDetection/HoughCorner.cpp -o CMakeFiles/Tracking.dir/HoughCorner.cpp.s

CMakeFiles/Tracking.dir/HoughCorner.cpp.o.requires:

.PHONY : CMakeFiles/Tracking.dir/HoughCorner.cpp.o.requires

CMakeFiles/Tracking.dir/HoughCorner.cpp.o.provides: CMakeFiles/Tracking.dir/HoughCorner.cpp.o.requires
	$(MAKE) -f CMakeFiles/Tracking.dir/build.make CMakeFiles/Tracking.dir/HoughCorner.cpp.o.provides.build
.PHONY : CMakeFiles/Tracking.dir/HoughCorner.cpp.o.provides

CMakeFiles/Tracking.dir/HoughCorner.cpp.o.provides.build: CMakeFiles/Tracking.dir/HoughCorner.cpp.o


# Object files for target Tracking
Tracking_OBJECTS = \
"CMakeFiles/Tracking.dir/VideoFlow.cpp.o" \
"CMakeFiles/Tracking.dir/HoughCorner.cpp.o"

# External object files for target Tracking
Tracking_EXTERNAL_OBJECTS =

Tracking: CMakeFiles/Tracking.dir/VideoFlow.cpp.o
Tracking: CMakeFiles/Tracking.dir/HoughCorner.cpp.o
Tracking: CMakeFiles/Tracking.dir/build.make
Tracking: /usr/local/lib/libopencv_videostab.so.2.4.13
Tracking: /usr/local/lib/libopencv_ts.a
Tracking: /usr/local/lib/libopencv_superres.so.2.4.13
Tracking: /usr/local/lib/libopencv_stitching.so.2.4.13
Tracking: /usr/local/lib/libopencv_contrib.so.2.4.13
Tracking: /usr/lib/x86_64-linux-gnu/libGLU.so
Tracking: /usr/lib/x86_64-linux-gnu/libGL.so
Tracking: /usr/local/lib/libopencv_nonfree.so.2.4.13
Tracking: /usr/local/lib/libopencv_ocl.so.2.4.13
Tracking: /usr/local/lib/libopencv_gpu.so.2.4.13
Tracking: /usr/local/lib/libopencv_photo.so.2.4.13
Tracking: /usr/local/lib/libopencv_objdetect.so.2.4.13
Tracking: /usr/local/lib/libopencv_legacy.so.2.4.13
Tracking: /usr/local/lib/libopencv_video.so.2.4.13
Tracking: /usr/local/lib/libopencv_ml.so.2.4.13
Tracking: /usr/local/lib/libopencv_calib3d.so.2.4.13
Tracking: /usr/local/lib/libopencv_features2d.so.2.4.13
Tracking: /usr/local/lib/libopencv_highgui.so.2.4.13
Tracking: /usr/local/lib/libopencv_imgproc.so.2.4.13
Tracking: /usr/local/lib/libopencv_flann.so.2.4.13
Tracking: /usr/local/lib/libopencv_core.so.2.4.13
Tracking: /usr/local/cuda/lib64/libnppc.so
Tracking: /usr/local/cuda/lib64/libnppi.so
Tracking: /usr/local/cuda/lib64/libnpps.so
Tracking: /usr/local/cuda/lib64/libcublas.so
Tracking: /usr/local/cuda/lib64/libcufft.so
Tracking: CMakeFiles/Tracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yuhan/IARC/CornerDetection/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable Tracking"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Tracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Tracking.dir/build: Tracking

.PHONY : CMakeFiles/Tracking.dir/build

CMakeFiles/Tracking.dir/requires: CMakeFiles/Tracking.dir/VideoFlow.cpp.o.requires
CMakeFiles/Tracking.dir/requires: CMakeFiles/Tracking.dir/HoughCorner.cpp.o.requires

.PHONY : CMakeFiles/Tracking.dir/requires

CMakeFiles/Tracking.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Tracking.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Tracking.dir/clean

CMakeFiles/Tracking.dir/depend:
	cd /home/yuhan/IARC/CornerDetection/bin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuhan/IARC/CornerDetection /home/yuhan/IARC/CornerDetection /home/yuhan/IARC/CornerDetection/bin /home/yuhan/IARC/CornerDetection/bin /home/yuhan/IARC/CornerDetection/bin/CMakeFiles/Tracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Tracking.dir/depend

