# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/adam/Chalkbot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/adam/Chalkbot/build

# Include any dependencies generated for this target.
include CMakeFiles/libraryTest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/libraryTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/libraryTest.dir/flags.make

CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.o: CMakeFiles/libraryTest.dir/flags.make
CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.o: ../tests/libraryTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/adam/Chalkbot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.o -c /home/adam/Chalkbot/tests/libraryTest.cpp

CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/adam/Chalkbot/tests/libraryTest.cpp > CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.i

CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/adam/Chalkbot/tests/libraryTest.cpp -o CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.s

# Object files for target libraryTest
libraryTest_OBJECTS = \
"CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.o"

# External object files for target libraryTest
libraryTest_EXTERNAL_OBJECTS =

libraryTest: CMakeFiles/libraryTest.dir/tests/libraryTest.cpp.o
libraryTest: CMakeFiles/libraryTest.dir/build.make
libraryTest: libmarkerTracking.so
libraryTest: /usr/local/lib/libopencv_gapi.so.4.6.0
libraryTest: /usr/local/lib/libopencv_highgui.so.4.6.0
libraryTest: /usr/local/lib/libopencv_ml.so.4.6.0
libraryTest: /usr/local/lib/libopencv_objdetect.so.4.6.0
libraryTest: /usr/local/lib/libopencv_photo.so.4.6.0
libraryTest: /usr/local/lib/libopencv_stitching.so.4.6.0
libraryTest: /usr/local/lib/libopencv_video.so.4.6.0
libraryTest: /usr/local/lib/libopencv_calib3d.so.4.6.0
libraryTest: /usr/local/lib/libopencv_dnn.so.4.6.0
libraryTest: /usr/local/lib/libopencv_features2d.so.4.6.0
libraryTest: /usr/local/lib/libopencv_flann.so.4.6.0
libraryTest: /usr/local/lib/libopencv_videoio.so.4.6.0
libraryTest: /usr/local/lib/libopencv_imgcodecs.so.4.6.0
libraryTest: /usr/local/lib/libopencv_imgproc.so.4.6.0
libraryTest: /usr/local/lib/libopencv_core.so.4.6.0
libraryTest: CMakeFiles/libraryTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/adam/Chalkbot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable libraryTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/libraryTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/libraryTest.dir/build: libraryTest

.PHONY : CMakeFiles/libraryTest.dir/build

CMakeFiles/libraryTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/libraryTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/libraryTest.dir/clean

CMakeFiles/libraryTest.dir/depend:
	cd /home/adam/Chalkbot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adam/Chalkbot /home/adam/Chalkbot /home/adam/Chalkbot/build /home/adam/Chalkbot/build /home/adam/Chalkbot/build/CMakeFiles/libraryTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/libraryTest.dir/depend

