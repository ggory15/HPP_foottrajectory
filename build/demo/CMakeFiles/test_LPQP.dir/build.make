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


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = "/home/ggory15/ongoing work/HPP_foottrajectory"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/ggory15/ongoing work/HPP_foottrajectory/build"

# Include any dependencies generated for this target.
include demo/CMakeFiles/test_LPQP.dir/depend.make

# Include the progress variables for this target.
include demo/CMakeFiles/test_LPQP.dir/progress.make

# Include the compile flags for this target's objects.
include demo/CMakeFiles/test_LPQP.dir/flags.make

demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o: demo/CMakeFiles/test_LPQP.dir/flags.make
demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o: ../demo/test_LPQP.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/ggory15/ongoing work/HPP_foottrajectory/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o"
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" && /usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_LPQP.dir/test_LPQP.cc.o -c "/home/ggory15/ongoing work/HPP_foottrajectory/demo/test_LPQP.cc"

demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_LPQP.dir/test_LPQP.cc.i"
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/ggory15/ongoing work/HPP_foottrajectory/demo/test_LPQP.cc" > CMakeFiles/test_LPQP.dir/test_LPQP.cc.i

demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_LPQP.dir/test_LPQP.cc.s"
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/ggory15/ongoing work/HPP_foottrajectory/demo/test_LPQP.cc" -o CMakeFiles/test_LPQP.dir/test_LPQP.cc.s

demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o.requires:

.PHONY : demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o.requires

demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o.provides: demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o.requires
	$(MAKE) -f demo/CMakeFiles/test_LPQP.dir/build.make demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o.provides.build
.PHONY : demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o.provides

demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o.provides.build: demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o


# Object files for target test_LPQP
test_LPQP_OBJECTS = \
"CMakeFiles/test_LPQP.dir/test_LPQP.cc.o"

# External object files for target test_LPQP
test_LPQP_EXTERNAL_OBJECTS =

demo/test_LPQP: demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o
demo/test_LPQP: demo/CMakeFiles/test_LPQP.dir/build.make
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_system.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_thread.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libpthread.so
demo/test_LPQP: src/libhpp-foot.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_system.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_thread.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
demo/test_LPQP: /usr/lib/x86_64-linux-gnu/libpthread.so
demo/test_LPQP: demo/CMakeFiles/test_LPQP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/ggory15/ongoing work/HPP_foottrajectory/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_LPQP"
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_LPQP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
demo/CMakeFiles/test_LPQP.dir/build: demo/test_LPQP

.PHONY : demo/CMakeFiles/test_LPQP.dir/build

demo/CMakeFiles/test_LPQP.dir/requires: demo/CMakeFiles/test_LPQP.dir/test_LPQP.cc.o.requires

.PHONY : demo/CMakeFiles/test_LPQP.dir/requires

demo/CMakeFiles/test_LPQP.dir/clean:
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" && $(CMAKE_COMMAND) -P CMakeFiles/test_LPQP.dir/cmake_clean.cmake
.PHONY : demo/CMakeFiles/test_LPQP.dir/clean

demo/CMakeFiles/test_LPQP.dir/depend:
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/ggory15/ongoing work/HPP_foottrajectory" "/home/ggory15/ongoing work/HPP_foottrajectory/demo" "/home/ggory15/ongoing work/HPP_foottrajectory/build" "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo/CMakeFiles/test_LPQP.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : demo/CMakeFiles/test_LPQP.dir/depend
