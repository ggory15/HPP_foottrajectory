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
include demo/CMakeFiles/test_NLP.dir/depend.make

# Include the progress variables for this target.
include demo/CMakeFiles/test_NLP.dir/progress.make

# Include the compile flags for this target's objects.
include demo/CMakeFiles/test_NLP.dir/flags.make

demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o: demo/CMakeFiles/test_NLP.dir/flags.make
demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o: ../demo/test_NLP.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/ggory15/ongoing work/HPP_foottrajectory/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o"
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" && /usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_NLP.dir/test_NLP.cc.o -c "/home/ggory15/ongoing work/HPP_foottrajectory/demo/test_NLP.cc"

demo/CMakeFiles/test_NLP.dir/test_NLP.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_NLP.dir/test_NLP.cc.i"
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/ggory15/ongoing work/HPP_foottrajectory/demo/test_NLP.cc" > CMakeFiles/test_NLP.dir/test_NLP.cc.i

demo/CMakeFiles/test_NLP.dir/test_NLP.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_NLP.dir/test_NLP.cc.s"
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/ggory15/ongoing work/HPP_foottrajectory/demo/test_NLP.cc" -o CMakeFiles/test_NLP.dir/test_NLP.cc.s

demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o.requires:

.PHONY : demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o.requires

demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o.provides: demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o.requires
	$(MAKE) -f demo/CMakeFiles/test_NLP.dir/build.make demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o.provides.build
.PHONY : demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o.provides

demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o.provides.build: demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o


# Object files for target test_NLP
test_NLP_OBJECTS = \
"CMakeFiles/test_NLP.dir/test_NLP.cc.o"

# External object files for target test_NLP
test_NLP_EXTERNAL_OBJECTS =

demo/test_NLP: demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o
demo/test_NLP: demo/CMakeFiles/test_NLP.dir/build.make
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_system.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_thread.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libpthread.so
demo/test_NLP: src/libhpp-foot.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_system.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_thread.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
demo/test_NLP: /usr/lib/x86_64-linux-gnu/libpthread.so
demo/test_NLP: demo/CMakeFiles/test_NLP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/ggory15/ongoing work/HPP_foottrajectory/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_NLP"
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_NLP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
demo/CMakeFiles/test_NLP.dir/build: demo/test_NLP

.PHONY : demo/CMakeFiles/test_NLP.dir/build

demo/CMakeFiles/test_NLP.dir/requires: demo/CMakeFiles/test_NLP.dir/test_NLP.cc.o.requires

.PHONY : demo/CMakeFiles/test_NLP.dir/requires

demo/CMakeFiles/test_NLP.dir/clean:
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" && $(CMAKE_COMMAND) -P CMakeFiles/test_NLP.dir/cmake_clean.cmake
.PHONY : demo/CMakeFiles/test_NLP.dir/clean

demo/CMakeFiles/test_NLP.dir/depend:
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/ggory15/ongoing work/HPP_foottrajectory" "/home/ggory15/ongoing work/HPP_foottrajectory/demo" "/home/ggory15/ongoing work/HPP_foottrajectory/build" "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo" "/home/ggory15/ongoing work/HPP_foottrajectory/build/demo/CMakeFiles/test_NLP.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : demo/CMakeFiles/test_NLP.dir/depend

