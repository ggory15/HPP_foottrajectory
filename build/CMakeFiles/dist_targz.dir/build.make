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

# Utility rule file for dist_targz.

# Include the progress variables for this target.
include CMakeFiles/dist_targz.dir/progress.make

CMakeFiles/dist_targz:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir="/home/ggory15/ongoing work/HPP_foottrajectory/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Generating tar.gz tarball and its signature..."
	/bin/tar -czf hpp-foot-UNKNOWN-dirty.tar.gz hpp-foot-UNKNOWN-dirty/ && /usr/bin/gpg --detach-sign --armor -o /home/ggory15/ongoing\ work/HPP_foottrajectory/build/hpp-foot-UNKNOWN-dirty.tar.gz.sig /home/ggory15/ongoing\ work/HPP_foottrajectory/build/hpp-foot-UNKNOWN-dirty.tar.gz

dist_targz: CMakeFiles/dist_targz
dist_targz: CMakeFiles/dist_targz.dir/build.make

.PHONY : dist_targz

# Rule to build all files generated by this target.
CMakeFiles/dist_targz.dir/build: dist_targz

.PHONY : CMakeFiles/dist_targz.dir/build

CMakeFiles/dist_targz.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dist_targz.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dist_targz.dir/clean

CMakeFiles/dist_targz.dir/depend:
	cd "/home/ggory15/ongoing work/HPP_foottrajectory/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/ggory15/ongoing work/HPP_foottrajectory" "/home/ggory15/ongoing work/HPP_foottrajectory" "/home/ggory15/ongoing work/HPP_foottrajectory/build" "/home/ggory15/ongoing work/HPP_foottrajectory/build" "/home/ggory15/ongoing work/HPP_foottrajectory/build/CMakeFiles/dist_targz.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/dist_targz.dir/depend

