# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /home/zzh/software/clion-2018.3.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zzh/software/clion-2018.3.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zzh/catkin_ws_test/src/ivcommon

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zzh/catkin_ws_test/src/ivcommon/cmake-build-release

# Utility rule file for ivcommon_detect_changes.

# Include the progress variables for this target.
include CMakeFiles/ivcommon_detect_changes.dir/progress.make

CMakeFiles/ivcommon_detect_changes:
	bash -c "find /home/zzh/catkin_ws_test/src/ivcommon/ -not -iwholename '*.git*' | sort | sed 's/^/#/' | diff -N -q /home/zzh/catkin_ws_test/src/ivcommon/cmake-build-release/AllFiles.cmake - || find /home/zzh/catkin_ws_test/src/ivcommon/ -not -iwholename '*.git*' | sort | sed 's/^/#/' > /home/zzh/catkin_ws_test/src/ivcommon/cmake-build-release/AllFiles.cmake"

ivcommon_detect_changes: CMakeFiles/ivcommon_detect_changes
ivcommon_detect_changes: CMakeFiles/ivcommon_detect_changes.dir/build.make

.PHONY : ivcommon_detect_changes

# Rule to build all files generated by this target.
CMakeFiles/ivcommon_detect_changes.dir/build: ivcommon_detect_changes

.PHONY : CMakeFiles/ivcommon_detect_changes.dir/build

CMakeFiles/ivcommon_detect_changes.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ivcommon_detect_changes.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ivcommon_detect_changes.dir/clean

CMakeFiles/ivcommon_detect_changes.dir/depend:
	cd /home/zzh/catkin_ws_test/src/ivcommon/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zzh/catkin_ws_test/src/ivcommon /home/zzh/catkin_ws_test/src/ivcommon /home/zzh/catkin_ws_test/src/ivcommon/cmake-build-release /home/zzh/catkin_ws_test/src/ivcommon/cmake-build-release /home/zzh/catkin_ws_test/src/ivcommon/cmake-build-release/CMakeFiles/ivcommon_detect_changes.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ivcommon_detect_changes.dir/depend
