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
CMAKE_SOURCE_DIR = /home/robot/robot_ws/RTCAN_Motor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/robot_ws/RTCAN_Motor/build

# Utility rule file for run_my_script.

# Include the progress variables for this target.
include CMakeFiles/run_my_script.dir/progress.make

CMakeFiles/run_my_script:
	chmod +x /home/robot/robot_ws/RTCAN_Motor/build/run_script.sh
	./run_script.sh

run_my_script: CMakeFiles/run_my_script
run_my_script: CMakeFiles/run_my_script.dir/build.make

.PHONY : run_my_script

# Rule to build all files generated by this target.
CMakeFiles/run_my_script.dir/build: run_my_script

.PHONY : CMakeFiles/run_my_script.dir/build

CMakeFiles/run_my_script.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_my_script.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_my_script.dir/clean

CMakeFiles/run_my_script.dir/depend:
	cd /home/robot/robot_ws/RTCAN_Motor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/robot_ws/RTCAN_Motor /home/robot/robot_ws/RTCAN_Motor /home/robot/robot_ws/RTCAN_Motor/build /home/robot/robot_ws/RTCAN_Motor/build /home/robot/robot_ws/RTCAN_Motor/build/CMakeFiles/run_my_script.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_my_script.dir/depend

