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
CMAKE_SOURCE_DIR = /home/dohyeon/raisim_ws/raisimLib/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dohyeon/raisim_ws/raisimLib/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/sensors.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sensors.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sensors.dir/flags.make

CMakeFiles/sensors.dir/src/server/sensors.cpp.o: CMakeFiles/sensors.dir/flags.make
CMakeFiles/sensors.dir/src/server/sensors.cpp.o: ../src/server/sensors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dohyeon/raisim_ws/raisimLib/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sensors.dir/src/server/sensors.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sensors.dir/src/server/sensors.cpp.o -c /home/dohyeon/raisim_ws/raisimLib/examples/src/server/sensors.cpp

CMakeFiles/sensors.dir/src/server/sensors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensors.dir/src/server/sensors.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dohyeon/raisim_ws/raisimLib/examples/src/server/sensors.cpp > CMakeFiles/sensors.dir/src/server/sensors.cpp.i

CMakeFiles/sensors.dir/src/server/sensors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensors.dir/src/server/sensors.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dohyeon/raisim_ws/raisimLib/examples/src/server/sensors.cpp -o CMakeFiles/sensors.dir/src/server/sensors.cpp.s

# Object files for target sensors
sensors_OBJECTS = \
"CMakeFiles/sensors.dir/src/server/sensors.cpp.o"

# External object files for target sensors
sensors_EXTERNAL_OBJECTS =

sensors: CMakeFiles/sensors.dir/src/server/sensors.cpp.o
sensors: CMakeFiles/sensors.dir/build.make
sensors: /home/dohyeon/raisim_ws/raisimLib/raisim/linux/lib/libraisim.so.1.1.7
sensors: /home/dohyeon/raisim_ws/raisimLib/raisim/linux/lib/libraisimPng.so
sensors: /home/dohyeon/raisim_ws/raisimLib/raisim/linux/lib/libraisimZ.so
sensors: /home/dohyeon/raisim_ws/raisimLib/raisim/linux/lib/libraisimODE.so.1.1.7
sensors: /home/dohyeon/raisim_ws/raisimLib/raisim/linux/lib/libraisimMine.so
sensors: CMakeFiles/sensors.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dohyeon/raisim_ws/raisimLib/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sensors"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sensors.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sensors.dir/build: sensors

.PHONY : CMakeFiles/sensors.dir/build

CMakeFiles/sensors.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sensors.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sensors.dir/clean

CMakeFiles/sensors.dir/depend:
	cd /home/dohyeon/raisim_ws/raisimLib/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dohyeon/raisim_ws/raisimLib/examples /home/dohyeon/raisim_ws/raisimLib/examples /home/dohyeon/raisim_ws/raisimLib/examples/build /home/dohyeon/raisim_ws/raisimLib/examples/build /home/dohyeon/raisim_ws/raisimLib/examples/build/CMakeFiles/sensors.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sensors.dir/depend

