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
CMAKE_SOURCE_DIR = /home/dohyeon/raisim_ws/raisimLib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dohyeon/raisim_ws/raisimLib/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/kinova.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/kinova.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/kinova.dir/flags.make

examples/CMakeFiles/kinova.dir/src/maps/kinova.cpp.o: examples/CMakeFiles/kinova.dir/flags.make
examples/CMakeFiles/kinova.dir/src/maps/kinova.cpp.o: ../examples/src/maps/kinova.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dohyeon/raisim_ws/raisimLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/kinova.dir/src/maps/kinova.cpp.o"
	cd /home/dohyeon/raisim_ws/raisimLib/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinova.dir/src/maps/kinova.cpp.o -c /home/dohyeon/raisim_ws/raisimLib/examples/src/maps/kinova.cpp

examples/CMakeFiles/kinova.dir/src/maps/kinova.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinova.dir/src/maps/kinova.cpp.i"
	cd /home/dohyeon/raisim_ws/raisimLib/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dohyeon/raisim_ws/raisimLib/examples/src/maps/kinova.cpp > CMakeFiles/kinova.dir/src/maps/kinova.cpp.i

examples/CMakeFiles/kinova.dir/src/maps/kinova.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinova.dir/src/maps/kinova.cpp.s"
	cd /home/dohyeon/raisim_ws/raisimLib/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dohyeon/raisim_ws/raisimLib/examples/src/maps/kinova.cpp -o CMakeFiles/kinova.dir/src/maps/kinova.cpp.s

# Object files for target kinova
kinova_OBJECTS = \
"CMakeFiles/kinova.dir/src/maps/kinova.cpp.o"

# External object files for target kinova
kinova_EXTERNAL_OBJECTS =

examples/kinova: examples/CMakeFiles/kinova.dir/src/maps/kinova.cpp.o
examples/kinova: examples/CMakeFiles/kinova.dir/build.make
examples/kinova: ../raisim/linux/lib/libraisim.so.1.1.7
examples/kinova: ../raisim/linux/lib/libraisimPng.so
examples/kinova: ../raisim/linux/lib/libraisimZ.so
examples/kinova: ../raisim/linux/lib/libraisimODE.so.1.1.7
examples/kinova: ../raisim/linux/lib/libraisimMine.so
examples/kinova: examples/CMakeFiles/kinova.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dohyeon/raisim_ws/raisimLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable kinova"
	cd /home/dohyeon/raisim_ws/raisimLib/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinova.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/kinova.dir/build: examples/kinova

.PHONY : examples/CMakeFiles/kinova.dir/build

examples/CMakeFiles/kinova.dir/clean:
	cd /home/dohyeon/raisim_ws/raisimLib/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/kinova.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/kinova.dir/clean

examples/CMakeFiles/kinova.dir/depend:
	cd /home/dohyeon/raisim_ws/raisimLib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dohyeon/raisim_ws/raisimLib /home/dohyeon/raisim_ws/raisimLib/examples /home/dohyeon/raisim_ws/raisimLib/build /home/dohyeon/raisim_ws/raisimLib/build/examples /home/dohyeon/raisim_ws/raisimLib/build/examples/CMakeFiles/kinova.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/kinova.dir/depend

