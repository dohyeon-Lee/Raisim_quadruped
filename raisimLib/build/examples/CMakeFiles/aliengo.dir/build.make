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
include examples/CMakeFiles/aliengo.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/aliengo.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/aliengo.dir/flags.make

examples/CMakeFiles/aliengo.dir/src/server/aliengo.cpp.o: examples/CMakeFiles/aliengo.dir/flags.make
examples/CMakeFiles/aliengo.dir/src/server/aliengo.cpp.o: ../examples/src/server/aliengo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dohyeon/raisim_ws/raisimLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/aliengo.dir/src/server/aliengo.cpp.o"
	cd /home/dohyeon/raisim_ws/raisimLib/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aliengo.dir/src/server/aliengo.cpp.o -c /home/dohyeon/raisim_ws/raisimLib/examples/src/server/aliengo.cpp

examples/CMakeFiles/aliengo.dir/src/server/aliengo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aliengo.dir/src/server/aliengo.cpp.i"
	cd /home/dohyeon/raisim_ws/raisimLib/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dohyeon/raisim_ws/raisimLib/examples/src/server/aliengo.cpp > CMakeFiles/aliengo.dir/src/server/aliengo.cpp.i

examples/CMakeFiles/aliengo.dir/src/server/aliengo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aliengo.dir/src/server/aliengo.cpp.s"
	cd /home/dohyeon/raisim_ws/raisimLib/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dohyeon/raisim_ws/raisimLib/examples/src/server/aliengo.cpp -o CMakeFiles/aliengo.dir/src/server/aliengo.cpp.s

# Object files for target aliengo
aliengo_OBJECTS = \
"CMakeFiles/aliengo.dir/src/server/aliengo.cpp.o"

# External object files for target aliengo
aliengo_EXTERNAL_OBJECTS =

examples/aliengo: examples/CMakeFiles/aliengo.dir/src/server/aliengo.cpp.o
examples/aliengo: examples/CMakeFiles/aliengo.dir/build.make
examples/aliengo: ../raisim/linux/lib/libraisim.so.1.1.7
examples/aliengo: ../raisim/linux/lib/libraisimPng.so
examples/aliengo: ../raisim/linux/lib/libraisimZ.so
examples/aliengo: ../raisim/linux/lib/libraisimODE.so.1.1.7
examples/aliengo: ../raisim/linux/lib/libraisimMine.so
examples/aliengo: examples/CMakeFiles/aliengo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dohyeon/raisim_ws/raisimLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable aliengo"
	cd /home/dohyeon/raisim_ws/raisimLib/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aliengo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/aliengo.dir/build: examples/aliengo

.PHONY : examples/CMakeFiles/aliengo.dir/build

examples/CMakeFiles/aliengo.dir/clean:
	cd /home/dohyeon/raisim_ws/raisimLib/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/aliengo.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/aliengo.dir/clean

examples/CMakeFiles/aliengo.dir/depend:
	cd /home/dohyeon/raisim_ws/raisimLib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dohyeon/raisim_ws/raisimLib /home/dohyeon/raisim_ws/raisimLib/examples /home/dohyeon/raisim_ws/raisimLib/build /home/dohyeon/raisim_ws/raisimLib/build/examples /home/dohyeon/raisim_ws/raisimLib/build/examples/CMakeFiles/aliengo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/aliengo.dir/depend

