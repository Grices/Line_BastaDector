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
CMAKE_SOURCE_DIR = /home/penghua/BasestaDect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/penghua/BasestaDect/build

# Include any dependencies generated for this target.
include CMakeFiles/method.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/method.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/method.dir/flags.make

CMakeFiles/method.dir/src/method.cpp.o: CMakeFiles/method.dir/flags.make
CMakeFiles/method.dir/src/method.cpp.o: ../src/method.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/penghua/BasestaDect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/method.dir/src/method.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/method.dir/src/method.cpp.o -c /home/penghua/BasestaDect/src/method.cpp

CMakeFiles/method.dir/src/method.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/method.dir/src/method.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/penghua/BasestaDect/src/method.cpp > CMakeFiles/method.dir/src/method.cpp.i

CMakeFiles/method.dir/src/method.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/method.dir/src/method.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/penghua/BasestaDect/src/method.cpp -o CMakeFiles/method.dir/src/method.cpp.s

# Object files for target method
method_OBJECTS = \
"CMakeFiles/method.dir/src/method.cpp.o"

# External object files for target method
method_EXTERNAL_OBJECTS =

libmethod.a: CMakeFiles/method.dir/src/method.cpp.o
libmethod.a: CMakeFiles/method.dir/build.make
libmethod.a: CMakeFiles/method.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/penghua/BasestaDect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmethod.a"
	$(CMAKE_COMMAND) -P CMakeFiles/method.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/method.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/method.dir/build: libmethod.a

.PHONY : CMakeFiles/method.dir/build

CMakeFiles/method.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/method.dir/cmake_clean.cmake
.PHONY : CMakeFiles/method.dir/clean

CMakeFiles/method.dir/depend:
	cd /home/penghua/BasestaDect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/penghua/BasestaDect /home/penghua/BasestaDect /home/penghua/BasestaDect/build /home/penghua/BasestaDect/build /home/penghua/BasestaDect/build/CMakeFiles/method.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/method.dir/depend

