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
include CMakeFiles/WRITEFILE.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/WRITEFILE.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/WRITEFILE.dir/flags.make

CMakeFiles/WRITEFILE.dir/src/writefile.cpp.o: CMakeFiles/WRITEFILE.dir/flags.make
CMakeFiles/WRITEFILE.dir/src/writefile.cpp.o: ../src/writefile.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/penghua/BasestaDect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/WRITEFILE.dir/src/writefile.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/WRITEFILE.dir/src/writefile.cpp.o -c /home/penghua/BasestaDect/src/writefile.cpp

CMakeFiles/WRITEFILE.dir/src/writefile.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/WRITEFILE.dir/src/writefile.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/penghua/BasestaDect/src/writefile.cpp > CMakeFiles/WRITEFILE.dir/src/writefile.cpp.i

CMakeFiles/WRITEFILE.dir/src/writefile.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/WRITEFILE.dir/src/writefile.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/penghua/BasestaDect/src/writefile.cpp -o CMakeFiles/WRITEFILE.dir/src/writefile.cpp.s

# Object files for target WRITEFILE
WRITEFILE_OBJECTS = \
"CMakeFiles/WRITEFILE.dir/src/writefile.cpp.o"

# External object files for target WRITEFILE
WRITEFILE_EXTERNAL_OBJECTS =

WRITEFILE: CMakeFiles/WRITEFILE.dir/src/writefile.cpp.o
WRITEFILE: CMakeFiles/WRITEFILE.dir/build.make
WRITEFILE: CMakeFiles/WRITEFILE.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/penghua/BasestaDect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable WRITEFILE"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/WRITEFILE.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/WRITEFILE.dir/build: WRITEFILE

.PHONY : CMakeFiles/WRITEFILE.dir/build

CMakeFiles/WRITEFILE.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/WRITEFILE.dir/cmake_clean.cmake
.PHONY : CMakeFiles/WRITEFILE.dir/clean

CMakeFiles/WRITEFILE.dir/depend:
	cd /home/penghua/BasestaDect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/penghua/BasestaDect /home/penghua/BasestaDect /home/penghua/BasestaDect/build /home/penghua/BasestaDect/build /home/penghua/BasestaDect/build/CMakeFiles/WRITEFILE.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/WRITEFILE.dir/depend

