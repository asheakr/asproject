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
CMAKE_SOURCE_DIR = /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build

# Include any dependencies generated for this target.
include C/CMakeFiles/socket_int.dir/depend.make

# Include the progress variables for this target.
include C/CMakeFiles/socket_int.dir/progress.make

# Include the compile flags for this target's objects.
include C/CMakeFiles/socket_int.dir/flags.make

# Object files for target socket_int
socket_int_OBJECTS =

# External object files for target socket_int
socket_int_EXTERNAL_OBJECTS = \
"/home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build/C/CMakeFiles/socket_o.dir/inet/libinetsocket.c.o" \
"/home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build/C/CMakeFiles/socket_o.dir/unix/libunixsocket.c.o"

C/libsocket_int.a: C/CMakeFiles/socket_o.dir/inet/libinetsocket.c.o
C/libsocket_int.a: C/CMakeFiles/socket_o.dir/unix/libunixsocket.c.o
C/libsocket_int.a: C/CMakeFiles/socket_int.dir/build.make
C/libsocket_int.a: C/CMakeFiles/socket_int.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Linking C static library libsocket_int.a"
	cd /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build/C && $(CMAKE_COMMAND) -P CMakeFiles/socket_int.dir/cmake_clean_target.cmake
	cd /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build/C && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/socket_int.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
C/CMakeFiles/socket_int.dir/build: C/libsocket_int.a

.PHONY : C/CMakeFiles/socket_int.dir/build

C/CMakeFiles/socket_int.dir/clean:
	cd /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build/C && $(CMAKE_COMMAND) -P CMakeFiles/socket_int.dir/cmake_clean.cmake
.PHONY : C/CMakeFiles/socket_int.dir/clean

C/CMakeFiles/socket_int.dir/depend:
	cd /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket/C /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build/C /home/ashe/AS/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build/C/CMakeFiles/socket_int.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : C/CMakeFiles/socket_int.dir/depend

