# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lunabotics/2025-tibalt-bot/arduino/tibaltArduino/teensy/rygel/vendor/fmt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lunabotics/2025-tibalt-bot/build/FMT

# Include any dependencies generated for this target.
include test/CMakeFiles/gmock.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/gmock.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/gmock.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/gmock.dir/flags.make

test/CMakeFiles/gmock.dir/gmock-gtest-all.cc.o: test/CMakeFiles/gmock.dir/flags.make
test/CMakeFiles/gmock.dir/gmock-gtest-all.cc.o: /home/lunabotics/2025-tibalt-bot/arduino/tibaltArduino/teensy/rygel/vendor/fmt/test/gmock-gtest-all.cc
test/CMakeFiles/gmock.dir/gmock-gtest-all.cc.o: test/CMakeFiles/gmock.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lunabotics/2025-tibalt-bot/build/FMT/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/gmock.dir/gmock-gtest-all.cc.o"
	cd /home/lunabotics/2025-tibalt-bot/build/FMT/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/gmock.dir/gmock-gtest-all.cc.o -MF CMakeFiles/gmock.dir/gmock-gtest-all.cc.o.d -o CMakeFiles/gmock.dir/gmock-gtest-all.cc.o -c /home/lunabotics/2025-tibalt-bot/arduino/tibaltArduino/teensy/rygel/vendor/fmt/test/gmock-gtest-all.cc

test/CMakeFiles/gmock.dir/gmock-gtest-all.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gmock.dir/gmock-gtest-all.cc.i"
	cd /home/lunabotics/2025-tibalt-bot/build/FMT/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lunabotics/2025-tibalt-bot/arduino/tibaltArduino/teensy/rygel/vendor/fmt/test/gmock-gtest-all.cc > CMakeFiles/gmock.dir/gmock-gtest-all.cc.i

test/CMakeFiles/gmock.dir/gmock-gtest-all.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gmock.dir/gmock-gtest-all.cc.s"
	cd /home/lunabotics/2025-tibalt-bot/build/FMT/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lunabotics/2025-tibalt-bot/arduino/tibaltArduino/teensy/rygel/vendor/fmt/test/gmock-gtest-all.cc -o CMakeFiles/gmock.dir/gmock-gtest-all.cc.s

# Object files for target gmock
gmock_OBJECTS = \
"CMakeFiles/gmock.dir/gmock-gtest-all.cc.o"

# External object files for target gmock
gmock_EXTERNAL_OBJECTS =

test/libgmock.a: test/CMakeFiles/gmock.dir/gmock-gtest-all.cc.o
test/libgmock.a: test/CMakeFiles/gmock.dir/build.make
test/libgmock.a: test/CMakeFiles/gmock.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lunabotics/2025-tibalt-bot/build/FMT/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libgmock.a"
	cd /home/lunabotics/2025-tibalt-bot/build/FMT/test && $(CMAKE_COMMAND) -P CMakeFiles/gmock.dir/cmake_clean_target.cmake
	cd /home/lunabotics/2025-tibalt-bot/build/FMT/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gmock.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/gmock.dir/build: test/libgmock.a
.PHONY : test/CMakeFiles/gmock.dir/build

test/CMakeFiles/gmock.dir/clean:
	cd /home/lunabotics/2025-tibalt-bot/build/FMT/test && $(CMAKE_COMMAND) -P CMakeFiles/gmock.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/gmock.dir/clean

test/CMakeFiles/gmock.dir/depend:
	cd /home/lunabotics/2025-tibalt-bot/build/FMT && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lunabotics/2025-tibalt-bot/arduino/tibaltArduino/teensy/rygel/vendor/fmt /home/lunabotics/2025-tibalt-bot/arduino/tibaltArduino/teensy/rygel/vendor/fmt/test /home/lunabotics/2025-tibalt-bot/build/FMT /home/lunabotics/2025-tibalt-bot/build/FMT/test /home/lunabotics/2025-tibalt-bot/build/FMT/test/CMakeFiles/gmock.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/gmock.dir/depend

