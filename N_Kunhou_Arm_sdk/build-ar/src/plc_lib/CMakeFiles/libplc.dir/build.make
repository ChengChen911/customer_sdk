# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar

# Include any dependencies generated for this target.
include src/plc_lib/CMakeFiles/libplc.dir/depend.make

# Include the progress variables for this target.
include src/plc_lib/CMakeFiles/libplc.dir/progress.make

# Include the compile flags for this target's objects.
include src/plc_lib/CMakeFiles/libplc.dir/flags.make

src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o: src/plc_lib/CMakeFiles/libplc.dir/flags.make
src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o: /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source/src/plc_lib/plc_siemens.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o"
	cd /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib && arm-xilinx-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libplc.dir/plc_siemens.cpp.o -c /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source/src/plc_lib/plc_siemens.cpp

src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libplc.dir/plc_siemens.cpp.i"
	cd /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib && arm-xilinx-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source/src/plc_lib/plc_siemens.cpp > CMakeFiles/libplc.dir/plc_siemens.cpp.i

src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libplc.dir/plc_siemens.cpp.s"
	cd /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib && arm-xilinx-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source/src/plc_lib/plc_siemens.cpp -o CMakeFiles/libplc.dir/plc_siemens.cpp.s

src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o.requires:
.PHONY : src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o.requires

src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o.provides: src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o.requires
	$(MAKE) -f src/plc_lib/CMakeFiles/libplc.dir/build.make src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o.provides.build
.PHONY : src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o.provides

src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o.provides.build: src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o

src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o: src/plc_lib/CMakeFiles/libplc.dir/flags.make
src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o: /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source/src/plc_lib/plc_base.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o"
	cd /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib && arm-xilinx-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libplc.dir/plc_base.cpp.o -c /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source/src/plc_lib/plc_base.cpp

src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libplc.dir/plc_base.cpp.i"
	cd /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib && arm-xilinx-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source/src/plc_lib/plc_base.cpp > CMakeFiles/libplc.dir/plc_base.cpp.i

src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libplc.dir/plc_base.cpp.s"
	cd /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib && arm-xilinx-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source/src/plc_lib/plc_base.cpp -o CMakeFiles/libplc.dir/plc_base.cpp.s

src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o.requires:
.PHONY : src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o.requires

src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o.provides: src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o.requires
	$(MAKE) -f src/plc_lib/CMakeFiles/libplc.dir/build.make src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o.provides.build
.PHONY : src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o.provides

src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o.provides.build: src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o

# Object files for target libplc
libplc_OBJECTS = \
"CMakeFiles/libplc.dir/plc_siemens.cpp.o" \
"CMakeFiles/libplc.dir/plc_base.cpp.o"

# External object files for target libplc
libplc_EXTERNAL_OBJECTS =

/home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/lib/arm/liblibplc.a: src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o
/home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/lib/arm/liblibplc.a: src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o
/home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/lib/arm/liblibplc.a: src/plc_lib/CMakeFiles/libplc.dir/build.make
/home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/lib/arm/liblibplc.a: src/plc_lib/CMakeFiles/libplc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/lib/arm/liblibplc.a"
	cd /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib && $(CMAKE_COMMAND) -P CMakeFiles/libplc.dir/cmake_clean_target.cmake
	cd /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/libplc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/plc_lib/CMakeFiles/libplc.dir/build: /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/lib/arm/liblibplc.a
.PHONY : src/plc_lib/CMakeFiles/libplc.dir/build

src/plc_lib/CMakeFiles/libplc.dir/requires: src/plc_lib/CMakeFiles/libplc.dir/plc_siemens.cpp.o.requires
src/plc_lib/CMakeFiles/libplc.dir/requires: src/plc_lib/CMakeFiles/libplc.dir/plc_base.cpp.o.requires
.PHONY : src/plc_lib/CMakeFiles/libplc.dir/requires

src/plc_lib/CMakeFiles/libplc.dir/clean:
	cd /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib && $(CMAKE_COMMAND) -P CMakeFiles/libplc.dir/cmake_clean.cmake
.PHONY : src/plc_lib/CMakeFiles/libplc.dir/clean

src/plc_lib/CMakeFiles/libplc.dir/depend:
	cd /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/source/src/plc_lib /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib /home/ubuntu1404/customer_sdk/N_Kunhou_Arm_sdk/build-ar/src/plc_lib/CMakeFiles/libplc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/plc_lib/CMakeFiles/libplc.dir/depend
