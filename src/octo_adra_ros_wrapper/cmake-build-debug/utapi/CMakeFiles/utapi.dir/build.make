# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /snap/clion/180/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/180/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug

# Include any dependencies generated for this target.
include utapi/CMakeFiles/utapi.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include utapi/CMakeFiles/utapi.dir/compiler_depend.make

# Include the progress variables for this target.
include utapi/CMakeFiles/utapi.dir/progress.make

# Include the compile flags for this target's objects.
include utapi/CMakeFiles/utapi.dir/flags.make

utapi/CMakeFiles/utapi.dir/common/network.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/common/network.cpp.o: ../utapi/common/network.cpp
utapi/CMakeFiles/utapi.dir/common/network.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object utapi/CMakeFiles/utapi.dir/common/network.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/common/network.cpp.o -MF CMakeFiles/utapi.dir/common/network.cpp.o.d -o CMakeFiles/utapi.dir/common/network.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/network.cpp

utapi/CMakeFiles/utapi.dir/common/network.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/common/network.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/network.cpp > CMakeFiles/utapi.dir/common/network.cpp.i

utapi/CMakeFiles/utapi.dir/common/network.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/common/network.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/network.cpp -o CMakeFiles/utapi.dir/common/network.cpp.s

utapi/CMakeFiles/utapi.dir/common/preempt_rt.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/common/preempt_rt.cpp.o: ../utapi/common/preempt_rt.cpp
utapi/CMakeFiles/utapi.dir/common/preempt_rt.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object utapi/CMakeFiles/utapi.dir/common/preempt_rt.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/common/preempt_rt.cpp.o -MF CMakeFiles/utapi.dir/common/preempt_rt.cpp.o.d -o CMakeFiles/utapi.dir/common/preempt_rt.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/preempt_rt.cpp

utapi/CMakeFiles/utapi.dir/common/preempt_rt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/common/preempt_rt.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/preempt_rt.cpp > CMakeFiles/utapi.dir/common/preempt_rt.cpp.i

utapi/CMakeFiles/utapi.dir/common/preempt_rt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/common/preempt_rt.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/preempt_rt.cpp -o CMakeFiles/utapi.dir/common/preempt_rt.cpp.s

utapi/CMakeFiles/utapi.dir/common/socket_serial.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/common/socket_serial.cpp.o: ../utapi/common/socket_serial.cpp
utapi/CMakeFiles/utapi.dir/common/socket_serial.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object utapi/CMakeFiles/utapi.dir/common/socket_serial.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/common/socket_serial.cpp.o -MF CMakeFiles/utapi.dir/common/socket_serial.cpp.o.d -o CMakeFiles/utapi.dir/common/socket_serial.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/socket_serial.cpp

utapi/CMakeFiles/utapi.dir/common/socket_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/common/socket_serial.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/socket_serial.cpp > CMakeFiles/utapi.dir/common/socket_serial.cpp.i

utapi/CMakeFiles/utapi.dir/common/socket_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/common/socket_serial.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/socket_serial.cpp -o CMakeFiles/utapi.dir/common/socket_serial.cpp.s

utapi/CMakeFiles/utapi.dir/common/socket_tcp.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/common/socket_tcp.cpp.o: ../utapi/common/socket_tcp.cpp
utapi/CMakeFiles/utapi.dir/common/socket_tcp.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object utapi/CMakeFiles/utapi.dir/common/socket_tcp.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/common/socket_tcp.cpp.o -MF CMakeFiles/utapi.dir/common/socket_tcp.cpp.o.d -o CMakeFiles/utapi.dir/common/socket_tcp.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/socket_tcp.cpp

utapi/CMakeFiles/utapi.dir/common/socket_tcp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/common/socket_tcp.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/socket_tcp.cpp > CMakeFiles/utapi.dir/common/socket_tcp.cpp.i

utapi/CMakeFiles/utapi.dir/common/socket_tcp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/common/socket_tcp.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/common/socket_tcp.cpp -o CMakeFiles/utapi.dir/common/socket_tcp.cpp.s

utapi/CMakeFiles/utapi.dir/base/servo_api_base.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/base/servo_api_base.cpp.o: ../utapi/base/servo_api_base.cpp
utapi/CMakeFiles/utapi.dir/base/servo_api_base.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object utapi/CMakeFiles/utapi.dir/base/servo_api_base.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/base/servo_api_base.cpp.o -MF CMakeFiles/utapi.dir/base/servo_api_base.cpp.o.d -o CMakeFiles/utapi.dir/base/servo_api_base.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/servo_api_base.cpp

utapi/CMakeFiles/utapi.dir/base/servo_api_base.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/base/servo_api_base.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/servo_api_base.cpp > CMakeFiles/utapi.dir/base/servo_api_base.cpp.i

utapi/CMakeFiles/utapi.dir/base/servo_api_base.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/base/servo_api_base.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/servo_api_base.cpp -o CMakeFiles/utapi.dir/base/servo_api_base.cpp.s

utapi/CMakeFiles/utapi.dir/adra/adra_api_base.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/adra/adra_api_base.cpp.o: ../utapi/adra/adra_api_base.cpp
utapi/CMakeFiles/utapi.dir/adra/adra_api_base.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object utapi/CMakeFiles/utapi.dir/adra/adra_api_base.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/adra/adra_api_base.cpp.o -MF CMakeFiles/utapi.dir/adra/adra_api_base.cpp.o.d -o CMakeFiles/utapi.dir/adra/adra_api_base.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/adra/adra_api_base.cpp

utapi/CMakeFiles/utapi.dir/adra/adra_api_base.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/adra/adra_api_base.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/adra/adra_api_base.cpp > CMakeFiles/utapi.dir/adra/adra_api_base.cpp.i

utapi/CMakeFiles/utapi.dir/adra/adra_api_base.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/adra/adra_api_base.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/adra/adra_api_base.cpp -o CMakeFiles/utapi.dir/adra/adra_api_base.cpp.s

utapi/CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.o: ../utapi/adra/adra_api_serial.cpp
utapi/CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object utapi/CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.o -MF CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.o.d -o CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/adra/adra_api_serial.cpp

utapi/CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/adra/adra_api_serial.cpp > CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.i

utapi/CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/adra/adra_api_serial.cpp -o CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.s

utapi/CMakeFiles/utapi.dir/base/arm_api_base.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/base/arm_api_base.cpp.o: ../utapi/base/arm_api_base.cpp
utapi/CMakeFiles/utapi.dir/base/arm_api_base.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object utapi/CMakeFiles/utapi.dir/base/arm_api_base.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/base/arm_api_base.cpp.o -MF CMakeFiles/utapi.dir/base/arm_api_base.cpp.o.d -o CMakeFiles/utapi.dir/base/arm_api_base.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/arm_api_base.cpp

utapi/CMakeFiles/utapi.dir/base/arm_api_base.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/base/arm_api_base.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/arm_api_base.cpp > CMakeFiles/utapi.dir/base/arm_api_base.cpp.i

utapi/CMakeFiles/utapi.dir/base/arm_api_base.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/base/arm_api_base.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/arm_api_base.cpp -o CMakeFiles/utapi.dir/base/arm_api_base.cpp.s

utapi/CMakeFiles/utapi.dir/base/arm_report_status.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/base/arm_report_status.cpp.o: ../utapi/base/arm_report_status.cpp
utapi/CMakeFiles/utapi.dir/base/arm_report_status.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object utapi/CMakeFiles/utapi.dir/base/arm_report_status.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/base/arm_report_status.cpp.o -MF CMakeFiles/utapi.dir/base/arm_report_status.cpp.o.d -o CMakeFiles/utapi.dir/base/arm_report_status.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/arm_report_status.cpp

utapi/CMakeFiles/utapi.dir/base/arm_report_status.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/base/arm_report_status.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/arm_report_status.cpp > CMakeFiles/utapi.dir/base/arm_report_status.cpp.i

utapi/CMakeFiles/utapi.dir/base/arm_report_status.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/base/arm_report_status.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/arm_report_status.cpp -o CMakeFiles/utapi.dir/base/arm_report_status.cpp.s

utapi/CMakeFiles/utapi.dir/base/arm_report_config.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/base/arm_report_config.cpp.o: ../utapi/base/arm_report_config.cpp
utapi/CMakeFiles/utapi.dir/base/arm_report_config.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object utapi/CMakeFiles/utapi.dir/base/arm_report_config.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/base/arm_report_config.cpp.o -MF CMakeFiles/utapi.dir/base/arm_report_config.cpp.o.d -o CMakeFiles/utapi.dir/base/arm_report_config.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/arm_report_config.cpp

utapi/CMakeFiles/utapi.dir/base/arm_report_config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/base/arm_report_config.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/arm_report_config.cpp > CMakeFiles/utapi.dir/base/arm_report_config.cpp.i

utapi/CMakeFiles/utapi.dir/base/arm_report_config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/base/arm_report_config.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/base/arm_report_config.cpp -o CMakeFiles/utapi.dir/base/arm_report_config.cpp.s

utapi/CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.o: ../utapi/utra/utra_api_tcp.cpp
utapi/CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object utapi/CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.o -MF CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.o.d -o CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_api_tcp.cpp

utapi/CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_api_tcp.cpp > CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.i

utapi/CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_api_tcp.cpp -o CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.s

utapi/CMakeFiles/utapi.dir/utra/utra_report_status.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/utra/utra_report_status.cpp.o: ../utapi/utra/utra_report_status.cpp
utapi/CMakeFiles/utapi.dir/utra/utra_report_status.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object utapi/CMakeFiles/utapi.dir/utra/utra_report_status.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/utra/utra_report_status.cpp.o -MF CMakeFiles/utapi.dir/utra/utra_report_status.cpp.o.d -o CMakeFiles/utapi.dir/utra/utra_report_status.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_report_status.cpp

utapi/CMakeFiles/utapi.dir/utra/utra_report_status.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/utra/utra_report_status.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_report_status.cpp > CMakeFiles/utapi.dir/utra/utra_report_status.cpp.i

utapi/CMakeFiles/utapi.dir/utra/utra_report_status.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/utra/utra_report_status.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_report_status.cpp -o CMakeFiles/utapi.dir/utra/utra_report_status.cpp.s

utapi/CMakeFiles/utapi.dir/utra/utra_report_config.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/utra/utra_report_config.cpp.o: ../utapi/utra/utra_report_config.cpp
utapi/CMakeFiles/utapi.dir/utra/utra_report_config.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object utapi/CMakeFiles/utapi.dir/utra/utra_report_config.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/utra/utra_report_config.cpp.o -MF CMakeFiles/utapi.dir/utra/utra_report_config.cpp.o.d -o CMakeFiles/utapi.dir/utra/utra_report_config.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_report_config.cpp

utapi/CMakeFiles/utapi.dir/utra/utra_report_config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/utra/utra_report_config.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_report_config.cpp > CMakeFiles/utapi.dir/utra/utra_report_config.cpp.i

utapi/CMakeFiles/utapi.dir/utra/utra_report_config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/utra/utra_report_config.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_report_config.cpp -o CMakeFiles/utapi.dir/utra/utra_report_config.cpp.s

utapi/CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.o: ../utapi/utra/utra_flxie_api.cpp
utapi/CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object utapi/CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.o -MF CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.o.d -o CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_flxie_api.cpp

utapi/CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_flxie_api.cpp > CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.i

utapi/CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_flxie_api.cpp -o CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.s

utapi/CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.o: utapi/CMakeFiles/utapi.dir/flags.make
utapi/CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.o: ../utapi/utra/utra_flxiv_api.cpp
utapi/CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.o: utapi/CMakeFiles/utapi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object utapi/CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.o"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utapi/CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.o -MF CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.o.d -o CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.o -c /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_flxiv_api.cpp

utapi/CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.i"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_flxiv_api.cpp > CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.i

utapi/CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.s"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi/utra/utra_flxiv_api.cpp -o CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.s

# Object files for target utapi
utapi_OBJECTS = \
"CMakeFiles/utapi.dir/common/network.cpp.o" \
"CMakeFiles/utapi.dir/common/preempt_rt.cpp.o" \
"CMakeFiles/utapi.dir/common/socket_serial.cpp.o" \
"CMakeFiles/utapi.dir/common/socket_tcp.cpp.o" \
"CMakeFiles/utapi.dir/base/servo_api_base.cpp.o" \
"CMakeFiles/utapi.dir/adra/adra_api_base.cpp.o" \
"CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.o" \
"CMakeFiles/utapi.dir/base/arm_api_base.cpp.o" \
"CMakeFiles/utapi.dir/base/arm_report_status.cpp.o" \
"CMakeFiles/utapi.dir/base/arm_report_config.cpp.o" \
"CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.o" \
"CMakeFiles/utapi.dir/utra/utra_report_status.cpp.o" \
"CMakeFiles/utapi.dir/utra/utra_report_config.cpp.o" \
"CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.o" \
"CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.o"

# External object files for target utapi
utapi_EXTERNAL_OBJECTS =

devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/common/network.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/common/preempt_rt.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/common/socket_serial.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/common/socket_tcp.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/base/servo_api_base.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/adra/adra_api_base.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/adra/adra_api_serial.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/base/arm_api_base.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/base/arm_report_status.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/base/arm_report_config.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/utra/utra_api_tcp.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/utra/utra_report_status.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/utra/utra_report_config.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/utra/utra_flxie_api.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/utra/utra_flxiv_api.cpp.o
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/build.make
devel/lib/libutapi.a: utapi/CMakeFiles/utapi.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Linking CXX static library ../devel/lib/libutapi.a"
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && $(CMAKE_COMMAND) -P CMakeFiles/utapi.dir/cmake_clean_target.cmake
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/utapi.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utapi/CMakeFiles/utapi.dir/build: devel/lib/libutapi.a
.PHONY : utapi/CMakeFiles/utapi.dir/build

utapi/CMakeFiles/utapi.dir/clean:
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi && $(CMAKE_COMMAND) -P CMakeFiles/utapi.dir/cmake_clean.cmake
.PHONY : utapi/CMakeFiles/utapi.dir/clean

utapi/CMakeFiles/utapi.dir/depend:
	cd /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/utapi /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi /home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/cmake-build-debug/utapi/CMakeFiles/utapi.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utapi/CMakeFiles/utapi.dir/depend

