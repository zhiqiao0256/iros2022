# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/pi/rosTest/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/rosTest/catkin_ws/build

# Include any dependencies generated for this target.
include optitrack/CMakeFiles/optitrack_sub.dir/depend.make

# Include the progress variables for this target.
include optitrack/CMakeFiles/optitrack_sub.dir/progress.make

# Include the compile flags for this target's objects.
include optitrack/CMakeFiles/optitrack_sub.dir/flags.make

optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o: optitrack/CMakeFiles/optitrack_sub.dir/flags.make
optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o: /home/pi/rosTest/catkin_ws/src/optitrack/src/mocap_subscriber.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/rosTest/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o"
	cd /home/pi/rosTest/catkin_ws/build/optitrack && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o -c /home/pi/rosTest/catkin_ws/src/optitrack/src/mocap_subscriber.cpp

optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.i"
	cd /home/pi/rosTest/catkin_ws/build/optitrack && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/rosTest/catkin_ws/src/optitrack/src/mocap_subscriber.cpp > CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.i

optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.s"
	cd /home/pi/rosTest/catkin_ws/build/optitrack && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/rosTest/catkin_ws/src/optitrack/src/mocap_subscriber.cpp -o CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.s

optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o.requires:

.PHONY : optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o.requires

optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o.provides: optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o.requires
	$(MAKE) -f optitrack/CMakeFiles/optitrack_sub.dir/build.make optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o.provides.build
.PHONY : optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o.provides

optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o.provides.build: optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o


# Object files for target optitrack_sub
optitrack_sub_OBJECTS = \
"CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o"

# External object files for target optitrack_sub
optitrack_sub_EXTERNAL_OBJECTS =

/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: optitrack/CMakeFiles/optitrack_sub.dir/build.make
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /opt/ros/kinetic/lib/libmessage_filters.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /opt/ros/kinetic/lib/libroscpp.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /opt/ros/kinetic/lib/librosconsole.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /opt/ros/kinetic/lib/librostime.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /opt/ros/kinetic/lib/libcpp_common.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: /usr/local/lib/libzmq.so
/home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub: optitrack/CMakeFiles/optitrack_sub.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/rosTest/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub"
	cd /home/pi/rosTest/catkin_ws/build/optitrack && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optitrack_sub.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
optitrack/CMakeFiles/optitrack_sub.dir/build: /home/pi/rosTest/catkin_ws/devel/lib/optitrack/optitrack_sub

.PHONY : optitrack/CMakeFiles/optitrack_sub.dir/build

optitrack/CMakeFiles/optitrack_sub.dir/requires: optitrack/CMakeFiles/optitrack_sub.dir/src/mocap_subscriber.cpp.o.requires

.PHONY : optitrack/CMakeFiles/optitrack_sub.dir/requires

optitrack/CMakeFiles/optitrack_sub.dir/clean:
	cd /home/pi/rosTest/catkin_ws/build/optitrack && $(CMAKE_COMMAND) -P CMakeFiles/optitrack_sub.dir/cmake_clean.cmake
.PHONY : optitrack/CMakeFiles/optitrack_sub.dir/clean

optitrack/CMakeFiles/optitrack_sub.dir/depend:
	cd /home/pi/rosTest/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/rosTest/catkin_ws/src /home/pi/rosTest/catkin_ws/src/optitrack /home/pi/rosTest/catkin_ws/build /home/pi/rosTest/catkin_ws/build/optitrack /home/pi/rosTest/catkin_ws/build/optitrack/CMakeFiles/optitrack_sub.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : optitrack/CMakeFiles/optitrack_sub.dir/depend

