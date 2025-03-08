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
CMAKE_SOURCE_DIR = /home/jg/bno055_hardware/src/imu_tools/imu_filter_madgwick

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jg/bno055_hardware/build/imu_filter_madgwick

# Include any dependencies generated for this target.
include test/CMakeFiles/stateless_orientation_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/stateless_orientation_test.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/stateless_orientation_test.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/stateless_orientation_test.dir/flags.make

test/CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.o: test/CMakeFiles/stateless_orientation_test.dir/flags.make
test/CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.o: /home/jg/bno055_hardware/src/imu_tools/imu_filter_madgwick/test/stateless_orientation_test.cpp
test/CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.o: test/CMakeFiles/stateless_orientation_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jg/bno055_hardware/build/imu_filter_madgwick/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.o"
	cd /home/jg/bno055_hardware/build/imu_filter_madgwick/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.o -MF CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.o.d -o CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.o -c /home/jg/bno055_hardware/src/imu_tools/imu_filter_madgwick/test/stateless_orientation_test.cpp

test/CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.i"
	cd /home/jg/bno055_hardware/build/imu_filter_madgwick/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jg/bno055_hardware/src/imu_tools/imu_filter_madgwick/test/stateless_orientation_test.cpp > CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.i

test/CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.s"
	cd /home/jg/bno055_hardware/build/imu_filter_madgwick/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jg/bno055_hardware/src/imu_tools/imu_filter_madgwick/test/stateless_orientation_test.cpp -o CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.s

# Object files for target stateless_orientation_test
stateless_orientation_test_OBJECTS = \
"CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.o"

# External object files for target stateless_orientation_test
stateless_orientation_test_EXTERNAL_OBJECTS =

test/stateless_orientation_test: test/CMakeFiles/stateless_orientation_test.dir/stateless_orientation_test.cpp.o
test/stateless_orientation_test: test/CMakeFiles/stateless_orientation_test.dir/build.make
test/stateless_orientation_test: gtest/libgtest_main.a
test/stateless_orientation_test: gtest/libgtest.a
test/stateless_orientation_test: libimu_filter_madgwick.so
test/stateless_orientation_test: /opt/ros/humble/lib/libcomponent_manager.so
test/stateless_orientation_test: /opt/ros/humble/lib/libclass_loader.so
test/stateless_orientation_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_generator_py.so
test/stateless_orientation_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_py.so
test/stateless_orientation_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /usr/lib/x86_64-linux-gnu/liborocos-kdl.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstatic_transform_broadcaster_node.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtf2_ros.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtf2.so
test/stateless_orientation_test: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
test/stateless_orientation_test: /opt/ros/humble/lib/libmessage_filters.so
test/stateless_orientation_test: /opt/ros/humble/lib/librclcpp_action.so
test/stateless_orientation_test: /opt/ros/humble/lib/librclcpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/liblibstatistics_collector.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_action.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_yaml_param_parser.so
test/stateless_orientation_test: /opt/ros/humble/lib/libyaml.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtracetools.so
test/stateless_orientation_test: /opt/ros/humble/lib/librmw_implementation.so
test/stateless_orientation_test: /opt/ros/humble/lib/libament_index_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_logging_spdlog.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcl_logging_interface.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libfastcdr.so.1.0.24
test/stateless_orientation_test: /opt/ros/humble/lib/librmw.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_generator_py.so
test/stateless_orientation_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libaction_msgs__rosidl_generator_py.so
test/stateless_orientation_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
test/stateless_orientation_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libaction_msgs__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_generator_py.so
test/stateless_orientation_test: /usr/lib/x86_64-linux-gnu/libpython3.10.so
test/stateless_orientation_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosidl_typesupport_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcpputils.so
test/stateless_orientation_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_generator_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librosidl_runtime_c.so
test/stateless_orientation_test: /opt/ros/humble/lib/librcutils.so
test/stateless_orientation_test: test/CMakeFiles/stateless_orientation_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jg/bno055_hardware/build/imu_filter_madgwick/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable stateless_orientation_test"
	cd /home/jg/bno055_hardware/build/imu_filter_madgwick/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stateless_orientation_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/stateless_orientation_test.dir/build: test/stateless_orientation_test
.PHONY : test/CMakeFiles/stateless_orientation_test.dir/build

test/CMakeFiles/stateless_orientation_test.dir/clean:
	cd /home/jg/bno055_hardware/build/imu_filter_madgwick/test && $(CMAKE_COMMAND) -P CMakeFiles/stateless_orientation_test.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/stateless_orientation_test.dir/clean

test/CMakeFiles/stateless_orientation_test.dir/depend:
	cd /home/jg/bno055_hardware/build/imu_filter_madgwick && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jg/bno055_hardware/src/imu_tools/imu_filter_madgwick /home/jg/bno055_hardware/src/imu_tools/imu_filter_madgwick/test /home/jg/bno055_hardware/build/imu_filter_madgwick /home/jg/bno055_hardware/build/imu_filter_madgwick/test /home/jg/bno055_hardware/build/imu_filter_madgwick/test/CMakeFiles/stateless_orientation_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/stateless_orientation_test.dir/depend

