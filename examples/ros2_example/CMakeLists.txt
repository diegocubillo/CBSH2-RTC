# Example CMakeLists.txt for a ROS2 node using cbs_lib
cmake_minimum_required(VERSION 3.8)
project(my_ros2_planner_node)

# Add /usr/local to CMAKE_PREFIX_PATH for finding CBS library
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Find the CBS library
# Use manual search for better compatibility with ROS2
find_library(CBS_LIB cbs_lib PATHS /usr/local/lib)
set(CBS_INCLUDE_DIR /usr/local/include/cbs_lib)

if(NOT CBS_LIB)
  message(FATAL_ERROR "CBS library not found. Please install it with 'sudo make install' from the CBS project build directory.")
endif()

message(STATUS "Found CBS library: ${CBS_LIB}")
message(STATUS "CBS include directory: ${CBS_INCLUDE_DIR}")

# Create the executable
add_executable(planner_node src/planner_node.cpp)

# Include directories
target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set C++ standard
target_compile_features(planner_node PUBLIC cxx_std_14)

# Link libraries
ament_target_dependencies(planner_node
  rclcpp
  geometry_msgs
  nav_msgs
)

# Link CBS library
target_include_directories(planner_node PRIVATE ${CBS_INCLUDE_DIR})
target_link_libraries(planner_node ${CBS_LIB})

# Install executable
install(TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files, config files, etc. if needed
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
