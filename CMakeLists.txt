cmake_minimum_required(VERSION 3.8)
project(lidardrone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap_msgs REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare a C++ executable
add_executable(occupancy_grid_mapper_node src/occupancy_grid_mapper_node.cpp)
add_executable(apm_bridge src/apm_bridge.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(occupancy_grid_mapper_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
)

target_link_libraries(occupancy_grid_mapper_node ${PCL_LIBRARIES})

ament_target_dependencies(apm_bridge
 rclcpp
 geometry_msgs
 mocap_msgs
)

# Install executables and/or libraries
install(TARGETS
  occupancy_grid_mapper_node
  apm_bridge
  DESTINATION lib/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
