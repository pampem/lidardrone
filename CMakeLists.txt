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
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare a C++ executable
add_executable(occupancy_grid_mapper_node src/occupancy_grid_mapper_node.cpp)
add_executable(glim_apm_bridge src/glim_apm_bridge.cpp)
add_executable(mocap_apm_bridge src/mocap_apm_bridge.cpp)
add_executable(drone_controller src/drone_controller.cpp)
add_executable(eval_glim src/eval_glim.cpp)
add_executable(glim_mocap_apm_bridge src/glim_mocap_apm_bridge.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(occupancy_grid_mapper_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
)

target_link_libraries(occupancy_grid_mapper_node ${PCL_LIBRARIES})

ament_target_dependencies(glim_apm_bridge
 rclcpp
 geometry_msgs
 mocap_msgs
 mavros_msgs
)

ament_target_dependencies(mocap_apm_bridge
 rclcpp
 geometry_msgs
 mocap_msgs
 mavros_msgs
)

ament_target_dependencies(glim_apm_mocap_bridge
 rclcpp
 geometry_msgs
 mocap_msgs
 mavros_msgs
)

ament_target_dependencies(drone_controller
 rclcpp
 geometry_msgs
 mocap_msgs
 mavros_msgs
)

ament_target_dependencies(eval_glim
 rclcpp
 geometry_msgs
 mocap_msgs
 mavros_msgs
 tf2_geometry_msgs
 tf2
)

# Install executables and/or libraries
install(TARGETS
  occupancy_grid_mapper_node
  glim_apm_bridge
  glim_mocap_apm_bridge
  drone_controller
  eval_glim
  mocap_apm_bridge
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
