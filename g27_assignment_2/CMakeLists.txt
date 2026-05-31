cmake_minimum_required(VERSION 3.8)
project(g27_assignment_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -------------------------------------------------------------------------
# Generate custom service
# -------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCubePoses.srv"
  DEPENDENCIES geometry_msgs
)

# Typesupport target produced by rosidl_generate_interfaces above
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# -------------------------------------------------------------------------
# pose_estimator_node
# -------------------------------------------------------------------------
add_executable(pose_estimator_node
  src/pose_estimator_node.cpp
)
target_include_directories(pose_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pose_estimator_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(pose_estimator_node
  ${cpp_typesupport_target}
)

# -------------------------------------------------------------------------
# cube_swap_node
# -------------------------------------------------------------------------
add_executable(cube_swap_node
  src/cube_swap_node.cpp
)
target_include_directories(cube_swap_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cube_swap_node
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2_eigen
  moveit_ros_planning_interface
  moveit_task_constructor_core
)
target_link_libraries(cube_swap_node
  ${cpp_typesupport_target}
)

# -------------------------------------------------------------------------
# Install
# -------------------------------------------------------------------------
install(TARGETS
  pose_estimator_node
  cube_swap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  srv
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
