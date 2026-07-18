cmake_minimum_required(VERSION 3.8)
project(racing_control_2026)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ai_msgs REQUIRED)

# Include directories
include_directories(include)

# Source files
set(SOURCES
  src/main.cpp
  src/racing_controller.cpp
  src/pid_controller.cpp
  src/navigator.cpp
  src/route_manager.cpp
  src/obstacle_avoider.cpp
  src/phase_manager.cpp
  src/external_services.cpp
)

# Add executable
add_executable(racing_controller ${SOURCES})

# Link dependencies
ament_target_dependencies(racing_controller
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ai_msgs
)

# Install targets
install(TARGETS
  racing_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
