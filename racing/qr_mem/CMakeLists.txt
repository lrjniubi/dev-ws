cmake_minimum_required(VERSION 3.5)
project(qr_mem)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(origincar_msg REQUIRED)

# zbar library
find_package(PkgConfig)
pkg_check_modules(ZBAR REQUIRED zbar)

# Include directories
include_directories(
  include
  ${ZBAR_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Generate interfaces for the messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SampleMessage.msg"
  "msg/SignSwitch.msg"
  DEPENDENCIES std_msgs
)

# Add executable
add_executable(qr_mem_node src/qr_mem.cpp)
ament_target_dependencies(qr_mem_node 
  rclcpp
  hbm_img_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  origincar_msg
)

# Link the generated message interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(qr_mem_node "${cpp_typesupport_target}")

# Link external libraries
target_link_libraries(qr_mem_node
  ${ZBAR_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Install targets
install(TARGETS
  qr_mem_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files if they exist
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install configuration files if they exist
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
