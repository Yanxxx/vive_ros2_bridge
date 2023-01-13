cmake_minimum_required(VERSION 3.8)
project(vive_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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

# htc-vive-ros2-bridge.cpp

include_directories("./include/")

find_library(OPENVR_LIB openvr_api "lib/" NO_DEFAULT_PATH) #TODO other operating systems
MESSAGE ("Found libopenvr_api: ${OPENVR_LIB}")

add_executable(vive_ros2_bridge src/vive_ros2_bridge.cpp)
target_link_libraries (vive_ros2_bridge openvr_api ${OPENVR_LIB})
ament_target_dependencies(
   vive_ros2_bridge
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)

install(TARGETS
   vive_ros2_bridge
   DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY lib/
  DESTINATION lib
)
