cmake_minimum_required(VERSION 3.8)
project(riptide_turtle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

# add the C++ executable
add_executable(turtle_node src/turtle_node.cpp)
add_executable(turtle_kill_switch_publisher src/turtle_kill_switch_publisher.cpp)

# link the necessary libraries
ament_target_dependencies(turtle_node rclcpp geometry_msgs std_msgs example_interfaces)
ament_target_dependencies(turtle_kill_switch_publisher rclcpp std_msgs)

# install the C++ executables
install(TARGETS
  turtle_node
  turtle_kill_switch_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
