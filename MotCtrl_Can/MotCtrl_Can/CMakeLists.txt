cmake_minimum_required(VERSION 3.8)
project(MotCtrl_Can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(ros2_socketcan_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(MotCtrl_node src/MotCtrl_node.cpp src/dm_motor_driver.cpp src/encos_motor_driver.cpp src/SocketCAN.cpp)

ament_target_dependencies(
  MotCtrl_node
  "rclcpp"
  "ros2_socketcan"
  "ros2_socketcan_msgs"
  "sensor_msgs"
)

target_link_libraries(MotCtrl_node fmt::fmt spdlog::spdlog)

install(TARGETS
  MotCtrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
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
