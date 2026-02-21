cmake_minimum_required(VERSION 3.8)
project(milton_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(qbot_platform_driver_interface src/qbot_platform_driver_interface.cpp)
target_link_libraries(qbot_platform_driver_interface -lquanser_communications)
ament_target_dependencies(qbot_platform_driver_interface rclcpp std_msgs geometry_msgs sensor_msgs control_msgs)

add_executable(command src/command.cpp)
target_link_libraries(command -lquanser_devices -lquanser_communications -lquanser_runtime -lquanser_common)
ament_target_dependencies(command rclcpp geometry_msgs)

install(TARGETS qbot_platform_driver_interface
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS command
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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
