cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nuturtle_description REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(turtlelib_ros REQUIRED)
ament_export_dependencies(rosidl_default_runtime)

rosidl_generate_interfaces(
  ${PROJECT_NAME}_srv
  "srv/InitialPose.srv"
  "srv/Circle.srv"
  LIBRARY_NAME ${PROJECT_NAME}
  DEPENDENCIES
  nuturtle_description
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

# executables
add_executable(turtle_control src/turtle_control.cpp)
target_link_libraries(turtle_control turtlelib::turtlelib)
ament_target_dependencies(turtle_control
  rclcpp geometry_msgs sensor_msgs nuturtlebot_msgs
)

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry
  turtlelib::turtlelib turtlelib_ros::turtlelib_ros "${cpp_typesupport_target}"
)
ament_target_dependencies(odometry
  rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

add_executable(circle src/circle.cpp)
target_link_libraries(circle "${cpp_typesupport_target}")
ament_target_dependencies(circle
  rclcpp geometry_msgs std_srvs
)

install(TARGETS
  turtle_control odometry circle
  DESTINATION lib/${PROJECT_NAME}
)

# install files
install(DIRECTORY
  launch config
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
