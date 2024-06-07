cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(turtlelib)
find_package(turtlelib_ros)
find_package(Armadillo)
ament_export_dependencies(rosidl_default_runtime)

include_directories(${ARMADILLO_INCLUDE_DIRS})

rosidl_generate_interfaces(
  ${PROJECT_NAME}_msg
  "msg/Landmark.msg"
  "msg/Landmarks.msg"
  LIBRARY_NAME ${PROJECT_NAME}
  DEPENDENCIES
  geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msg "rosidl_typesupport_cpp")


add_executable(nuslam src/nuslam.cpp)
target_link_libraries(nuslam
  turtlelib::turtlelib turtlelib_ros::turtlelib_ros ${ARMADILLO_LIBRARIES}
  "${cpp_typesupport_target}"
)
ament_target_dependencies(nuslam
  rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs
)

add_executable(landmarks src/landmarks.cpp)
target_link_libraries(landmarks
  turtlelib::turtlelib ${ARMADILLO_LIBRARIES} "${cpp_typesupport_target}"
)
ament_target_dependencies(landmarks
  rclcpp sensor_msgs visualization_msgs
)

add_executable(real_lidar_visualize src/real_lidar_visualize.cpp)
ament_target_dependencies(real_lidar_visualize
  rclcpp sensor_msgs
)

# install targets
install(TARGETS
  nuslam landmarks real_lidar_visualize
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
