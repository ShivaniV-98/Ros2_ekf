cmake_minimum_required(VERSION 3.9)
project(turtlelib_ros)

# Required packages
find_package(turtlelib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# create library
add_library(turtlelib_ros src/convert.cpp)

# link libraries
target_link_libraries(turtlelib_ros turtlelib::turtlelib)
ament_target_dependencies(turtlelib_ros
  geometry_msgs tf2 tf2_geometry_msgs
)

# public include directories
target_include_directories(turtlelib_ros
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# enable C++ 17
target_compile_features(turtlelib_ros PUBLIC cxx_std_17) 

# warnings are your friend!
target_compile_options(turtlelib_ros PUBLIC -Wall -Wextra -Wpedantic)

# CMake also has the ability to generate doxygen documentation
if(NOT CMAKE_CROSSCOMPILING)
    find_package(Doxygen)
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
    doxygen_add_docs(doxygen include/ src/ README.md ALL)
endif()

# install include directories
install(DIRECTORY include/ DESTINATION include)

# install targets
install(TARGETS turtlelib_ros EXPORT turtlelib_ros-targets)

# export targets
install(EXPORT turtlelib_ros-targets
        FILE turtlelib_ros-config.cmake
        NAMESPACE turtlelib_ros::
        DESTINATION lib/cmake/${PROJECT_NAME})