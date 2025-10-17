## Project Setup
cmake_minimum_required(VERSION 3.22)

project(segway_rmp_ros2)

# Default to C++23.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(tf2_geometry_msgs REQUIRED)
find_package(libsegwayrmp REQUIRED)  # Include libsegwayrmp
find_package(rosidl_default_generators REQUIRED)

# Specify your message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SegwayStatus.msg"
  "msg/SegwayStatusStamped.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# Add your executable
add_executable(segway_rmp_node_ros2 src/segway_ros2_rmp.cpp)

target_include_directories(segway_rmp_node_ros2 PUBLIC
  include
  ${CMAKE_INSTALL_PREFIX}/libsegwayrmp/include  # Add this line to include the install directory
)

# Link libraries
target_link_libraries(segway_rmp_node_ros2
  ${Boost_LIBRARIES}
  ${libsegwayrmp_INCLUDE_DIRS}
)

ament_target_dependencies(segway_rmp_node_ros2
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
  Boost
  tf2_geometry_msgs
  libsegwayrmp  # Add segwayrmp to ament_target_dependencies
)

# Ensure that message dependencies are properly exported
rosidl_target_interfaces(segway_rmp_node_ros2
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
  segway_rmp_node_ros2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

# Export package dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(libsegwayrmp)  # Export segwayrmp dependency
ament_package()