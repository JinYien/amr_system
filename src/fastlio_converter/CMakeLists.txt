cmake_minimum_required(VERSION 3.8)
project(fastlio_converter)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(fastlio_converter_node
  src/node.cpp
  src/scan_builder.cpp
  src/settings.cpp
)
target_include_directories(fastlio_converter_node PRIVATE include)
ament_target_dependencies(fastlio_converter_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(fastlio_converter_node yaml-cpp)

install(TARGETS fastlio_converter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
