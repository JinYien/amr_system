cmake_minimum_required(VERSION 3.8)
project(custom_message)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/DriveCommand.msg"
  "msg/HandleCommand.msg"
  "msg/Intersection.msg"
  "msg/JoystickCommand.msg"
  "msg/MixControl.msg"
  "msg/Sound.msg"
  "msg/Teensy.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()