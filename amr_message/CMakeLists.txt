cmake_minimum_required(VERSION 3.8)
project(amr_message)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Command.msg"
  "msg/Control.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/Intersection.msg"
  "msg/PidCommand.msg"
  "msg/PidGain.msg"
  "msg/State.msg"
  "msg/Teensy.msg"
)

install(FILES
  amr_message/qos.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp rclcpp yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()