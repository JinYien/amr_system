cmake_minimum_required(VERSION 3.14)
project(livox_driver)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-mcpu=cortex-a78ae" HAVE_MCPU_CORTEX_A78AE)
  if(HAVE_MCPU_CORTEX_A78AE)
    add_compile_options(-mcpu=cortex-a78ae)
  endif()
endif()

include(cmake/version.cmake)
project(${PROJECT_NAME} VERSION ${LIVOX_DRIVER_VERSION} LANGUAGES CXX)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

set(LIVOX_INTERFACES livox_interfaces2)
rosidl_generate_interfaces(${LIVOX_INTERFACES}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)
find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES "livox_lidar_api.h" "livox_lidar_def.h"
  REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/livox_driver.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp
  src/parse_cfg_file/parse_yaml_config.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")

target_include_directories(${PROJECT_NAME} PUBLIC
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
  3rdparty
  src
)

target_link_libraries(${PROJECT_NAME}
  ${LIVOX_LIDAR_SDK_LIBRARY}
  yaml-cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::DriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
