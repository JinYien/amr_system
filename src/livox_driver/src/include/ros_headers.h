#ifndef ROS_HEADERS_H_
#define ROS_HEADERS_H_

#include <thread>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "livox_driver/msg/custom_point.hpp"
#include "livox_driver/msg/custom_msg.hpp"

#define DRIVER_INFO(node, ...) RCLCPP_INFO((node).get_logger(), __VA_ARGS__)
#define DRIVER_WARN(node, ...) RCLCPP_WARN((node).get_logger(), __VA_ARGS__)
#define DRIVER_ERROR(node, ...) RCLCPP_ERROR((node).get_logger(), __VA_ARGS__)

#endif
