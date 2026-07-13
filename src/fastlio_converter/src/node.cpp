#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "fastlio_converter/scan_builder.hpp"
#include "fastlio_converter/settings.hpp"

using std::placeholders::_1;

class ConverterNode : public rclcpp::Node
{
public:
  ConverterNode()
      : Node("fastlio_converter_node"),
        settings(load_settings(declare_parameter<std::string>("config_path", ""))),
        builder(settings.filter, settings.footprint)
  {
    // ======================================================
    RCLCPP_INFO(get_logger(), "Initializing node ...");
    // ======================================================
    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // ======================================================
    RCLCPP_INFO(get_logger(), "Publishing & subscribing topics ...");
    // ======================================================
    scan_publisher = create_publisher<sensor_msgs::msg::LaserScan>(
        settings.topics.laser_scan_publisher, 10);
    cloud_subscription = create_subscription<sensor_msgs::msg::PointCloud2>(
        settings.topics.point_cloud_subscriber, 10,
        std::bind(&ConverterNode::on_cloud, this, _1));

    // ======================================================
    RCLCPP_INFO(get_logger(), "Running process ...");
    // ======================================================
  }

private:
  bool ensure_sensor_offset()
  {
    if (offset_ready)
    {
      return true;
    }
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer->lookupTransform(
          settings.frames.base, settings.frames.sensor, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &)
    {
      return false;
    }
    const auto &t = transform.transform.translation;
    const auto &q = transform.transform.rotation;
    const tf2::Matrix3x3 matrix(tf2::Quaternion(q.x, q.y, q.z, q.w));
    std::array<std::array<double, 3>, 3> rotation;
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        rotation[row][col] = matrix[row][col];
      }
    }
    const std::array<double, 3> translation = {t.x, t.y, t.z};
    builder.set_sensor_offset(rotation, translation);
    offset_ready = true;
    return true;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    if (!ensure_sensor_offset())
    {
      if (!warned)
      {
        RCLCPP_WARN(
            get_logger(),
            "TF lookup %s -> %s failed in ensure_sensor_offset, no scan is published until robot_state_publisher provides it",
            settings.frames.base.c_str(), settings.frames.sensor.c_str());
        warned = true;
      }
      return;
    }

    builder.reset();
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*message, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z))
      {
        builder.add_point(*iter_x, *iter_y, *iter_z);
      }
    }

    publish_scan(message);
  }

  void publish_scan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message)
  {
    const FilterSettings &filter = builder.filter();
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = message->header.stamp;
    scan.header.frame_id = settings.frames.base;
    scan.angle_min = static_cast<float>(filter.angle_min);
    scan.angle_max = static_cast<float>(filter.angle_max);
    scan.angle_increment = static_cast<float>(filter.angle_increment);
    scan.time_increment = 0.0f;
    scan.scan_time = 0.0f;
    scan.range_min = static_cast<float>(filter.range_min);
    scan.range_max = static_cast<float>(filter.range_max);
    scan.ranges = builder.ranges();
    scan_publisher->publish(scan);
  }

  Settings settings;
  LaserScanBuilder builder;

  bool offset_ready = false;
  bool warned = false;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConverterNode>());
  rclcpp::shutdown();
  return 0;
}
