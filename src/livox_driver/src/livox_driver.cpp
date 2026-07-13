#include <chrono>
#include <thread>

#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"
#include "parse_cfg_file/parse_yaml_config.h"

using namespace livox_ros;

namespace livox_ros
{

  static std::string ResolveLidarConfigPath(const std::string &config_path,
                                            const std::string &device_type)
  {
    if (device_type.empty())
    {
      return device_type;
    }
    size_t pos = config_path.find_last_of('/');
    std::string dir = (pos == std::string::npos) ? std::string(".") : config_path.substr(0, pos);
    return dir + "/" + device_type + ".json";
  }

  DriverNode::DriverNode(const rclcpp::NodeOptions &node_options)
      : Node("livox_driver_node", node_options)
  {
    DRIVER_INFO(*this, "Initializing node ...");
    this->declare_parameter("config_path", "");
    std::string config_path;
    this->get_parameter("config_path", config_path);

    YamlConfig config;
    if (config_path.empty() || !ParseYamlConfig(config_path, config))
    {
      DRIVER_ERROR(*this, "Could not read the config file '%s'; using default settings", config_path.c_str());
    }

    double publish_freq = config.publish_frequency;
    if (publish_freq > 100.0)
    {
      publish_freq = 100.0;
    }
    else if (publish_freq < 0.5)
    {
      publish_freq = 0.5;
    }

    future_ = exit_signal_.get_future();

    DRIVER_INFO(*this, "Publishing & subscribing topics ...");
    lddc_ptr_ = std::make_unique<Lddc>(config.point_cloud_format,
                                       config.separate_topic_per_lidar ? 1 : 0, config.frame_id);
    lddc_ptr_->SetRosNode(this);
    lddc_ptr_->SetYamlConfig(config);

    std::string lidar_config_path = ResolveLidarConfigPath(config_path, config.device_type);
    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));
    if (!read_lidar->InitLdsLidar(lidar_config_path))
    {
      DRIVER_ERROR(*this, "Could not start the lidar; check that it is connected and powered on");
    }

    DRIVER_INFO(*this, "Running process ...");
    pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
    imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
  }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do
  {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do
  {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}
