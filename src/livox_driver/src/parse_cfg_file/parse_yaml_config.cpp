#include "parse_yaml_config.h"

#include <yaml-cpp/yaml.h>

namespace livox_ros
{

  bool ParseYamlConfig(const std::string &path, YamlConfig &config)
  {
    if (path.empty())
    {
      return false;
    }

    YAML::Node root;
    try
    {
      root = YAML::LoadFile(path);

      if (root["device"])
      {
        const YAML::Node &device = root["device"];
        if (device["device_type"])
        {
          config.device_type = device["device_type"].as<std::string>();
        }
        if (device["point_cloud_format"])
        {
          config.point_cloud_format = device["point_cloud_format"].as<int>();
        }
        if (device["publish_frequency"])
        {
          config.publish_frequency = device["publish_frequency"].as<double>();
        }
        if (device["separate_topic_per_lidar"])
        {
          config.separate_topic_per_lidar = device["separate_topic_per_lidar"].as<bool>();
        }
        if (device["frame_id"])
        {
          config.frame_id = device["frame_id"].as<std::string>();
        }
      }

      if (root["topics"])
      {
        const YAML::Node &topics = root["topics"];
        if (topics["lidar_publisher"])
        {
          config.point_cloud_topic = topics["lidar_publisher"].as<std::string>();
        }
        if (topics["imu_publisher"])
        {
          config.imu_topic = topics["imu_publisher"].as<std::string>();
        }
      }

      if (root["qos"])
      {
        const YAML::Node &qos = root["qos"];
        if (qos["reliability"])
        {
          config.qos.reliability = qos["reliability"].as<std::string>();
        }
        if (qos["durability"])
        {
          config.qos.durability = qos["durability"].as<std::string>();
        }
        if (qos["history"])
        {
          config.qos.history = qos["history"].as<std::string>();
        }
        if (qos["queue_depth"])
        {
          config.qos.queue_depth = qos["queue_depth"].as<int>();
        }
      }

      if (root["filters"])
      {
        const YAML::Node &filters = root["filters"];
        if (filters["min_range"])
        {
          config.filters.min_range = filters["min_range"].as<double>();
          config.filters.enable = true;
        }
        if (filters["max_range"])
        {
          config.filters.max_range = filters["max_range"].as<double>();
          config.filters.enable = true;
        }
        if (filters["min_height"])
        {
          config.filters.min_height = filters["min_height"].as<double>();
          config.filters.enable = true;
        }
        if (filters["max_height"])
        {
          config.filters.max_height = filters["max_height"].as<double>();
          config.filters.enable = true;
        }
      }
    }
    catch (const std::exception &e)
    {
      return false;
    }

    config.loaded = true;
    return true;
  }

}
