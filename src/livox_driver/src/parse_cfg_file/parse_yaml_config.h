#ifndef LIVOX_DRIVER_PARSE_YAML_CONFIG_H_
#define LIVOX_DRIVER_PARSE_YAML_CONFIG_H_

#include <string>

namespace livox_ros
{

  struct YamlQosConfig
  {
    std::string reliability = "reliable";
    std::string durability = "volatile";
    std::string history = "keep_last";
    int queue_depth = 10;
  };

  struct YamlFilterConfig
  {
    bool enable = false;
    double min_range = 0.0;
    double max_range = 0.0;
    double min_height = 0.0;
    double max_height = 0.0;
  };

  struct YamlConfig
  {
    bool loaded = false;
    std::string device_type;
    int point_cloud_format = 0;
    double publish_frequency = 10.0;
    bool separate_topic_per_lidar = false;
    std::string frame_id = "livox_frame";
    std::string point_cloud_topic;
    std::string imu_topic;
    YamlQosConfig qos;
    YamlFilterConfig filters;
  };

  bool ParseYamlConfig(const std::string &path, YamlConfig &config);

}

#endif
