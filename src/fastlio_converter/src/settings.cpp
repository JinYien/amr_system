#include "fastlio_converter/settings.hpp"

#include <cmath>

#include <yaml-cpp/yaml.h>

namespace
{
  double radians(double degrees)
  {
    return degrees * M_PI / 180.0;
  }
} // namespace

Settings load_settings(const std::string &path)
{
  const YAML::Node root = YAML::LoadFile(path);

  Settings settings;

  const YAML::Node topics = root["topics"];
  settings.topics.point_cloud_subscriber = topics["point_cloud_subscriber"].as<std::string>();
  settings.topics.laser_scan_publisher = topics["laser_scan_publisher"].as<std::string>();

  const YAML::Node frames = root["frames"];
  settings.frames.base = frames["base"].as<std::string>();
  settings.frames.sensor = frames["sensor"].as<std::string>();

  const YAML::Node filter = root["filter"];
  settings.filter.min_height = filter["min_height"].as<double>();
  settings.filter.max_height = filter["max_height"].as<double>();
  settings.filter.range_min = filter["range_min"].as<double>();
  settings.filter.range_max = filter["range_max"].as<double>();
  settings.filter.angle_min = radians(filter["angle_min"].as<double>());
  settings.filter.angle_max = radians(filter["angle_max"].as<double>());
  settings.filter.angle_increment = radians(filter["angle_increment"].as<double>());

  const YAML::Node footprint = root["footprint"];
  settings.footprint.length = footprint["length"].as<double>();
  settings.footprint.width = footprint["width"].as<double>();
  settings.footprint.margin = footprint["margin"].as<double>();

  return settings;
}
