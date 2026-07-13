#ifndef FASTLIO_CONVERTER_SETTINGS_HPP
#define FASTLIO_CONVERTER_SETTINGS_HPP

#include <string>

struct TopicSettings
{
  std::string point_cloud_subscriber;
  std::string laser_scan_publisher;
};

struct FrameSettings
{
  std::string base;
  std::string sensor;
};

struct FilterSettings
{
  double min_height;
  double max_height;
  double range_min;
  double range_max;
  double angle_min;
  double angle_max;
  double angle_increment;
};

struct FootprintSettings
{
  double length;
  double width;
  double margin;
};

struct Settings
{
  TopicSettings topics;
  FrameSettings frames;
  FilterSettings filter;
  FootprintSettings footprint;
};

Settings load_settings(const std::string &path);

#endif // FASTLIO_CONVERTER_SETTINGS_HPP
