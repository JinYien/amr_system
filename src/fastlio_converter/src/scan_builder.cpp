#include "fastlio_converter/scan_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

LaserScanBuilder::LaserScanBuilder(
    const FilterSettings &filter, const FootprintSettings &footprint)
    : filter_settings(filter),
      half_length(footprint.length / 2.0 + footprint.margin),
      half_width(footprint.width / 2.0 + footprint.margin),
      bins(static_cast<int>(
               std::round((filter.angle_max - filter.angle_min) / filter.angle_increment)) +
           1),
      rotation{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
      translation{0.0, 0.0, 0.0},
      range_bins(bins, std::numeric_limits<float>::infinity())
{
}

void LaserScanBuilder::set_sensor_offset(
    const std::array<std::array<double, 3>, 3> &sensor_rotation,
    const std::array<double, 3> &sensor_translation)
{
  rotation = sensor_rotation;
  translation = sensor_translation;
}

void LaserScanBuilder::reset()
{
  std::fill(range_bins.begin(), range_bins.end(), std::numeric_limits<float>::infinity());
}

void LaserScanBuilder::add_point(double px, double py, double pz)
{
  const double x = rotation[0][0] * px + rotation[0][1] * py + rotation[0][2] * pz + translation[0];
  const double y = rotation[1][0] * px + rotation[1][1] * py + rotation[1][2] * pz + translation[1];
  const double z = rotation[2][0] * px + rotation[2][1] * py + rotation[2][2] * pz + translation[2];

  if (z < filter_settings.min_height || z > filter_settings.max_height)
  {
    return;
  }
  if (std::abs(x) <= half_length && std::abs(y) <= half_width)
  {
    return;
  }

  const double range = std::hypot(x, y);
  if (range < filter_settings.range_min || range > filter_settings.range_max)
  {
    return;
  }

  const double angle = std::atan2(y, x);
  if (angle < filter_settings.angle_min || angle > filter_settings.angle_max)
  {
    return;
  }

  const int index = static_cast<int>(
      std::lround((angle - filter_settings.angle_min) / filter_settings.angle_increment));
  if (index < 0 || index >= bins)
  {
    return;
  }
  if (range < range_bins[index])
  {
    range_bins[index] = static_cast<float>(range);
  }
}

const std::vector<float> &LaserScanBuilder::ranges() const
{
  return range_bins;
}

const FilterSettings &LaserScanBuilder::filter() const
{
  return filter_settings;
}
