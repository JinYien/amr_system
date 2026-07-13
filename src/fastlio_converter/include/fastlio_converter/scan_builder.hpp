#ifndef FASTLIO_CONVERTER_SCAN_BUILDER_HPP
#define FASTLIO_CONVERTER_SCAN_BUILDER_HPP

#include <array>
#include <vector>

#include "fastlio_converter/settings.hpp"

class LaserScanBuilder
{
public:
  LaserScanBuilder(const FilterSettings &filter, const FootprintSettings &footprint);

  void set_sensor_offset(
      const std::array<std::array<double, 3>, 3> &sensor_rotation,
      const std::array<double, 3> &sensor_translation);

  void reset();
  void add_point(double x, double y, double z);

  const std::vector<float> &ranges() const;
  const FilterSettings &filter() const;

private:
  FilterSettings filter_settings;
  double half_length;
  double half_width;
  int bins;

  std::array<std::array<double, 3>, 3> rotation;
  std::array<double, 3> translation;
  std::vector<float> range_bins;
};

#endif // FASTLIO_CONVERTER_SCAN_BUILDER_HPP
