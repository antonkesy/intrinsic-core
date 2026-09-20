#ifndef INTRINSIC_PERCEPTION_CAMERAS_SENSOR_PIXEL_SIZE_SENSOR_PIXEL_SIZES_H_
#define INTRINSIC_PERCEPTION_CAMERAS_SENSOR_PIXEL_SIZE_SENSOR_PIXEL_SIZES_H_

#include <string_view>

#include "absl/container/flat_hash_map.h"

namespace intrinsic::perception {

inline const absl::flat_hash_map<std::string_view,
                                 absl::flat_hash_map<std::string_view, double>>&
SensorPixelSizes() {
  static const auto& sensor_pixel_sizes =
      *new absl::flat_hash_map<std::string_view,
                               absl::flat_hash_map<std::string_view, double>>{
%SENSOR_PIXEL_SIZES%
      };
  return sensor_pixel_sizes;
}

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CAMERAS_SENSOR_PIXEL_SIZE_SENSOR_PIXEL_SIZES_H_
