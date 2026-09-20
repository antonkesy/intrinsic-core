// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/perception/cameras/sensor_information.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/pixel_type.h"

namespace intrinsic {
namespace perception {

SensorInformation::SensorInformation(
    int64_t id, absl::string_view display_name,
    std::optional<CameraParams> factory_camera_params,
    std::optional<Pose> camera_t_sensor,
    std::vector<PixelType> supported_pixel_types, Dimensions dimensions,
    bool disabled)
    : id_(id),
      display_name_(display_name),
      factory_camera_params_(factory_camera_params),
      camera_t_sensor_(camera_t_sensor),
      supported_pixel_types_(supported_pixel_types),
      dimensions_(dimensions),
      disabled_(disabled) {}

int64_t SensorInformation::id() const { return id_; }

std::string SensorInformation::display_name() const { return display_name_; }

const std::optional<CameraParams>& SensorInformation::factory_camera_params()
    const {
  return factory_camera_params_;
}

std::optional<Pose> SensorInformation::camera_t_sensor() const {
  return camera_t_sensor_;
}

std::vector<PixelType> SensorInformation::supported_pixel_types() const {
  return supported_pixel_types_;
}
Dimensions SensorInformation::dimensions() const { return dimensions_; }

bool SensorInformation::disabled() const { return disabled_; }

}  // namespace perception
}  // namespace intrinsic
