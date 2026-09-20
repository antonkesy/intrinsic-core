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

#ifndef INTRINSIC_PERCEPTION_PUBLIC_SENSOR_INFORMATION_H_
#define INTRINSIC_PERCEPTION_PUBLIC_SENSOR_INFORMATION_H_

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

// Convenience class for the SensorInformation proto. Contains pertinent
// information about a single sensor in a multi-sensor camera.
class SensorInformation {
 public:
  SensorInformation() = delete;

  SensorInformation(int64_t id, absl::string_view display_name,
                    std::optional<CameraParams> factory_camera_params,
                    std::optional<Pose> camera_t_sensor,
                    std::vector<PixelType> supported_pixel_types,
                    Dimensions dimensions, bool disabled);

  int64_t id() const;
  std::string display_name() const;
  const std::optional<CameraParams>& factory_camera_params() const;
  std::optional<Pose> camera_t_sensor() const;
  std::vector<PixelType> supported_pixel_types() const;
  Dimensions dimensions() const;
  bool disabled() const;

 private:
  int64_t id_;
  std::string display_name_;
  std::optional<CameraParams> factory_camera_params_;
  std::optional<Pose> camera_t_sensor_;
  std::vector<PixelType> supported_pixel_types_;
  Dimensions dimensions_;
  bool disabled_;
};

struct DescribeCameraResponse {
  std::vector<SensorInformation> sensors;
};

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_PUBLIC_SENSOR_INFORMATION_H_
