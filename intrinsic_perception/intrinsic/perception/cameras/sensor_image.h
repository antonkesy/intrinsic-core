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

#ifndef INTRINSIC_PERCEPTION_PUBLIC_SENSOR_IMAGE_H_
#define INTRINSIC_PERCEPTION_PUBLIC_SENSOR_IMAGE_H_

#include <cstdint>
#include <optional>
#include <utility>

#include "absl/time/time.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"

namespace intrinsic {
namespace perception {

// Data class associated with SensorImage protobuf.
// Contains an image from a single sensor on a multiframe camera.
class SensorImage {
 public:
  SensorImage() = delete;

  // Constructor from a specific image.
  template <class Traits>
  SensorImage(int64_t sensor_id, absl::Time acquisition_time,
              std::optional<CameraParams> camera_params,
              std::optional<Pose> camera_t_sensor, Image<Traits>&& buffer);

  // Special constructor with new pose.
  SensorImage(SensorImage&& image, const Pose& camera_t_sensor);

  // Special constructor with new image.
  template <class Traits>
  SensorImage(const SensorImage& image, Image<Traits>&& buffer);

  bool IsEmpty() const;

  int64_t sensor_id() const;
  const absl::Time& acquisition_time() const;
  const std::optional<Pose>& camera_t_sensor() const;

  const std::optional<CameraParams>& camera_params() const;

  const std::optional<Image<Gray8u>>& gray8u() const&;
  const std::optional<Image<Rgb8u>>& rgb8u() const&;
  const std::optional<Image<Gray32f>>& gray32f() const&;
  const std::optional<Image<Depth32f>>& depth32f() const&;
  const std::optional<Image<Point32f>>& point32f() const&;
  const std::optional<Image<Normal32f>>& normal32f() const&;

  std::optional<Image<Gray8u>> gray8u() &&;
  std::optional<Image<Rgb8u>> rgb8u() &&;
  std::optional<Image<Gray32f>> gray32f() &&;
  std::optional<Image<Depth32f>> depth32f() &&;
  std::optional<Image<Point32f>> point32f() &&;
  std::optional<Image<Normal32f>> normal32f() &&;

  Dimensions Dimensions() const;

 private:
  int64_t sensor_id_;
  absl::Time acquisition_time_;
  std::optional<CameraParams> camera_params_;
  std::optional<Pose> camera_t_sensor_;

  // Exactly one will be set. Used optionals instead of variant for easy
  // reference returns
  std::optional<Image<Rgb8u>> rgb8u_;
  std::optional<Image<Gray8u>> gray8u_;
  std::optional<Image<Gray32f>> gray32f_;
  std::optional<Image<Depth32f>> depth32f_;
  std::optional<Image<Point32f>> point32f_;
  std::optional<Image<Normal32f>> normal32f_;

  friend class SensorImageBuilder;
};

// Template definitions.

template <class Traits>
SensorImage::SensorImage(int64_t sensor_id, absl::Time acquisition_time,
                         std::optional<CameraParams> camera_params,
                         std::optional<Pose> camera_t_sensor,
                         Image<Traits>&& buffer)
    : sensor_id_(sensor_id),
      acquisition_time_(acquisition_time),
      camera_params_(camera_params),
      camera_t_sensor_(camera_t_sensor) {
  if constexpr (std::is_same_v<Rgb8u, Traits>) {
    rgb8u_ = std::move(buffer);
  } else if constexpr (std::is_same_v<Gray8u, Traits>) {
    gray8u_ = std::move(buffer);
  } else if constexpr (std::is_same_v<Gray32f, Traits>) {
    gray32f_ = std::move(buffer);
  } else if constexpr (std::is_same_v<Depth32f, Traits>) {
    depth32f_ = std::move(buffer);
  } else if constexpr (std::is_same_v<Point32f, Traits>) {
    point32f_ = std::move(buffer);
  } else if constexpr (std::is_same_v<Normal32f, Traits>) {
    normal32f_ = std::move(buffer);
  } else {
    static_assert(false, "Unsupported image trait.");
  }
}

template <class Traits>
SensorImage::SensorImage(const SensorImage& image, Image<Traits>&& buffer)
    : SensorImage(image.sensor_id(), image.acquisition_time(),
                  image.camera_params(), image.camera_t_sensor(),
                  std::move(buffer)) {}

// Builder class for SensorImage.
//
// Example usage:
//   SensorImage sensor_image = SensorImageBuilder()
//                                 .From(std::move(source_sensor_image))
//                                 .SetSensorId(123)
//                                 .SetCameraParams(camera_params)
//                                 .Build();
class SensorImageBuilder {
 public:
  SensorImageBuilder();
  SensorImageBuilder& From(SensorImage&& image);

  SensorImageBuilder(SensorImageBuilder&& builder) = default;
  SensorImageBuilder& operator=(SensorImageBuilder&& builder) = default;
  SensorImageBuilder(const SensorImageBuilder&) = delete;
  SensorImageBuilder& operator=(const SensorImageBuilder&) = delete;

  SensorImageBuilder& SetSensorId(int64_t sensor_id);
  SensorImageBuilder& SetAcquisitionTime(absl::Time acquisition_time);
  SensorImageBuilder& SetCameraParams(
      std::optional<CameraParams> camera_params);
  SensorImageBuilder& SetCameraTSensor(std::optional<Pose> camera_t_sensor);

  template <class Traits>
  SensorImageBuilder& SetImage(Image<Traits>&& image);

  SensorImage Build();

 private:
  SensorImage image_;
  bool is_built_ = false;
};

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_PUBLIC_SENSOR_IMAGE_H_
