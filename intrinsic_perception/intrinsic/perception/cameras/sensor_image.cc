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

#include "intrinsic/perception/cameras/sensor_image.h"

#include <cstdint>
#include <optional>
#include <utility>

#include "absl/log/check.h"
#include "absl/time/time.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"

namespace intrinsic {
namespace perception {

SensorImage::SensorImage(SensorImage&& image, const Pose& camera_t_sensor)
    : sensor_id_(image.sensor_id()),
      acquisition_time_(std::move(image.acquisition_time_)),
      camera_params_(std::move(image.camera_params_)),
      camera_t_sensor_(camera_t_sensor),
      rgb8u_(std::move(image.rgb8u_)),
      gray8u_(std::move(image.gray8u_)),
      gray32f_(std::move(image.gray32f_)),
      depth32f_(std::move(image.depth32f_)),
      point32f_(std::move(image.point32f_)),
      normal32f_(std::move(image.normal32f_)) {}

bool SensorImage::IsEmpty() const {
  return !gray8u().has_value() && !rgb8u().has_value() &&
         !gray32f().has_value() && !depth32f().has_value() &&
         !point32f().has_value() && !normal32f().has_value();
}

int64_t SensorImage::sensor_id() const { return sensor_id_; }

const absl::Time& SensorImage::acquisition_time() const {
  return acquisition_time_;
}

const std::optional<Pose3d>& SensorImage::camera_t_sensor() const {
  return camera_t_sensor_;
}

const std::optional<CameraParams>& SensorImage::camera_params() const {
  return camera_params_;
}

const std::optional<Image<Gray8u>>& SensorImage::gray8u() const& {
  return gray8u_;
}

const std::optional<Image<Rgb8u>>& SensorImage::rgb8u() const& {
  return rgb8u_;
}

const std::optional<Image<Gray32f>>& SensorImage::gray32f() const& {
  return gray32f_;
}

const std::optional<Image<Depth32f>>& SensorImage::depth32f() const& {
  return depth32f_;
}

const std::optional<Image<Point32f>>& SensorImage::point32f() const& {
  return point32f_;
}

const std::optional<Image<Normal32f>>& SensorImage::normal32f() const& {
  return normal32f_;
}

std::optional<Image<Gray8u>> SensorImage::gray8u() && {
  return std::move(gray8u_);
}

std::optional<Image<Rgb8u>> SensorImage::rgb8u() && {
  return std::move(rgb8u_);
}

std::optional<Image<Gray32f>> SensorImage::gray32f() && {
  return std::move(gray32f_);
}

std::optional<Image<Depth32f>> SensorImage::depth32f() && {
  return std::move(depth32f_);
}

std::optional<Image<Point32f>> SensorImage::point32f() && {
  return std::move(point32f_);
}

std::optional<Image<Normal32f>> SensorImage::normal32f() && {
  return std::move(normal32f_);
}

Dimensions SensorImage::Dimensions() const {
  if (gray8u_.has_value()) {
    return gray8u_->dimensions();
  } else if (rgb8u_.has_value()) {
    return rgb8u_->dimensions();
  } else if (gray32f_.has_value()) {
    return gray32f_->dimensions();
  } else if (depth32f_.has_value()) {
    return depth32f_->dimensions();
  } else if (point32f_.has_value()) {
    return point32f_->dimensions();
  } else if (normal32f_.has_value()) {
    return normal32f_->dimensions();
  }
  return Dimensions();
}

SensorImageBuilder::SensorImageBuilder()
    : image_(SensorImage(0, absl::InfinitePast(), std::nullopt, std::nullopt,
                         Image<Rgb8u>(Dimensions(0, 0)))) {}

SensorImageBuilder& SensorImageBuilder::From(SensorImage&& image) {
  image_ = std::move(image);
  return *this;
}

SensorImageBuilder& SensorImageBuilder::SetSensorId(int64_t sensor_id) {
  image_.sensor_id_ = sensor_id;
  return *this;
}

SensorImageBuilder& SensorImageBuilder::SetAcquisitionTime(
    absl::Time acquisition_time) {
  image_.acquisition_time_ = acquisition_time;
  return *this;
}

SensorImageBuilder& SensorImageBuilder::SetCameraParams(
    std::optional<CameraParams> camera_params) {
  image_.camera_params_ = camera_params;
  return *this;
}

SensorImageBuilder& SensorImageBuilder::SetCameraTSensor(
    std::optional<Pose> camera_t_sensor) {
  image_.camera_t_sensor_ = camera_t_sensor;
  return *this;
}

template <class Traits>
SensorImageBuilder& SensorImageBuilder::SetImage(Image<Traits>&& image) {
  image_ = SensorImage(image_, std::move(image));
  return *this;
}

template SensorImageBuilder& SensorImageBuilder::SetImage(Image<Rgb8u>&& image);
template SensorImageBuilder& SensorImageBuilder::SetImage(
    Image<Gray8u>&& image);
template SensorImageBuilder& SensorImageBuilder::SetImage(
    Image<Gray32f>&& image);
template SensorImageBuilder& SensorImageBuilder::SetImage(
    Image<Depth32f>&& image);
template SensorImageBuilder& SensorImageBuilder::SetImage(
    Image<Point32f>&& image);
template SensorImageBuilder& SensorImageBuilder::SetImage(
    Image<Normal32f>&& image);

SensorImage SensorImageBuilder::Build() {
  CHECK(!is_built_) << "SensorImageBuilder can only be built once.";
  is_built_ = true;
  return std::move(image_);
}

}  // namespace perception
}  // namespace intrinsic
