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

#ifndef INTRINSIC_PERCEPTION_CORE_UNDISTORTION_H_
#define INTRINSIC_PERCEPTION_CORE_UNDISTORTION_H_

#include <cstdint>
#include <optional>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"

namespace intrinsic {
namespace perception {

// This class can be used to undistort images. It is implemented as a class
// since it caches the maps required for the undistortion to improve the
// performance.
class Undistortion {
 public:
  Undistortion() = default;
  explicit Undistortion(const CameraParams& camera_params);

  // Returns the internally used camera params, if set.
  const std::optional<CameraParams>& camera_params() const;

  // This function returns undistorted images and makes the class behave like a
  // function. It can be used as
  // <pre>
  //   Undistoration undistort{camera_params};
  //   const Image<Rgb8> undistorted_image = undistort(distorted_image);
  // </pre>
  //
  // This function is currently specialized for Rgb8 only. It is implemented as
  // a template function in order to be able to hide the OpenCV implementation
  // details. If no undistortion is required, this function call will result in
  // a copy of the input image.
  template <typename ImageTrait>
  Image<ImageTrait> operator()(const Image<ImageTrait>& distorted_image) const;

  // Returns undistorted images.
  // If no undistortion is required, this function returns the input image as is
  // and will trigger no unnecessary copy of the input image.
  template <typename ImageTrait>
  Image<ImageTrait> operator()(Image<ImageTrait>&& distorted_image) const;

 private:
  std::optional<CameraParams> camera_params_;
  Image<Generic16i2> undist_map_xy_;
  Image<Generic16u> undist_interp_tables_;
};

// A map from sensor id to undistortion.
using UndistortionBySensorId = absl::flat_hash_map<int64_t, Undistortion>;

// Creates an undistortion map with float coordinates for given camera params.
absl::StatusOr<Image<Generic32f2>> CreateUndistortionMap(
    const CameraParams& camera_params);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_UNDISTORTION_H_
