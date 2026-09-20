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

#ifndef INTRINSIC_PERCEPTION_CORE_CAMERA_PARAMS_H_
#define INTRINSIC_PERCEPTION_CORE_CAMERA_PARAMS_H_

#include <stdbool.h>

#include <cstdint>
#include <optional>
#include <ostream>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/distortion_params.h"
#include "intrinsic/perception/core/intrinsic_params.h"

namespace intrinsic {
namespace perception {

// This class stores all camera parameters.
//
// Camera parameters store
//   * the intrinsic parameters which were determined through a calibration,
//     which implicitly define the image size.
//   * distortion parameters (modeling tangential and spherical distortion)
//
// Note: The camera params explicitly offer no default constructor. Invalid
//   objects can be represented through std::optional or std::unique_ptr. We
//   expressively do not want to allow the creation of invalid objects.
struct CameraParams {
  // Create camera parameters from user provided parameters.
  CameraParams(const IntrinsicParams& intrinsic_params,
               const std::optional<DistortionParams>& distortion_params)
      : intrinsic_params(intrinsic_params),
        distortion_params(distortion_params) {}

  // Creates camera params without distortion params.
  explicit CameraParams(const IntrinsicParams& intrinsic_params)
      : intrinsic_params(intrinsic_params) {}

  // Create camera parameters from dimensions and a focal length value specified
  // in pixels which is horizontally and vertically identical.
  CameraParams(const Dimensions& dimensions, double focal_length)
      : CameraParams(IntrinsicParams(dimensions, focal_length, focal_length,
                                     0.5 * (dimensions.cols - 1),
                                     0.5 * (dimensions.rows - 1))) {}

  const Dimensions& Dimensions() const { return intrinsic_params.dimensions(); }

  IntrinsicParams intrinsic_params;
  std::optional<DistortionParams> distortion_params;
};

// A map from sensor id to camera parameters.
using CameraParamsBySensorId = absl::flat_hash_map<int64_t, CameraParams>;

// Returns camera parameters for a scaled image.
CameraParams Resize(const CameraParams& camera_params, Dimensions size);

// Returns OK status if all values of the intrinsic and distortion parameters
// from both camera parameters are equal up to the user specified epsilon.
absl::Status CameraParamsNear(const CameraParams& a, const CameraParams& b,
                              double eps);

absl::Status CameraParamsNear(const CameraParamsBySensorId& a,
                              const CameraParamsBySensorId& b, double eps);

// Prints camera params to the specified output stream.
// The PrintTo method generates testing specific output and can be distinct from
// a generic streaming operator.
void PrintTo(const CameraParams& camera_params, std::ostream* os);
void PrintTo(const CameraParamsBySensorId& camera_params, std::ostream* os);

std::ostream& operator<<(std::ostream& os, const CameraParams& camera_params);
std::ostream& operator<<(std::ostream& os,
                         const CameraParamsBySensorId& camera_params);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_CAMERA_PARAMS_H_
