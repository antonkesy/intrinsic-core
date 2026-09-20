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

#include "intrinsic/perception/core/camera_params.h"

#include <ostream>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/distortion_params.h"
#include "intrinsic/perception/core/intrinsic_params.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {

CameraParams Resize(const CameraParams& camera_params, Dimensions size) {
  IntrinsicParams resized_intrinsic_params =
      Resize(camera_params.intrinsic_params, size);
  return CameraParams(resized_intrinsic_params,
                      camera_params.distortion_params);
}

// Returns OK status if all values of the intrinsic and distortion parameters
// from both camera parameters are equal up to the user specified epsilon.
absl::Status CameraParamsNear(const CameraParams& a, const CameraParams& b,
                              double eps) {
  INTR_RETURN_IF_ERROR(
      IntrinsicParamsNear(a.intrinsic_params, b.intrinsic_params, eps));
  const DistortionParams a_distortion_params =
      a.distortion_params.value_or(DistortionParams());
  const DistortionParams b_distortion_params =
      b.distortion_params.value_or(DistortionParams());
  INTR_RETURN_IF_ERROR(
      DistortionParamsNear(a_distortion_params, b_distortion_params, eps));
  return absl::OkStatus();
}

absl::Status CameraParamsNear(const CameraParamsBySensorId& a,
                              const CameraParamsBySensorId& b, double eps) {
  if (a.size() != b.size()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Camera parameters have a different number of sensors.";
  }
  for (const auto& [a_id, a_params] : a) {
    const auto b_it = b.find(a_id);
    if (b_it == b.end()) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "Camera parameters are from different sensors. Id " << a_id
             << " is not present in the second map of camera parameters.";
    }
    INTR_RETURN_IF_ERROR(CameraParamsNear(a_params, b_it->second, eps));
  }
  return absl::OkStatus();
}

// Prints camera params to the specified output stream.
// The PrintTo method generates testing specific output and can be distinct from
// a generic streaming operator.
void PrintTo(const CameraParams& camera_params, std::ostream* os) {
  CHECK_NE(os, nullptr);
  PrintTo(camera_params.intrinsic_params, os);
  *os << ", ";
  if (camera_params.distortion_params.has_value()) {
    PrintTo(*camera_params.distortion_params, os);
  } else {
    *os << "distortion not set";
  }
}

void PrintTo(const CameraParamsBySensorId& camera_params, std::ostream* os) {
  CHECK_NE(os, nullptr);
  *os << "{";
  for (const auto& [sensor_id, camera_params] : camera_params) {
    *os << "sensor_id: " << sensor_id << ", ";
    PrintTo(camera_params, os);
  }
  *os << "}";
}

std::ostream& operator<<(std::ostream& os, const CameraParams& camera_params) {
  PrintTo(camera_params, &os);
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const CameraParamsBySensorId& camera_params) {
  PrintTo(camera_params, &os);
  return os;
}

}  // namespace perception
}  // namespace intrinsic
