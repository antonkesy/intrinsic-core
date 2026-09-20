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

#ifndef INTRINSIC_PERCEPTION_CALIBRATION_STEREO_CALIBRATION_H_
#define INTRINSIC_PERCEPTION_CALIBRATION_STEREO_CALIBRATION_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/eigen_types.h"

namespace intrinsic {
namespace perception {

struct StereoCalibrationResult {
  Pose3d cam1_t_cam0;
  double reprojection_error;
};

absl::StatusOr<StereoCalibrationResult> CalibrateStereoExtrinsic(
    const std::vector<std::vector<Vector3f>>& obj_points,
    const std::vector<std::vector<Vector2f>>& img_points0,
    const std::vector<std::vector<Vector2f>>& img_points1,
    const CameraParams& camera_params0, const CameraParams& camera_params1);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CALIBRATION_STEREO_CALIBRATION_H_
