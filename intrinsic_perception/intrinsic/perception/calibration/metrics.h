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

#ifndef INTRINSIC_PERCEPTION_CALIBRATION_METRICS_H_
#define INTRINSIC_PERCEPTION_CALIBRATION_METRICS_H_

#include "absl/status/statusor.h"
#include "intrinsic/math/proto/pose.pb.h"
#include "intrinsic/perception/proto/v1/camera_to_robot_calibration.pb.h"

namespace intrinsic::perception {
absl::StatusOr<intrinsic_proto::perception::v1::CameraToRobotCalibrationResult>
ComputeCameraToRobotCalibrationErrorMetrics(
    const intrinsic_proto::perception::v1::CameraToRobotCalibrationRequest&
        request,
    const intrinsic_proto::perception::v1::CameraToRobotCalibrationResult&
        result);

absl::StatusOr<intrinsic_proto::perception::v1::CameraToRobotValidationResult>
ComputeCameraToRobotValidationMetrics(
    const intrinsic_proto::perception::v1::CameraToRobotCalibrationRequest&
        request,
    const intrinsic_proto::Pose& pose);

}  // namespace intrinsic::perception
#endif  // INTRINSIC_PERCEPTION_CALIBRATION_METRICS_H_
