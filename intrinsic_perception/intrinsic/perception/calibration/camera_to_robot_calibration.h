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

#ifndef INTRINSIC_PERCEPTION_CALIBRATION_CAMERA_TO_ROBOT_CALIBRATION_H_
#define INTRINSIC_PERCEPTION_CALIBRATION_CAMERA_TO_ROBOT_CALIBRATION_H_

#include "absl/status/statusor.h"
#include "intrinsic/perception/proto/v1/camera_to_robot_calibration.pb.h"

namespace intrinsic {
namespace perception {

absl::StatusOr<intrinsic_proto::perception::v1::CameraToRobotCalibrationResult>
CalibrateCameraToRobot(
    const intrinsic_proto::perception::v1::CameraToRobotCalibrationRequest&
        request);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CALIBRATION_CAMERA_TO_ROBOT_CALIBRATION_H_
