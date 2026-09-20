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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_CALIBRATION_UTILS_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_CALIBRATION_UTILS_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "include/ur_client_library/primary/robot_state/kinematics_info.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

struct EntityAndTransform {
  std::string entity_name;
  Pose3d parent_t_entity;
  bool is_joint;  // Otherwise link.
};

absl::StatusOr<std::vector<EntityAndTransform>> CalculateChain(
    const ::urcl::primary_interface::KinematicsInfo& kin_info,
    absl::string_view robot_model);

absl::StatusOr<std::string> GetCalibratedRobotIkSolverKey(
    absl::string_view robot_model);

std::string GetCalibratedRobotIkSolverTip();

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_CALIBRATION_UTILS_H_
