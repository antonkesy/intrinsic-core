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

#include "intrinsic/motion_planning/motion_planner/motion_planning_limits_utils.h"

#include <algorithm>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.pb.h"
#include "intrinsic/motion_planning/proto/motion_specification_proto_utils.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_limits.pb.h"

namespace intrinsic {

absl::StatusOr<intrinsic_proto::motion_planning::v1::DynamicCartesianLimits>
GetMostRelaxedDynamicCartesianLimitsCombination(
    const intrinsic_proto::motion_planning::v1::DynamicCartesianLimits&
        dynamic_cartesian_limits_1,
    const intrinsic_proto::motion_planning::v1::DynamicCartesianLimits&
        dynamic_cartesian_limits_2) {
  intrinsic_proto::motion_planning::v1::DynamicCartesianLimits
      most_relaxed_dynamic_cartesian_limits;
  if (dynamic_cartesian_limits_1.has_max_translational_velocity() &&
      dynamic_cartesian_limits_2.has_max_translational_velocity()) {
    most_relaxed_dynamic_cartesian_limits.set_max_translational_velocity(
        std::max(dynamic_cartesian_limits_1.max_translational_velocity(),
                 dynamic_cartesian_limits_2.max_translational_velocity()));
  }
  if (dynamic_cartesian_limits_1.has_max_translational_acceleration() &&
      dynamic_cartesian_limits_2.has_max_translational_acceleration()) {
    most_relaxed_dynamic_cartesian_limits.set_max_translational_acceleration(
        std::max(dynamic_cartesian_limits_1.max_translational_acceleration(),
                 dynamic_cartesian_limits_2.max_translational_acceleration()));
  }
  if (dynamic_cartesian_limits_1.has_max_rotational_velocity() &&
      dynamic_cartesian_limits_2.has_max_rotational_velocity()) {
    most_relaxed_dynamic_cartesian_limits.set_max_rotational_velocity(
        std::max(dynamic_cartesian_limits_1.max_rotational_velocity(),
                 dynamic_cartesian_limits_2.max_rotational_velocity()));
  }
  if (dynamic_cartesian_limits_1.has_max_rotational_acceleration() &&
      dynamic_cartesian_limits_2.has_max_rotational_acceleration()) {
    most_relaxed_dynamic_cartesian_limits.set_max_rotational_acceleration(
        std::max(dynamic_cartesian_limits_1.max_rotational_acceleration(),
                 dynamic_cartesian_limits_2.max_rotational_acceleration()));
  }
  return most_relaxed_dynamic_cartesian_limits;
}

absl::StatusOr<intrinsic_proto::motion_planning::v1::JointLimitsUpdate>
GetMostRelaxedJointLimitsUpdateCombination(
    const intrinsic_proto::motion_planning::v1::JointLimitsUpdate&
        joint_limits_update_1,
    const intrinsic_proto::motion_planning::v1::JointLimitsUpdate&
        joint_limits_update_2) {
  const absl::StatusOr<JointLimits> joint_limits_1 =
      ToJointLimits(joint_limits_update_1);
  const absl::StatusOr<JointLimits> joint_limits_2 =
      ToJointLimits(joint_limits_update_2);
  if (!joint_limits_1.ok() &&
      absl::StrContains(joint_limits_1.status().message(),
                        "No field is set in the JointLimitsUpdate proto:")) {
    // `joint_limits_update_1` is already the most relaxed `JointLimitsUpdate`,
    // since no field is set in the proto, meaning the limits are all unlimited.
    return joint_limits_update_1;
  }
  if (!joint_limits_2.ok() &&
      absl::StrContains(joint_limits_2.status().message(),
                        "No field is set in the JointLimitsUpdate proto:")) {
    // `joint_limits_update_2` is already the most relaxed `JointLimitsUpdate`,
    // since no field is set in the proto, meaning the limits are all unlimited.
    return joint_limits_update_2;
  }
  if (joint_limits_1->size() != joint_limits_2->size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("The input JointLimitsUpdate's have different sizes: ",
                     joint_limits_1->size(), " vs ", joint_limits_2->size()));
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(JointLimits most_relaxed_joint_limits,
                                JointLimits::Unlimited(joint_limits_1->size()));
  most_relaxed_joint_limits.min_position =
      joint_limits_1->min_position.cwiseMin(joint_limits_2->min_position);
  most_relaxed_joint_limits.max_position =
      joint_limits_1->max_position.cwiseMax(joint_limits_2->max_position);
  most_relaxed_joint_limits.max_velocity =
      joint_limits_1->max_velocity.cwiseMax(joint_limits_2->max_velocity);
  most_relaxed_joint_limits.max_acceleration =
      joint_limits_1->max_acceleration.cwiseMax(
          joint_limits_2->max_acceleration);
  most_relaxed_joint_limits.max_jerk =
      joint_limits_1->max_jerk.cwiseMax(joint_limits_2->max_jerk);
  return ToJointLimitsUpdateProto(most_relaxed_joint_limits);
}

}  // namespace intrinsic
