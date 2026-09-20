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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_VALIDATION_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_VALIDATION_H_

#include <optional>

#include "absl/status/status.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/proto/collision_checker_config.pb.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// Performs collision check on the trajectory.
//
// The resolution of collision checks along the trajectory can be performed by
// specifying collision_check_spacing. If unspecified, this function defaults to
// a value of 0.01 in joint space.
absl::Status CheckCollisions(
    const World& world, RobotCollectionsEntityId robot_id,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    const JointTrajectoryPVA& trajectory,
    const intrinsic_proto::RuleSet& collision_rule_set,
    std::optional<double> collision_check_spacing);

absl::Status CheckCollisions(const JointTrajectoryPVA& trajectory,
                             const KinematicsSystemProxy& proxy,
                             std::optional<double> collision_check_spacing);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_VALIDATION_H_
