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

#include "intrinsic/motion_planning/trajectory_planning/validation.h"

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/motion_planning/motion_planner/motion_planning_error_utils.h"
#include "intrinsic/motion_planning/path_planning/interpolation.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy_util.h"
#include "intrinsic/motion_planning/path_planning/planners/validation.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/annotate.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

absl::Status CheckCollisions(
    const World& world, RobotCollectionsEntityId robot_id,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    const JointTrajectoryPVA& trajectory,
    const intrinsic_proto::RuleSet& collision_rule_set,
    std::optional<double> collision_check_spacing) {
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<KinematicsSystemProxy> proxy,
      CreateKinematicsProxyWithConfig(world, robot_id, collision_checker_config,
                                      /*constraints=*/{}, collision_rule_set),
      _.LogError());
  return CheckCollisions(trajectory, *proxy, collision_check_spacing);
}

absl::Status CheckCollisions(const JointTrajectoryPVA& trajectory,
                             const KinematicsSystemProxy& proxy,
                             std::optional<double> collision_check_spacing) {
  absl::Status front_valid =
      ValidateConfiguration(proxy, trajectory.data().front().position);

  if (!front_valid.ok()) {
    return UpdateStatusErrorContext(
        front_valid,
        intrinsic_proto::motion_planning::v1::ErrorContext::
            TRAJECTORY_PLANNING_INITIAL_STATE_COLLISION,
        absl::StatusCode::kInternal);
  }

  absl::Status back_valid =
      ValidateConfiguration(proxy, trajectory.data().back().position);
  if (!back_valid.ok()) {
    return UpdateStatusErrorContext(
        back_valid,
        intrinsic_proto::motion_planning::v1::ErrorContext::
            TRAJECTORY_PLANNING_GOAL_STATE_COLLISION,
        absl::StatusCode::kInternal);
  }

  // Check collisions along the trajectory.
  //
  // First, formulate the lambda for collision checking - it returns true
  // when there is a collision.
  auto check_fn = [&proxy](const eigenmath::VectorXd& point) {
    return !ValidateConfiguration(proxy, point).ok();
  };

  // Copy the positions to form a path.
  std::vector<eigenmath::VectorXd> path;
  path.reserve(trajectory.size());
  for (const auto& point : trajectory.data()) {
    path.push_back(point.position);
  }

  // Set a default collision check spacing if none was provided.
  if (!collision_check_spacing.has_value()) {
    constexpr double kDefaultCollisionCheckSpacing = 0.01;
    collision_check_spacing = kDefaultCollisionCheckSpacing;
  }

  // Perform the checks.
  INTR_ASSIGN_OR_RETURN(
      auto result,
      InterpolateAndCheck(path, *collision_check_spacing, check_fn));

  if (result.has_value()) {
    // Found a invalid point. Validate it one more time so we can get the
    // error message to print.
    eigenmath::VectorXd invalid_point = std::get<0>(*result);
    absl::Status validate_result = ValidateConfiguration(proxy, invalid_point);
    if (validate_result.ok()) {
      return absl::InternalError(absl::StrFormat(
          "Unexpected error. The point (%s) was flagged by checks, but did "
          "NOT report a validation error when checked individually. This "
          "is a system error and should be reported.",
          toString(invalid_point)));
    }
    auto updated_status = PrependError(
        validate_result, "A point along the planned trajectory is invalid.");
    return UpdateStatusErrorContext(
        updated_status,
        intrinsic_proto::motion_planning::v1::ErrorContext::
            TRAJECTORY_PLANNING_STATE_COLLISION,
        absl::StatusCode::kInternal);
  }

  return absl::OkStatus();
}

}  // namespace intrinsic
