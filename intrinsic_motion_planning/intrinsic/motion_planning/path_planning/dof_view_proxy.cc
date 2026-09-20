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

#include "intrinsic/motion_planning/path_planning/dof_view_proxy.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_join.h"
#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/proto/motion_specification_proto_utils.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/collision/collision_types.h"
#include "intrinsic/world/collision/util/collision_world_util.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/proto/entity_search.pb.h"
#include "intrinsic/world/util/kinematic_world_util.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> DofViewProxy::Create(
    const World& world, const std::vector<JointEntityId>& joint_entity_ids,
    std::vector<std::unique_ptr<ConstraintInterface>> constraints,
    const intrinsic_proto::RuleSet& rule_set,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    bool disable_collision_checking) {
  auto world_copy = std::make_unique<World>(world.Clone());

  INTR_ASSIGN_OR_RETURN(auto dof_view,
                        world_copy->GetDofKinematicView(joint_entity_ids));
  std::shared_ptr<CollisionChecker> collision_checker = nullptr;
  if (!disable_collision_checking) {
    INTR_ASSIGN_OR_RETURN(
        collision_checker,
        GetCollisionCheckerForDofView(world_copy.get(), *dof_view, rule_set,
                                      collision_checker_config));
    INTR_RET_CHECK(collision_checker != nullptr);
  }

  return absl::WrapUnique(
      new DofViewProxy(std::move(world_copy), std::move(dof_view),
                       std::move(collision_checker), std::move(constraints)));
}

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> DofViewProxy::Create(
    const World& world, RobotCollectionsEntityId robot_id,
    std::vector<std::unique_ptr<ConstraintInterface>> constraints,
    const intrinsic_proto::RuleSet& rule_set,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    bool disable_collision_checking) {
  INTR_ASSIGN_OR_RETURN(auto joint_entity_ids, world.GetRobotDofs(robot_id));
  return DofViewProxy::Create(world, joint_entity_ids, std::move(constraints),
                              rule_set, collision_checker_config,
                              disable_collision_checking);
}

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> DofViewProxy::Create(
    const object_world::ObjectWorld& object_world,
    RobotCollectionsEntityId robot_id,
    std::optional<
        intrinsic_proto::motion_planning::v1::UniformGeometricConstraint>
        constraints_proto,
    const intrinsic_proto::RuleSet& rule_set,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    bool disable_collision_checking) {
  std::vector<std::unique_ptr<ConstraintInterface>> constraints;
  if (constraints_proto.has_value()) {
    INTR_ASSIGN_OR_RETURN(constraints, GetUniformGeometricConstraintsFromProto(
                                           object_world, *constraints_proto));
  }
  return Create(object_world.GetEntityWorld(), robot_id, std::move(constraints),
                rule_set, collision_checker_config, disable_collision_checking);
}

DofViewProxy::DofViewProxy(
    std::unique_ptr<World> world, std::unique_ptr<DofKinematicView> dof_view,
    std::shared_ptr<CollisionChecker> collision_checker,
    std::vector<std::unique_ptr<ConstraintInterface>> constraints)
    : world_(std::move(world)),
      dof_view_(std::move(dof_view)),
      collision_checker_(std::move(collision_checker)),
      constraints_(std::move(constraints)) {}

absl::StatusOr<bool> DofViewProxy::AreConstraintsSatisfied(
    const eigenmath::VectorXd& configuration) const {
  for (const auto& constraint : constraints_) {
    INTR_ASSIGN_OR_RETURN(const bool satisfied,
                          constraint->IsSatisfied(configuration));
    if (!satisfied) {
      return false;
    }
  }
  return true;
}

absl::StatusOr<eigenmath::VectorXd>
DofViewProxy::ProjectOntoConstraintManifolds(
    const eigenmath::VectorXd& configuration, size_t max_projection_steps,
    double max_constraint_error_norm) const {
  return absl::UnimplementedError(
      "DofViewProxy::ProjectOntoConstraintManifolds() has not been "
      "implemented!");
}

absl::StatusOr<JointConfigurationValidationResult> DofViewProxy::IsValid(
    const eigenmath::VectorXd& configuration,
    CollisionCheckingDebug* collision_debug) const {
  if (seen_points_.contains(configuration)) {
    auto& data = seen_points_.at(configuration);
    if (collision_debug != nullptr) {
      collision_debug->collisions.insert(collision_debug->collisions.end(),
                                         data.collisions.begin(),
                                         data.collisions.end());
    }
    return data.result;
  }

  JointConfigurationValidationResult validation_status =
      JointConfigurationValidationResult{};
  // We first need to check if the configuration is valid in terms of joint
  // limits, otherwise setting the robot into the configuration and enforcing
  // the limits will cause an error.
  INTR_ASSIGN_OR_RETURN(const bool within_limits,
                        IsWithinDofLimits(*dof_view_, configuration));
  validation_status.within_limits_status =
      within_limits ? WithinLimitsStatus::kWithinLimits
                    : WithinLimitsStatus::kViolatedLimits;
  if (validation_status.within_limits_status ==
      WithinLimitsStatus::kViolatedLimits) {
    // Returns early (skipping the rest of the checks) to minimize run time.
    seen_points_[configuration] = {.result = validation_status};
    return validation_status;
  }
  INTR_ASSIGN_OR_RETURN(const bool constraints_satisfied,
                        AreConstraintsSatisfied(configuration));
  validation_status.constraint_satisfaction_status =
      constraints_satisfied
          ? ConstraintSatisfactionStatus::kAllConstraintsSatisfied
          : ConstraintSatisfactionStatus::kViolatedConstraints;
  if (validation_status.constraint_satisfaction_status ==
      ConstraintSatisfactionStatus::kViolatedConstraints) {
    // Returns early (skipping the rest of the checks) to minimize run time.
    seen_points_[configuration] = {.result = validation_status};
    return validation_status;
  }

  CollisionCheckingDebug inner_debug;
  if (collision_checker_ == nullptr) {
    validation_status.collision_status = MarginPairConflictStatus::kClear;
  } else {
    const eigenmath::VectorXd current_configuration = dof_view_->GetDofValues();
    INTR_RETURN_IF_ERROR(
        dof_view_->SetDofValues(configuration, /*enforce_limits=*/true));
    validation_status.collision_status =
        collision_checker_->IsInCollision(&inner_debug);
    INTR_RETURN_IF_ERROR(dof_view_->SetDofValues(current_configuration,
                                                 /*enforce_limits=*/true));
  }

  // We always run with inner_debug because we have to cache the results for
  // future calls even if the current call did not have the debug pointer.
  seen_points_[configuration] = {.result = validation_status,
                                 .collisions = inner_debug.collisions};

  if (collision_debug != nullptr) {
    collision_debug->collisions.insert(collision_debug->collisions.end(),
                                       inner_debug.collisions.begin(),
                                       inner_debug.collisions.end());
  }
  return validation_status;
}

absl::StatusOr<bool> DofViewProxy::IsWithinLimits(
    const eigenmath::VectorXd& configuration) const {
  return IsWithinDofLimits(*dof_view_, configuration);
}

std::vector<DofLabel> DofViewProxy::GetDofLabels() const {
  return dof_view_->GetDofLabels();
}

std::pair<PhysicalEntityId, PhysicalEntityId> DofViewProxy::GetBaseAndTipIds()
    const {
  // TODO(b/242905694): These are invalid ids returned to allow a
  // PathPlannerGraph to be constructed. The graph class should be changed to
  // used a more modern identifier that works whether we have a dof view or a
  // cartesian view.
  return std::make_pair(PhysicalEntityId(kInvalidEntityId),
                        PhysicalEntityId(kInvalidEntityId));
}

absl::StatusOr<Pose3d> DofViewProxy::GetFk(
    const eigenmath::VectorXd& configuration,
    const LabelId& named_location) const {
  return absl::UnimplementedError("Not implemented: DofViewProxy::GetFk");
}

absl::StatusOr<Pose3d> DofViewProxy::GetFkInWorld(
    const eigenmath::VectorXd& configuration,
    const LabelId& named_location) const {
  return absl::UnimplementedError(
      "Not implemented: DofViewProxy::GetFkInWorld");
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> DofViewProxy::GetIk(
    const Pose3d& pose) const {
  return absl::UnimplementedError("Not implemented: DofViewProxy::GetIk");
}

JointLimitsXd DofViewProxy::GetJointLimits() const {
  return dof_view_->GetDofApplicationLimits();
}

std::shared_ptr<CollisionChecker> DofViewProxy::GetCollisionChecker() const {
  return collision_checker_;
}

absl::Status DofViewProxy::SetJointLimits(const JointLimitsXd& joint_limits) {
  // Check if current configuration is within the new set of limits. If not, we
  // have to reset the dof values to something within the new limits. Otherwise,
  // the SetDofApplicationLimits causes an issue.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const auto limit_check_result,
      ::intrinsic::IsWithinLimits(dof_view_->GetDofValues(), joint_limits));
  if (!limit_check_result.p_ok) {
    const eigenmath::VectorXd new_position_value =
        (joint_limits.min_position + joint_limits.max_position) * 0.5;
    INTR_RETURN_IF_ERROR(dof_view_->SetDofValues(new_position_value,
                                                 /*enforce_limits=*/false));
  }
  return dof_view_->SetDofApplicationLimits(joint_limits,
                                            /*enforce_limits=*/true);
}

absl::StatusOr<eigenmath::VectorXd> DofViewProxy::GetRandomConfig(
    int* seed) const {
  auto [lower_limits, upper_limits] = dof_view_->GetDofValueApplicationLimits();

  if (lower_limits.size() != upper_limits.size()) {
    return intrinsic::InternalErrorBuilder()
           << "Lower and upper system limits are of different size.";
  }
  for (int i = 0; i < lower_limits.size(); ++i) {
    // Workaround for infinite joints if not explicitly specified by user.
    if (!std::isfinite(lower_limits[i])) {
      lower_limits[i] = -2 * M_PI;
    }
    if (!std::isfinite(upper_limits[i])) {
      upper_limits[i] = 2 * M_PI;
    }
  }

  return GetRandomConfig(seed, lower_limits, upper_limits);
}

absl::StatusOr<eigenmath::VectorXd> DofViewProxy::GetRandomConfig(
    int* seed, const eigenmath::VectorXd& lower_limits,
    const eigenmath::VectorXd& upper_limits) const {
  CHECK(seed != nullptr);
  // It's possible for us to wrap around while incrementing the seed. In this
  // case the random generator may become degenerate. This check ensures we
  // avoid this case.
  if (*seed < 0) {
    *seed = 0;
  }

  if (lower_limits.size() != upper_limits.size()) {
    return intrinsic::InternalErrorBuilder()
           << "Lower and upper system limits are of different size.";
  }
  for (int i = 0; i < lower_limits.size(); ++i) {
    // Do not accept infinite joints.
    if (!std::isfinite(lower_limits[i])) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "Joint " << i
             << " has infinite lower limit. We currently do not support "
                "sampling in infinite limits.";
    }
    if (!std::isfinite(upper_limits[i])) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "Joint " << i
             << " has infinite upper limit. We currently do not support "
                "sampling in infinite limits.";
    }
  }
  while (true) {
    eigenmath::VectorXd config =
        GetQuasiRandomDofConfiguration(*dof_view_, seed);
    INTR_ASSIGN_OR_RETURN(const bool within_limits,
                          IsWithinDofLimits(*dof_view_, config));
    if (within_limits) {
      return config;
    }
  }
}

absl::StatusOr<eigenmath::VectorXd> DofViewProxy::GetRandomConfigNear(
    const eigenmath::VectorXd& config, double distance, int* seed) const {
  return GetRandomConfigNear(
      config, eigenmath::VectorXd::Constant(config.size(), distance), seed);
}

absl::StatusOr<eigenmath::VectorXd> DofViewProxy::GetRandomConfigNear(
    const eigenmath::VectorXd& configuration,
    const eigenmath::VectorXd& distance, int* seed) const {
  CHECK(seed != nullptr);
  // It's possible for us to wrap around while incrementing the seed. In this
  // case the random generator may become degenerate. This check ensures we
  // avoid this case.
  if (*seed < 0) {
    *seed = 0;
  }

  // Set the limits according to the range and joint limits.
  auto [lower_limits, upper_limits] = dof_view_->GetDofValueApplicationLimits();
  CHECK_EQ(lower_limits.size(), upper_limits.size());
  for (int i = 0; i < lower_limits.size(); ++i) {
    lower_limits[i] = std::max(lower_limits[i], configuration[i] - distance[i]);
    upper_limits[i] = std::min(upper_limits[i], configuration[i] + distance[i]);
  }

  while (true) {
    INTR_ASSIGN_OR_RETURN(
        eigenmath::VectorXd random_config,
        eigenmath::GetQuasiRandomVectorXd(lower_limits, upper_limits, seed));
    INTR_ASSIGN_OR_RETURN(const bool within_limits,
                          IsWithinDofLimits(*dof_view_, random_config));
    if (within_limits) {
      return random_config;
    }
  }
}

absl::StatusOr<std::string> DofViewProxy::PrintCollisionCheckingDebug(
    const CollisionCheckingDebug& collision_debug) const {
  return collision_checker_->PrintCollisionCheckingDebug(collision_debug);
}

absl::StatusOr<intrinsic_proto::motion_planning::v1::CollisionDebug>
DofViewProxy::GetCollisionDebugMessage(
    const CollisionCheckingDebug& collision_debug) const {
  if (collision_debug.collisions.empty()) {
    return absl::InvalidArgumentError("No collisions found.");
  }
  // We only report the first collision.
  INTR_ASSIGN_OR_RETURN(std::tuple entities_to_report,
                        collision_checker_->GetCollisionEntitiesMessage(
                            collision_debug.collisions[0]));
  intrinsic_proto::motion_planning::v1::CollisionDebug collision_debug_message;
  *collision_debug_message.mutable_left_entity() =
      std::get<0>(entities_to_report);
  *collision_debug_message.mutable_right_entities() =
      absl::StrJoin(std::get<1>(entities_to_report), "\n");
  std::tuple entities =
      collision_checker_->GetCollisionEntities(collision_debug.collisions[0]);
  collision_debug_message.set_left_entity_id(std::get<0>(entities).value());
  for (const auto& right_entity_id : std::get<1>(entities)) {
    collision_debug_message.add_right_entity_ids(right_entity_id.value());
  }
  return collision_debug_message;
}

absl::StatusOr<const std::vector<std::unique_ptr<ConstraintInterface>>*>
DofViewProxy::GetConstraints() const {
  return &constraints_;
}

}  // namespace intrinsic
