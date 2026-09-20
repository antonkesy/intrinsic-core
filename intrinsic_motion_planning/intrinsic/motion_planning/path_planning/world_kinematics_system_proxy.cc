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

#include "intrinsic/motion_planning/path_planning/world_kinematics_system_proxy.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/pseudo_inverse.h"
#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/new_manipulator_kinematics.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_factory.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/proto/motion_specification_proto_utils.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/collision/collision_types.h"
#include "intrinsic/world/collision/util/collision_world_util.h"
#include "intrinsic/world/configuration_validation.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/util/kinematic_world_util.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

absl::StatusOr<std::string> SelectIkSolverKeyForChain(
    const kinematics::Chain& chain) {
  const auto base_tip_solver_keys =
      chain.GetAllSolverKeys(chain.GetBaseId(), chain.GetTipId());
  if (base_tip_solver_keys.empty()) {
    LOG(WARNING) << "No Ik solver key is defined for the kinematic chain: "
                 << chain.GetName()
                 << ". Using default iterative solver (kinematic_chain).";
    // TODO(b/254331614): Use static variable for default solver instead.
    return "kinematic_chain";
  }
  return base_tip_solver_keys.front();
}

absl::StatusOr<std::unique_ptr<WorldKinematicsSystemProxy>>
WorldKinematicsSystemProxy::Create(
    const World& world, PhysicalEntityId base_object_id,
    PhysicalEntityId tip_object_id,
    const ConfigurationValidatorOptions& dof_validator_options,
    std::vector<std::unique_ptr<ConstraintInterface>> constraints,
    const intrinsic_proto::RuleSet& rule_set,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    bool disable_collision_checking) {
  auto world_copy = std::make_unique<World>(world.Clone());
  INTR_ASSIGN_OR_RETURN(auto dof_view, world_copy->GetDofKinematicView(
                                           base_object_id, tip_object_id));

  INTR_ASSIGN_OR_RETURN(auto cart_view, world_copy->GetCartesianKinematicView(
                                            base_object_id, tip_object_id));

  std::shared_ptr<CollisionChecker> collision_checker = nullptr;
  if (!disable_collision_checking) {
    INTR_ASSIGN_OR_RETURN(
        collision_checker,
        GetCollisionCheckerForDofView(world_copy.get(), *dof_view, rule_set,
                                      collision_checker_config));
    INTR_RET_CHECK(collision_checker != nullptr);
  }

  return absl::WrapUnique(new WorldKinematicsSystemProxy(
      std::move(world_copy), base_object_id, tip_object_id,
      dof_validator_options, std::move(dof_view), std::move(cart_view),
      std::move(collision_checker), std::move(constraints)));
}

WorldKinematicsSystemProxy::WorldKinematicsSystemProxy(
    std::unique_ptr<World> world, PhysicalEntityId base_object_id,
    PhysicalEntityId tip_object_id,
    const ConfigurationValidatorOptions& dof_validator_options,
    std::unique_ptr<DofKinematicView> dov_view,
    std::unique_ptr<CartesianKinematicView> cart_view,
    std::shared_ptr<CollisionChecker> collision_checker,
    std::vector<std::unique_ptr<ConstraintInterface>> constraints)
    : world_(std::move(world)),
      base_object_id_(base_object_id),
      tip_object_id_(tip_object_id),
      dof_view_(std::move(dov_view)),
      cart_view_(std::move(cart_view)),
      collision_checker_(std::move(collision_checker)),
      constraints_(std::move(constraints)) {
  dof_validator_fn_ =
      dof_view_->GetConfigurationLimitsValidator(dof_validator_options);
}

absl::StatusOr<bool> WorldKinematicsSystemProxy::AreConstraintsSatisfied(
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
WorldKinematicsSystemProxy::ProjectOntoConstraintManifolds(
    const eigenmath::VectorXd& configuration, size_t max_projection_steps,
    double max_constraint_error_norm) const {
  if (max_projection_steps <= 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("max_projection_steps must be greater than 0, but got ",
                     max_projection_steps, " instead."));
  }
  if (max_constraint_error_norm <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "max_constraint_error_norm must be greater than 0, but got ",
        max_constraint_error_norm, " instead."));
  }

  eigenmath::VectorXd projected_configuration = configuration;
  INTR_ASSIGN_OR_RETURN(bool constraints_satisfied,
                        AreConstraintsSatisfied(projected_configuration));
  // We only do projection onto constraint manifolds if there is any constraint
  // violation.
  if (!constraints_satisfied) {
    size_t num_total_constraint_dimension = 0;
    const size_t num_dofs = configuration.size();
    for (const auto& constraint : constraints_) {
      if (num_dofs != constraint->DomainDimension()) {
        return absl::InternalError("Constraint Domain Dimension mis-matched!");
      }
      num_total_constraint_dimension += constraint->ConstraintDimension();
    }

    if (num_total_constraint_dimension > eigenmath::MAX_EIGEN_MATRIX_SIZE) {
      return absl::FailedPreconditionError(
          absl::StrCat("Total number of constraint dimension is greater than "
                       "the allowed size eigenmath::MAX_EIGEN_MATRIX_SIZE = ",
                       eigenmath::MAX_EIGEN_MATRIX_SIZE,
                       ". Please increase eigenmath::MAX_EIGEN_MATRIX_SIZE."));
    }

    size_t num_projection_steps = 0;
    const JointLimitsXd joint_limits = GetJointLimits();
    // Iteratively perform the projection:
    while (!constraints_satisfied &&
           (num_projection_steps < max_projection_steps)) {
      eigenmath::MatrixNMd aggregated_constraint_jacobian =
          eigenmath::MatrixNMd::Zero(num_total_constraint_dimension, num_dofs);
      eigenmath::VectorXd aggregated_constraint_evaluation =
          eigenmath::VectorXd::Zero(num_total_constraint_dimension);
      size_t constraint_count = 0;
      for (const auto& constraint : constraints_) {
        INTR_ASSIGN_OR_RETURN(eigenmath::MatrixXd constraint_jacobian,
                              constraint->Gradient(projected_configuration));
        INTR_ASSIGN_OR_RETURN(eigenmath::VectorXd constraint_evaluation,
                              constraint->Evaluate(projected_configuration));
        aggregated_constraint_jacobian.middleRows(
            constraint_count, constraint->ConstraintDimension()) =
            constraint_jacobian;
        aggregated_constraint_evaluation.middleRows(
            constraint_count, constraint->ConstraintDimension()) =
            constraint_evaluation;
        constraint_count += constraint->ConstraintDimension();
      }
      if (constraint_count != num_total_constraint_dimension) {
        return absl::InternalError(absl::StrCat(
            "Number of total constraint dimension (",
            num_total_constraint_dimension, ") is mismatched with the count (",
            constraint_count, ")!"));
      }

      if (aggregated_constraint_evaluation.norm() > max_constraint_error_norm) {
        // Re-scale the constraint evaluation vector, so that it is
        // "sufficiently small" (as governed by `max_constraint_error_norm`), to
        // take into account the locality assumption of a Jacobian matrix:
        aggregated_constraint_evaluation *=
            max_constraint_error_norm / aggregated_constraint_evaluation.norm();
      }

      // Perform pseudo-inverse on the aggregated constraint Jacobian matrix.
      // Throws an error when `aggregated_constraint_jacobian` is
      // rank-deficient, as checked against the `condition_number_threshold`.
      // The reasoning of this behavior is that this iterative projection
      // function may not converge when the `aggregated_constraint_jacobian` is
      // singular.
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const eigenmath::MatrixNMd inverse_aggregated_constraint_jacobian,
          icon::ComputePseudoInverse(aggregated_constraint_jacobian,
                                     /*lambda=*/1e-20,
                                     /*fail_if_badly_conditioned=*/true,
                                     /*condition_number_threshold=*/1.0e+12));

      projected_configuration -= inverse_aggregated_constraint_jacobian *
                                 aggregated_constraint_evaluation;

      if (!eigenmath::ClampVector(joint_limits.min_position,
                                  joint_limits.max_position,
                                  projected_configuration)) {
        return absl::InternalError(
            "Clamping projected_configuration to joint position limits "
            "failed.");
      }

      INTR_ASSIGN_OR_RETURN(constraints_satisfied,
                            AreConstraintsSatisfied(projected_configuration));
      num_projection_steps++;
    }

    if (num_projection_steps >= max_projection_steps) {
      return absl::InternalError(
          "Projection does not converge to a configuration "
          "on-constraint-manifold within the allowable number of projection "
          "steps.");
    }
  }

  return projected_configuration;
}

absl::StatusOr<JointConfigurationValidationResult>
WorldKinematicsSystemProxy::IsValid(
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
    INTR_RETURN_IF_ERROR(
        dof_view_->SetDofValues(configuration, /*enforce_limits=*/true));
    validation_status.collision_status =
        collision_checker_->IsInCollision(&inner_debug);
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

absl::StatusOr<bool> WorldKinematicsSystemProxy::IsWithinLimits(
    const eigenmath::VectorXd& configuration) const {
  INTR_ASSIGN_OR_RETURN(const bool within_limits,
                        IsWithinDofLimits(*dof_view_, configuration));
  return within_limits && dof_validator_fn_(configuration);
}

std::vector<DofLabel> WorldKinematicsSystemProxy::GetDofLabels() const {
  return dof_view_->GetDofLabels();
}

std::pair<PhysicalEntityId, PhysicalEntityId>
WorldKinematicsSystemProxy::GetBaseAndTipIds() const {
  return std::make_pair(base_object_id_, tip_object_id_);
}

absl::StatusOr<Pose3d> WorldKinematicsSystemProxy::GetFk(
    const eigenmath::VectorXd& configuration,
    const LabelId& named_location) const {
  std::optional<AttachmentEntityId> tag_object_id;

  for (const auto& object_id :
       world_->ValidateByLabel<AttachmentEntityId>(named_location)) {
    bool match = tip_object_id_ == object_id;
    if (!match) {
      auto common_ancestor_or =
          world_->FindCommonAncestor(tip_object_id_, object_id);
      match = common_ancestor_or.ok() &&
              common_ancestor_or.value() == tip_object_id_;
    }
    if (match) {
      CHECK(!tag_object_id.has_value());
      tag_object_id = object_id;
    }
  }

  CHECK(tag_object_id.has_value());

  INTR_RETURN_IF_ERROR(
      dof_view_->SetDofValues(configuration, /*enforce_limits=*/true));
  return world_->GetTransform(base_object_id_, *tag_object_id);
}

absl::StatusOr<Pose3d> WorldKinematicsSystemProxy::GetFkInWorld(
    const eigenmath::VectorXd& configuration,
    const LabelId& named_location) const {
  const auto workspace_t_base =
      world_->GetTransform(kRootEntityId, base_object_id_);
  INTR_ASSIGN_OR_RETURN(Pose3d base_t_tag,
                        GetFk(configuration, named_location));
  return workspace_t_base * base_t_tag;
}

absl::StatusOr<std::vector<eigenmath::VectorXd>>
WorldKinematicsSystemProxy::GetIk(const Pose3d& pose) const {
  // TODO(stoyang): This constant should not live here b/136107374
  static int kMaxNumSolutions = 8;
  auto ik_solutions = cart_view_->GetIkSolutions(pose);
  return ik_solutions->GetSampledSolutions(kMaxNumSolutions, std::nullopt);
}

absl::StatusOr<std::vector<eigenmath::VectorXd>>
WorldKinematicsSystemProxy::GetIk(const Pose3d& pose,
                                  const eigenmath::VectorXd& seed) const {
  // TODO(stoyang): This constant should not live here b/136107374
  static int kMaxNumSolutions = 8;
  auto ik_solutions = cart_view_->GetIkSolutions(pose, seed);
  return ik_solutions->GetSampledSolutions(kMaxNumSolutions, std::nullopt);
}

JointLimitsXd WorldKinematicsSystemProxy::GetJointLimits() const {
  return dof_view_->GetDofApplicationLimits();
}

std::shared_ptr<CollisionChecker>
WorldKinematicsSystemProxy::GetCollisionChecker() const {
  return collision_checker_;
}

absl::Status WorldKinematicsSystemProxy::SetJointLimits(
    const JointLimitsXd& joint_limits) {
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

absl::StatusOr<eigenmath::VectorXd> WorldKinematicsSystemProxy::GetRandomConfig(
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

absl::StatusOr<eigenmath::VectorXd> WorldKinematicsSystemProxy::GetRandomConfig(
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
    ASSIGN_OR_DIE(
        eigenmath::VectorXd config,
        eigenmath::GetQuasiRandomVectorXd(lower_limits, upper_limits, seed));
    INTR_ASSIGN_OR_RETURN(const bool within_limits,
                          IsWithinDofLimits(*dof_view_, config));
    if (within_limits && dof_validator_fn_(config)) {
      return config;
    }
  }
}

absl::StatusOr<eigenmath::VectorXd>
WorldKinematicsSystemProxy::GetRandomConfigNear(
    const eigenmath::VectorXd& config, double distance, int* seed) const {
  return GetRandomConfigNear(
      config, eigenmath::VectorXd::Constant(config.size(), distance), seed);
}

absl::StatusOr<eigenmath::VectorXd>
WorldKinematicsSystemProxy::GetRandomConfigNear(
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
    if (within_limits && dof_validator_fn_(random_config)) {
      return random_config;
    }
  }
}

absl::StatusOr<std::string>
WorldKinematicsSystemProxy::PrintCollisionCheckingDebug(
    const CollisionCheckingDebug& collision_debug) const {
  return collision_checker_->PrintCollisionCheckingDebug(collision_debug);
}

absl::StatusOr<intrinsic_proto::motion_planning::v1::CollisionDebug>
WorldKinematicsSystemProxy::GetCollisionDebugMessage(
    const CollisionCheckingDebug& collision_debug) const {
  if (collision_debug.collisions.empty()) {
    return absl::InvalidArgumentError("No collisions found.");
  }
  // We only report the first collision.
  INTR_ASSIGN_OR_RETURN(std::tuple entities_to_report,
                        collision_checker_->GetCollisionEntitiesMessage(
                            collision_debug.collisions[0]));

  std::tuple entities =
      collision_checker_->GetCollisionEntities(collision_debug.collisions[0]);
  intrinsic_proto::motion_planning::v1::CollisionDebug collision_debug_message;
  *collision_debug_message.mutable_left_entity() =
      std::get<0>(entities_to_report);
  *collision_debug_message.mutable_right_entities() =
      absl::StrJoin(std::get<1>(entities_to_report), "\n");
  collision_debug_message.set_left_entity_id(std::get<0>(entities).value());
  for (const auto& right_entity_id : std::get<1>(entities)) {
    collision_debug_message.add_right_entity_ids(right_entity_id.value());
  }
  return collision_debug_message;
}

absl::StatusOr<const std::vector<std::unique_ptr<ConstraintInterface>>*>
WorldKinematicsSystemProxy::GetConstraints() const {
  return &constraints_;
}

absl::StatusOr<std::unique_ptr<icon::ManipulatorKinematics>>
WorldKinematicsSystemProxy::GetManipulatorKinematics() const {
  INTR_ASSIGN_OR_RETURN(auto skeleton, world_->BuildChainSkeleton(
                                           base_object_id_, tip_object_id_));
  INTR_ASSIGN_OR_RETURN(kinematics::Chain chain,
                        kinematics::CreateChainFromModel(*skeleton));
  INTR_ASSIGN_OR_RETURN(const std::string solver_key,
                        SelectIkSolverKeyForChain(chain));

  VLOG(2) << "Constructing ManipulatorKinematics with IK solver key: "
          << solver_key;

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<kinematics::InverseKinematicsInterface>
                            inverse_kinematics,
                        kinematics::GetGlobalInverseKinematicsFactory()
                            .CreateInverseKinematicsSolver(solver_key, chain));
  return std::make_unique<icon::NewManipulatorKinematicsImpl>(
      std::move(inverse_kinematics), std::move(skeleton));
}

}  // namespace intrinsic
