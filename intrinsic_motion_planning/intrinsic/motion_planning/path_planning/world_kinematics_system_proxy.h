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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_WORLD_KINEMATICS_SYSTEM_PROXY_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_WORLD_KINEMATICS_SYSTEM_PROXY_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/configuration_validation.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/grouping.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/proto/collision_checker_config.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// A KinematicsSystemProxy implementation that bridges to the intrinsic::World.
class WorldKinematicsSystemProxy : public KinematicsSystemProxy {
 public:
  // Creates a WorldKinematicsSystemProxy for a kinematic chain between
  // `base_object_id` and `tip_object_id`.
  //
  // `world` contains the kinematic entities. `dof_validator_options` controls
  // joint limit validation. `constraints` are optional constraints to validate
  // against configurations. `rule_set` is merged with the World's default rule
  // set. `collision_checker_config` specifies the collision checker
  // configuration (defaults to `kDefaultCollisionCheckerConfigCase` if omitted
  // or unconfigured). If `disable_collision_checking` is true, collision
  // checking is disabled.
  static absl::StatusOr<std::unique_ptr<WorldKinematicsSystemProxy>> Create(
      const World& world, PhysicalEntityId base_object_id,
      PhysicalEntityId tip_object_id,
      const ConfigurationValidatorOptions& dof_validator_options,
      std::vector<std::unique_ptr<ConstraintInterface>> constraints = {},
      const intrinsic_proto::RuleSet& rule_set = intrinsic_proto::RuleSet(),
      const intrinsic_proto::world::CollisionCheckerConfig&
          collision_checker_config = {},
      bool disable_collision_checking = false);

  absl::StatusOr<JointConfigurationValidationResult> IsValid(
      const eigenmath::VectorXd& configuration,
      CollisionCheckingDebug* collision_debug) const override;

  absl::StatusOr<bool> IsWithinLimits(
      const eigenmath::VectorXd& configuration) const override;

  std::vector<DofLabel> GetDofLabels() const override;

  std::pair<PhysicalEntityId, PhysicalEntityId> GetBaseAndTipIds()
      const override;

  absl::StatusOr<Pose3d> GetFk(const eigenmath::VectorXd& configuration,
                               const LabelId& named_location) const override;

  absl::StatusOr<Pose3d> GetFkInWorld(
      const eigenmath::VectorXd& configuration,
      const LabelId& named_location) const override;

  absl::StatusOr<std::vector<eigenmath::VectorXd>> GetIk(
      const Pose3d& pose) const override;
  absl::StatusOr<std::vector<eigenmath::VectorXd>> GetIk(
      const Pose3d& pose, const eigenmath::VectorXd& seed) const override;

  JointLimitsXd GetJointLimits() const override;
  absl::Status SetJointLimits(const JointLimitsXd& joint_limits) override;
  std::shared_ptr<CollisionChecker> GetCollisionChecker() const override;

  absl::StatusOr<eigenmath::VectorXd> GetRandomConfigNear(
      const eigenmath::VectorXd& config, double distance,
      int* seed) const override;

  absl::StatusOr<eigenmath::VectorXd> GetRandomConfigNear(
      const eigenmath::VectorXd& configuration,
      const eigenmath::VectorXd& distance, int* seed) const override;

  absl::StatusOr<eigenmath::VectorXd> GetRandomConfig(int* seed) const override;
  absl::StatusOr<eigenmath::VectorXd> GetRandomConfig(
      int* seed, const eigenmath::VectorXd& lower_limits,
      const eigenmath::VectorXd& upper_limits) const override;

  absl::StatusOr<std::string> PrintCollisionCheckingDebug(
      const CollisionCheckingDebug& collision_debug) const override;

  absl::StatusOr<intrinsic_proto::motion_planning::v1::CollisionDebug>
  GetCollisionDebugMessage(
      const CollisionCheckingDebug& collision_debug) const override;

  absl::StatusOr<const std::vector<std::unique_ptr<ConstraintInterface>>*>
  GetConstraints() const override;

  absl::StatusOr<bool> AreConstraintsSatisfied(
      const eigenmath::VectorXd& configuration) const override;

  // Perform an (iterative) projection of the configuration onto the constraint
  // manifolds, in order to obtain the closest configuration that satisfies all
  // constraints. The parameter `max_constraint_error_norm` governs how small
  // each projection step is performed, taking into account the locality
  // assumption of a Jacobian matrix, i.e. the aggregated constraint Jacobian.
  // Throws `kInternalError` if the projection does not converge within
  // `max_projection_steps` iterations. For the usage, please refer to
  // go/intrinsic-path-constrained-sbmp-improvement-design and
  // go/intrinsic-path-constrained-sbmp-improvement-presentation for details.
  absl::StatusOr<eigenmath::VectorXd> ProjectOntoConstraintManifolds(
      const eigenmath::VectorXd& configuration, size_t max_projection_steps,
      double max_constraint_error_norm) const override;

  absl::StatusOr<std::unique_ptr<icon::ManipulatorKinematics>>
  GetManipulatorKinematics() const override;

 private:
  WorldKinematicsSystemProxy(
      std::unique_ptr<World> world, PhysicalEntityId base_object_id,
      PhysicalEntityId tip_object_id,
      const ConfigurationValidatorOptions& dof_validator_options,
      std::unique_ptr<DofKinematicView> dov_view,
      std::unique_ptr<CartesianKinematicView> cart_view,
      std::shared_ptr<CollisionChecker> collision_checker,
      std::vector<std::unique_ptr<ConstraintInterface>> constraints);

  std::unique_ptr<World> world_;
  PhysicalEntityId base_object_id_;
  PhysicalEntityId tip_object_id_;
  GroupId robot_id_;
  std::unique_ptr<DofKinematicView> dof_view_;
  std::unique_ptr<CartesianKinematicView> cart_view_;
  std::shared_ptr<CollisionChecker> collision_checker_;
  std::vector<std::unique_ptr<ConstraintInterface>> constraints_;

  // This function returns true when the path is collision free.
  // It checks both endpoints when provided to avoid asymmetry in checking.
  std::function<bool(const eigenmath::VectorXd& q_dest)> dof_validator_fn_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_WORLD_KINEMATICS_SYSTEM_PROXY_H_
