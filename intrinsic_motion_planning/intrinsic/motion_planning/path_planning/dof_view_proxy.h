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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DOF_VIEW_PROXY_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DOF_VIEW_PROXY_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/grouping.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/proto/collision_checker_config.pb.h"
#include "intrinsic/world/proto/entity_search.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// For consistency, we expose the function that converts a list of robots to a
// flattened list of joints. This allows callers to match specific dofs against
// a path using the ordering that this proxy uses during construction.
absl::StatusOr<std::vector<intrinsic_proto::world::EntitySearchCriteria>>
GetJointCriteriaFromGroups(const World& world,
                           const std::vector<GroupId>& group_ids);

// This proxy wraps a DofView. Ideally we would pass a dof view directly, but
// the dof view is tied to the world it is created from, so it makes the
// ownership more clear if this Proxy gets a world con constructs the dof_view
// itself.
class DofViewProxy : public KinematicsSystemProxy {
 public:
  // In all of these Create methods, the given RuleSet will get merged with the
  // World's collision RuleSet. If `collision_checker_config` is unconfigured or
  // omitted (CONFIG_NOT_SET), defaults to `kDefaultCollisionCheckerConfigCase`.
  static absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> Create(
      const World& world, const std::vector<JointEntityId>& joint_entity_ids,
      std::vector<std::unique_ptr<ConstraintInterface>> constraints = {},
      const intrinsic_proto::RuleSet& rule_set = intrinsic_proto::RuleSet(),
      const intrinsic_proto::world::CollisionCheckerConfig&
          collision_checker_config = {},
      bool disable_collision_checking = false);

  static absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> Create(
      const World& world, RobotCollectionsEntityId robot_id,
      std::vector<std::unique_ptr<ConstraintInterface>> constraints = {},
      const intrinsic_proto::RuleSet& rule_set = intrinsic_proto::RuleSet(),
      const intrinsic_proto::world::CollisionCheckerConfig&
          collision_checker_config = {},
      bool disable_collision_checking = false);

  static absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> Create(
      const object_world::ObjectWorld& object_world,
      RobotCollectionsEntityId robot_id,
      std::optional<
          intrinsic_proto::motion_planning::v1::UniformGeometricConstraint>
          constraints_proto = std::nullopt,
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

  // These fk/ik functions are all left unimplemented for now, because this
  // version of the proxy has been stripped down to represent a single dof view,
  // without all the label complexity.
  //
  // As we go forward, we can reimplement them, but we expect the interfaces may
  // need to change to support branching in a single dof view.
  absl::StatusOr<Pose3d> GetFk(const eigenmath::VectorXd& configuration,
                               const LabelId& named_location) const override;
  absl::StatusOr<Pose3d> GetFkInWorld(
      const eigenmath::VectorXd& configuration,
      const LabelId& named_location) const override;
  absl::StatusOr<std::vector<eigenmath::VectorXd>> GetIk(
      const Pose3d& pose) const override;

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
  // constraints. Currently un-implemented, thus the method immediately throws a
  // `kUnimplementedError`, until the KinematicsSystemProxies are consolidated
  // into one (b/311201472).
  absl::StatusOr<eigenmath::VectorXd> ProjectOntoConstraintManifolds(
      const eigenmath::VectorXd& configuration, size_t max_projection_steps,
      double max_constraint_error_norm) const override;

 private:
  DofViewProxy(std::unique_ptr<World> world,
               std::unique_ptr<DofKinematicView> dof_view,
               std::shared_ptr<CollisionChecker> collision_checker,
               std::vector<std::unique_ptr<ConstraintInterface>> constraints);

  std::unique_ptr<World> world_;
  std::unique_ptr<DofKinematicView> dof_view_;
  std::shared_ptr<CollisionChecker> collision_checker_;
  std::vector<std::unique_ptr<ConstraintInterface>> constraints_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DOF_VIEW_PROXY_H_
