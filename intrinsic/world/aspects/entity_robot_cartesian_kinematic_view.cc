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

#include "intrinsic/world/aspects/entity_robot_cartesian_kinematic_view.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/world/aspects/entity_deferred_ik_problem.h"
#include "intrinsic/world/aspects/entity_dof_kinematic_view.h"
#include "intrinsic/world/aspects/entity_kinematic_utils.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"

namespace intrinsic {
namespace entity_robots_details {

ConstEntityRobotCartesianKinematicView::ConstEntityRobotCartesianKinematicView(
    const entity_aspect_world_details::EntityWorld* world,
    RobotCollectionsEntityId robot_id)
    : world_(world), robot_id_(robot_id) {
  // TODO(intrinsic-eng): The order of the two objects should not matter.
  ASSIGN_OR_DIE(control_points_.first, world->GetBaseLink(robot_id));
  ASSIGN_OR_DIE(control_points_.second,
                world_->GetFinalEntityOfRobotKinematicChain(robot_id));
  CHECK_NE(control_points_.second, kInvalidEntityId)
      << "Robot " << robot_id.value() << " does not form a linear chain";

  ASSIGN_OR_DIE(std::vector<JointEntityId> dof_ids,
                world_->GetRobotDofs(robot_id_));
  const_dof_view_ = std::make_unique<
      entity_kinematic_world_details::ConstEntityDofKinematicView>(world_,
                                                                   dof_ids);
}

ConstEntityRobotCartesianKinematicView::ConstEntityRobotCartesianKinematicView(
    const entity_aspect_world_details::EntityWorld* world,
    const std::pair<AttachmentEntityId, AttachmentEntityId>& control_points,
    RobotCollectionsEntityId robot_id)
    : ConstEntityRobotCartesianKinematicView(world, robot_id) {
  // TODO(intrinsic-eng): The order of the two objects should not matter.
  // Make sure that the first control point is an ancestor of the robot's base.
  const auto& [robot_base, robot_tip] = control_points_;
  ASSIGN_OR_DIE(auto ancestor,
                world_->FindCommonAncestor(robot_base, control_points.first));
  CHECK_EQ(ancestor, control_points.first)
      << "first control point is not an ancestor of the robot's base";

  // Make sure the second control point is a descendent of the robot's tip
  // (which was assigned to control_points_.second).
  ASSIGN_OR_DIE(ancestor,
                world_->FindCommonAncestor(robot_tip, control_points.second));
  CHECK_EQ(ancestor, robot_tip)
      << "second control point is not a descendent of the robot's tip";

  control_points_ = control_points;
}

std::pair<PhysicalEntityId, PhysicalEntityId>
ConstEntityRobotCartesianKinematicView::ObjectsBeingControlled() const {
  // TODO(b/155232086): PhysicalEntityId should be aliased to AttachmentEntityId
  // instead of PhysicalEntityId, but until then, we have to cast to a more
  // specific TypedEntityId.
  return {PhysicalEntityId(control_points_.first.value()),
          PhysicalEntityId(control_points_.second.value())};
}

absl::StatusOr<std::string>
ConstEntityRobotCartesianKinematicView::GetIKSolverKey() const {
  return entity_kinematic_world_details::GetIKSolverKey(*world_, robot_id_);
}

std::unique_ptr<IkSolutions>
ConstEntityRobotCartesianKinematicView::GetIkSolutions(
    const Pose3d& obj1_t_obj2) const {
  ASSIGN_OR_DIE(auto robot_dofs, world_->GetRobotDofs(robot_id_));
  eigenmath::VectorXd dof_values(robot_dofs.size());
  for (int i = 0; i < robot_dofs.size(); i++) {
    ASSIGN_OR_DIE(const auto* dof_entity, world_->GetEntityById(robot_dofs[i]));
    ASSIGN_OR_DIE(const auto* dof_kinematics,
                  dof_entity->GetComponent<KinematicsComponent>());
    dof_values[i] = dof_kinematics->GetRawValue();
  }
  return GetIkSolutionsImpl(obj1_t_obj2, &dof_values);
}

std::unique_ptr<IkSolutions>
ConstEntityRobotCartesianKinematicView::GetIkSolutions(
    const Pose3d& obj1_t_obj2, const eigenmath::VectorXd& dof_values) const {
  return GetIkSolutionsImpl(obj1_t_obj2, &dof_values);
}

std::unique_ptr<IkSolutions>
ConstEntityRobotCartesianKinematicView::GetIkSolutionsImpl(
    const Pose3d& obj1_t_obj2, const eigenmath::VectorXd* dof_values) const {
  ASSIGN_OR_DIE(auto robot_base_id, world_->GetBaseLink(robot_id_));
  ASSIGN_OR_DIE(auto robot_tip_id,
                world_->GetFinalEntityOfRobotKinematicChain(robot_id_));
  CHECK_NE(robot_tip_id, kInvalidEntityId)
      << "Robot " << robot_id_.value() << " does not form a linear chain";

  Pose3d robot_base_t_control_point =
      world_->GetTransform(robot_base_id, control_points_.first);
  Pose3d control_point_t_robot_tip =
      world_->GetTransform(control_points_.second, robot_tip_id);
  const auto robot_base_t_robot_tip =
      robot_base_t_control_point * obj1_t_obj2 * control_point_t_robot_tip;

  return std::make_unique<
      entity_kinematic_world_details::EntityDeferredIKProblem>(
      *world_, robot_base_id, robot_tip_id, robot_base_t_robot_tip, dof_values);
}

std::shared_ptr<const DofKinematicView>
ConstEntityRobotCartesianKinematicView::GetDofView() const {
  return const_dof_view_;
}

std::shared_ptr<DofKinematicView>
ConstEntityRobotCartesianKinematicView::GetDofView() {
  LOG(FATAL) << "Non-const version of "
                "ConstEntityRobotCartesianKinematicView::GetDofView() should "
                "never be called";
}

EntityRobotCartesianKinematicView::EntityRobotCartesianKinematicView(
    entity_aspect_world_details::EntityWorld* world,
    RobotCollectionsEntityId robot_id)
    : ConstEntityRobotCartesianKinematicView(world, robot_id) {
  ASSIGN_OR_DIE(std::vector<JointEntityId> dof_ids,
                world_->GetRobotDofs(robot_id_));
  dof_view_ =
      std::make_unique<entity_kinematic_world_details::EntityDofKinematicView>(
          world, dof_ids);
}

// Constructor for a view that manipulates a robot, using an ancestor of its
// base and a descendent of its tip as reference points.
EntityRobotCartesianKinematicView::EntityRobotCartesianKinematicView(
    entity_aspect_world_details::EntityWorld* world,
    const std::pair<AttachmentEntityId, AttachmentEntityId>& control_points,
    RobotCollectionsEntityId robot_id)
    : ConstEntityRobotCartesianKinematicView(world, control_points, robot_id) {
  ASSIGN_OR_DIE(std::vector<JointEntityId> dof_ids,
                world_->GetRobotDofs(robot_id_));
  dof_view_ =
      std::make_unique<entity_kinematic_world_details::EntityDofKinematicView>(
          world, dof_ids);
}

std::shared_ptr<DofKinematicView>
EntityRobotCartesianKinematicView::GetDofView() {
  return dof_view_;
}

}  // namespace entity_robots_details
}  // namespace intrinsic
