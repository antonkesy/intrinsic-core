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

#include "intrinsic/world/aspects/chained_robot_cartesian_kinematic_view.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/aggregate_type.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/aspects/entity_deferred_ik_problem.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"

namespace intrinsic {
namespace entity_robots_details {

namespace {

std::string RobotNamesString(
    const entity_aspect_world_details::EntityWorld& world,
    const std::vector<RobotCollectionsEntityId>& robot_ids) {
  return absl::StrJoin(
      robot_ids, ",",
      [&world](std::string* str, const RobotCollectionsEntityId& id) {
        absl::StrAppend(str, world.GetLocalNameForEntityById(id),
                        " id=", id.value());
      });
}

}  // namespace

ConstChainedRobotCartesianKinematicView::
    ConstChainedRobotCartesianKinematicView(
        const entity_aspect_world_details::EntityWorld* world,
        PhysicalEntityId obj1, PhysicalEntityId obj2)
    : world_(world) {
  control_points_.first = obj1;
  control_points_.second = obj2;
}

absl::Status ConstChainedRobotCartesianKinematicView::Initialize() {
  INTR_ASSIGN_OR_RETURN(robot_ids_,
                        world_->GetRobotIdsInChain(control_points_.first,
                                                   control_points_.second));

  if (robot_ids_.empty()) {
    return intrinsic::NotFoundErrorBuilder()
           << "No robots found between "
           << world_->GetLocalNameForEntityById(control_points_.first)
           << " id=" << control_points_.first << " to "
           << world_->GetLocalNameForEntityById(control_points_.second)
           << " id=" << control_points_.second;
  }

  INTR_ASSIGN_OR_RETURN(chain_base_, world_->GetBaseLink(robot_ids_.front()));

  INTR_ASSIGN_OR_RETURN(auto tip_candidates,
                        world_->GetFinalEntitiesOfRobot(robot_ids_.back()));
  chain_tip_ = AttachmentEntityId(kInvalidEntityId);
  for (const auto& tip_candidate : tip_candidates) {
    INTR_ASSIGN_OR_RETURN(
        AttachmentEntityId common_ancestor,
        world_->FindCommonAncestor(control_points_.second, tip_candidate));
    if (common_ancestor == tip_candidate) {
      chain_tip_ = tip_candidate;
      break;
    }
  }

  if (chain_tip_ == kInvalidEntityId) {
    return intrinsic::NotFoundErrorBuilder()
           << "Unable to find the tip of the robot chain = {"
           << RobotNamesString(*world_, robot_ids_) << "}";
  }

  VLOG(1) << "Creating a ConstChainedRobotCartesianKinematicView for robots={"
          << RobotNamesString(*world_, robot_ids_)
          << "} from base=" << world_->GetLocalNameForEntityById(chain_base_)
          << " id=" << chain_base_
          << " to tip=" << world_->GetLocalNameForEntityById(chain_tip_)
          << " id=" << chain_tip_ << " for control points from "
          << world_->GetLocalNameForEntityById(control_points_.first)
          << " id=" << control_points_.first << " to "
          << world_->GetLocalNameForEntityById(control_points_.second)
          << " id=" << control_points_.second;

  INTR_ASSIGN_OR_RETURN(const_dof_view_,
                        world_->GetDofKinematicView(chain_base_, chain_tip_));

  return absl::OkStatus();
}

std::pair<PhysicalEntityId, PhysicalEntityId>
ConstChainedRobotCartesianKinematicView::ObjectsBeingControlled() const {
  // TODO(b/155232086): PhysicalEntityId should be aliased to AttachmentEntityId
  // instead of PhysicalEntityId, but until then, we have to cast to a more
  // specific TypedEntityId.
  return {PhysicalEntityId(control_points_.first.value()),
          PhysicalEntityId(control_points_.second.value())};
}

absl::StatusOr<std::string>
ConstChainedRobotCartesianKinematicView::GetIKSolverKey() const {
  // TODO(jeanfrancoisd): Make the solver key a parameters
  // entity_kinematic_world_details::GetIKSolverKey support a chain of robots.
  return "kinematic_chain";
}

std::unique_ptr<IkSolutions>
ConstChainedRobotCartesianKinematicView::GetIkSolutions(
    const Pose3d& obj1_t_obj2) const {
  CHECK_NE(const_dof_view_, nullptr) << "Initialize should have been called";
  eigenmath::VectorXd dof_values = const_dof_view_->GetDofValues();
  return GetIkSolutionsImpl(obj1_t_obj2, &dof_values);
}

std::unique_ptr<IkSolutions>
ConstChainedRobotCartesianKinematicView::GetIkSolutions(
    const Pose3d& obj1_t_obj2, const eigenmath::VectorXd& dof_values) const {
  return GetIkSolutionsImpl(obj1_t_obj2, &dof_values);
}

std::unique_ptr<IkSolutions>
ConstChainedRobotCartesianKinematicView::GetIkSolutionsImpl(
    const Pose3d& obj1_t_obj2, const eigenmath::VectorXd* dof_values) const {
  Pose3d control_point_t_robot_base =
      world_->GetTransform(control_points_.first, chain_base_);
  Pose3d robot_tip_t_control_point =
      world_->GetTransform(chain_tip_, control_points_.second);
  const auto robot_base_t_robot_tip = control_point_t_robot_base.inverse() *
                                      obj1_t_obj2 *
                                      robot_tip_t_control_point.inverse();

  ASSIGN_OR_DIE(auto solver_key, GetIKSolverKey());
  return std::make_unique<
      entity_kinematic_world_details::EntityDeferredIKProblem>(
      *world_, solver_key, chain_base_, chain_tip_, robot_base_t_robot_tip,
      dof_values);
}

std::shared_ptr<const DofKinematicView>
ConstChainedRobotCartesianKinematicView::GetDofView() const {
  CHECK_NE(const_dof_view_, nullptr) << "Initialize should have been called";
  return const_dof_view_;
}

std::shared_ptr<DofKinematicView>
ConstChainedRobotCartesianKinematicView::GetDofView() {
  LOG(FATAL) << "Non-const version of "
                "ConstChainedRobotCartesianKinematicView::GetDofView() should "
                "never be called";
}

ChainedRobotCartesianKinematicView::ChainedRobotCartesianKinematicView(
    entity_aspect_world_details::EntityWorld* world, PhysicalEntityId obj1,
    PhysicalEntityId obj2)
    : ConstChainedRobotCartesianKinematicView(world, obj1, obj2) {
  ASSIGN_OR_DIE(dof_view_, world->GetDofKinematicView(obj1, obj2));
}

std::shared_ptr<DofKinematicView>
ChainedRobotCartesianKinematicView::GetDofView() {
  return dof_view_;
}

}  // namespace entity_robots_details
}  // namespace intrinsic
