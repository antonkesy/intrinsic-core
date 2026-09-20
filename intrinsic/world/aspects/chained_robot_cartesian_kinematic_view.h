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

#ifndef INTRINSIC_WORLD_ASPECTS_CHAINED_ROBOT_CARTESIAN_KINEMATIC_VIEW_H_
#define INTRINSIC_WORLD_ASPECTS_CHAINED_ROBOT_CARTESIAN_KINEMATIC_VIEW_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"

namespace intrinsic {
namespace entity_robots_details {

// The classes in this file implement CartesianKinematicView which is backed by
// a chain of robot Entity.
//
// The fact that there are two classes (ConstChainedRobotCartesianKinematicView
// and ChainedRobotCartesianKinematicView) is intended to be a World-internal
// implementation detail. The need for two classes arises from the fact that
// const-ness only applies to pointers themselves and not the underlying type.
// (That is, if class Foo has a "Bar* bar_" member, a const Foo effectively has
// a "Bar const* bar_" member and NOT a "const Bar const* bar_".)
class ConstChainedRobotCartesianKinematicView : public CartesianKinematicView {
 public:
  // Constructor for a view between two entities. It is expected that a single
  // chain of robots can be established between the two entities.
  ConstChainedRobotCartesianKinematicView(
      const entity_aspect_world_details::EntityWorld* world,
      PhysicalEntityId obj1, PhysicalEntityId obj2);

  ~ConstChainedRobotCartesianKinematicView() override = default;

  absl::Status Initialize();

  // Returns the PhysicalEntityIds associated with this CartesianKinematicView.
  std::pair<PhysicalEntityId, PhysicalEntityId> ObjectsBeingControlled()
      const override;

  absl::StatusOr<std::string> GetIKSolverKey() const override;

  // Returns a set of solutions that would result in the two objects, being
  // controlled by this view, to have the desired transform between them. If
  // there is no error it will always return a valid pointer to a set of
  // solutions. There could be no solutions but the IkSolutions object will be
  // valid and return no solutions when asked.
  std::unique_ptr<IkSolutions> GetIkSolutions(
      const Pose3d& obj1_t_obj2) const override;

  // Returns a set of solutions that would result in the two objects, being
  // controlled by this view, to have the desired transform between them. This
  // is similar to the other GetIkSolutions call but takes an extra hint about
  // what joint configuration should be considered. It is useful when chaining
  // motions to consider the previous solution to make the motion smaller.
  std::unique_ptr<IkSolutions> GetIkSolutions(
      const Pose3d& obj1_t_obj2,
      const eigenmath::VectorXd& dof_values) const override;

  // Returns the underlying DofKinematicView representing the Dofs being
  // controlled by this CartesianKinematicView.
  std::shared_ptr<const DofKinematicView> GetDofView() const override;

  // Returns the underlying DofKinematicView representing the Dofs being
  // controlled by this CartesianKinematicView.
  // For FK, the returned DofKinematicView can be used to set given values and
  // then observe the updated state of the world through, e.g.,
  // PhysicalWorld::GetTransform.
  std::shared_ptr<DofKinematicView> GetDofView() override;

 protected:
  // The reference to the world from which this object originated.
  const entity_aspect_world_details::EntityWorld* world_ = nullptr;

  // EntityId of the robots in the chain being manipulated by this view.
  std::vector<RobotCollectionsEntityId> robot_ids_;

  // Control points (in order of robot base or base ancestor, robot tip or tip
  // descendent) whose relative transform is controlled by this view.
  std::pair<AttachmentEntityId, AttachmentEntityId> control_points_;

  // DofKinematicView returned by GetDofView() const.
  std::shared_ptr<const DofKinematicView> const_dof_view_;

  std::unique_ptr<IkSolutions> GetIkSolutionsImpl(
      const Pose3d& obj1_t_obj2, const eigenmath::VectorXd* dof_values) const;

  LinkEntityId chain_base_;
  AttachmentEntityId chain_tip_;
};

class ChainedRobotCartesianKinematicView
    : public ConstChainedRobotCartesianKinematicView {
 public:
  // Constructor for a view between two entities. It is expected that a single
  // chain of robots can be established between the two entities.
  ChainedRobotCartesianKinematicView(
      entity_aspect_world_details::EntityWorld* world, PhysicalEntityId obj1,
      PhysicalEntityId obj2);

  ~ChainedRobotCartesianKinematicView() override = default;

  // Returns the underlying DofKinematicView representing the Dofs being
  // controlled by this CartesianKinematicView.
  // For FK, the returned DofKinematicView can be used to set given values and
  // then observe the updated state of the world through, e.g.,
  // PhysicalWorld::GetTransform.
  std::shared_ptr<DofKinematicView> GetDofView() override;

 private:
  // DofKinematicView returned by GetDofView().
  std::shared_ptr<DofKinematicView> dof_view_;
};

}  // namespace entity_robots_details
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_ASPECTS_CHAINED_ROBOT_CARTESIAN_KINEMATIC_VIEW_H_
