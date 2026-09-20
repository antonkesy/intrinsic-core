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

#ifndef INTRINSIC_WORLD_ASPECTS_ENTITY_DOF_KINEMATIC_VIEW_H_
#define INTRINSIC_WORLD_ASPECTS_ENTITY_DOF_KINEMATIC_VIEW_H_

#include <cstddef>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/configuration_validation.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/labels.h"

namespace intrinsic {
namespace entity_kinematic_world_details {

// The fact that this file contains two classes (ConstEntityDofKinematicView and
// EntityDofKinematicView) is intended to be a World-internal implementation
// detail. The need for two classes arises from the fact that const-ness only
// applies to pointers themselves and not the underlying type. (That is, if
// class Foo has a "Bar* bar_" member, a const Foo effectively has a "Bar const*
// bar_" member and NOT a "const Bar const* bar_".)
class ConstEntityDofKinematicView : public DofKinematicView {
 public:
  ConstEntityDofKinematicView(
      const entity_aspect_world_details::EntityWorld* world,
      const std::vector<JointEntityId>& joints);

  // Sets the Dofs represented by this view.
  absl::Status SetDofValues(const eigenmath::VectorXd& dof_values,
                            bool enforce_limits,
                            std::optional<absl::Time> timestamp = std::nullopt,
                            bool enforce_monotonic_time = false) override;

  // Gets the Dofs represented by this KinematicView.
  eigenmath::VectorXd GetDofValues() const override;

  // Gets the number of Dofs represented by this KinematicView.
  size_t GetDofCount() const override;

  // Gets all the dof labels associated to this view.
  std::vector<DofLabel> GetDofLabels() const override;

  // Gets all the dof ids of this view.
  std::vector<DofId> GetAllDofIds() const override;

  // Gets the JointEntityIds of this view.
  const std::vector<JointEntityId>& GetJointEntityIds() const override;

  // Gets the lower dof value system limits as defined in the skeleton.
  eigenmath::VectorXd GetLowerDofValueSystemLimits() const override;

  // Gets the upper dof value system limits as defined in the skeleton.
  eigenmath::VectorXd GetUpperDofValueSystemLimits() const override;

  // Gets the lower dof value application limits as defined in the skeleton.
  eigenmath::VectorXd GetLowerDofValueApplicationLimits() const override;

  // Gets the upper dof value application limits as defined in the skeleton.
  eigenmath::VectorXd GetUpperDofValueApplicationLimits() const override;

  // Gets the <lower, upper≥ dof value application limits as defined in the
  // skeleton.
  std::pair<eigenmath::VectorXd, eigenmath::VectorXd>
  GetDofValueApplicationLimits() const override;

  // Return the application position, velocity, acceleration and jerk limits for
  // all Dofs. These are used primarily in trajectory planning.
  JointLimitsXd GetDofApplicationLimits() const override;

  // Return the system position, velocity, acceleration and jerk limits for
  // all Dofs. These are strict limits enforced by the robot controller.
  // Violating them triggers a fault.
  JointLimitsXd GetDofSystemLimits() const override;

  // Sets the application position, velocity, acceleration and jerk limits for
  // all Dofs. Returns an error if current values are outside of the new limits.
  absl::Status SetDofApplicationLimits(const JointLimitsXd& limits,
                                       bool enforce_limits) override;

  // Sets the system position, velocity, acceleration and jerk limits for
  // all Dofs. Returns an error if current values are outside of the new limits.
  absl::Status SetDofSystemLimits(const JointLimitsXd& limits,
                                  bool enforce_limits) override;

  ConfigurationValidator GetConfigurationLimitsValidator(
      ConfigurationValidatorOptions options) const override;

 protected:
  const KinematicsComponent* GetKinematicsComponent(
      int joint_handles_index) const;

  // The reference to the world that this view belongs to.
  const entity_aspect_world_details::EntityWorld* const_world_ = nullptr;

  // List of handles to joints controlled by this view. The order of this list
  // dictates the order of vectors used in implementing the DofKinematicView
  // interface.
  std::vector<JointEntityId> joint_handles_;
};

class EntityDofKinematicView final : public ConstEntityDofKinematicView {
 public:
  EntityDofKinematicView(entity_aspect_world_details::EntityWorld* world,
                         const std::vector<JointEntityId>& joints);

  // Sets the Dofs represented by this view.
  absl::Status SetDofValues(const eigenmath::VectorXd& dof_values,
                            bool enforce_limits,
                            std::optional<absl::Time> timestamp = std::nullopt,
                            bool enforce_monotonic_time = false) override;

  // Sets the application position, velocity, acceleration and jerk limits for
  // all Dofs. Returns an error if current values are outside of the new limits.
  absl::Status SetDofApplicationLimits(const JointLimitsXd& limits,
                                       bool enforce_limits) override;

  // Sets the system position, velocity, acceleration and jerk limits for
  // all Dofs. Returns an error if current values are outside of the new limits.
  absl::Status SetDofSystemLimits(const JointLimitsXd& limits,
                                  bool enforce_limits) override;

 private:
  KinematicsComponent* GetKinematicsComponent(int joint_handles_index);

  // The reference to the world that this view belongs to.
  entity_aspect_world_details::EntityWorld* world_ = nullptr;
};

}  // namespace entity_kinematic_world_details
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_ASPECTS_ENTITY_DOF_KINEMATIC_VIEW_H_
