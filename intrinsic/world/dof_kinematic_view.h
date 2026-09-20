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

#ifndef INTRINSIC_WORLD_DOF_KINEMATIC_VIEW_H_
#define INTRINSIC_WORLD_DOF_KINEMATIC_VIEW_H_

#include <cstddef>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/util/string_type.h"
#include "intrinsic/world/configuration_validation.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/labels.h"

namespace intrinsic {

INTRINSIC_DEFINE_STRING_TYPE_AS(DofId,
                                intrinsic::SharedPtrStringRepresentation);

// The DofKinematicView is an abstraction to allow for easy use of the
// kinematics systems available within the World. A view is associated with
// an abstract coordinate system. Some of those coordinates are free and form a
// set of controllable degrees of freedom (DOF). The exact details of how those
// Dofs modify the World are not exposed to the user of the view.
class DofKinematicView {
 public:
  virtual ~DofKinematicView() = default;

  // Sets the Dofs represented by this view. If enforce_limits is set it will
  // error on out of range dof values.
  virtual absl::Status SetDofValues(
      const eigenmath::VectorXd& dof_values, bool enforce_limits,
      std::optional<absl::Time> timestamp = std::nullopt,
      bool enforce_monotonic_time = false) = 0;

  // Gets the Dofs represented by this KinematicView.
  virtual eigenmath::VectorXd GetDofValues() const = 0;

  // Gets the number of Dofs represented by this KinematicView.
  virtual size_t GetDofCount() const = 0;

  // Gets all the dof labels associated to this view.
  virtual std::vector<DofLabel> GetDofLabels() const = 0;

  // Gets all the dof ids of this view.
  virtual std::vector<DofId> GetAllDofIds() const = 0;

  // Gets the JointEntityIds of this view.
  virtual const std::vector<JointEntityId>& GetJointEntityIds() const = 0;

  // Gets the lower dof value system limits as defined in the skeleton.
  virtual eigenmath::VectorXd GetLowerDofValueSystemLimits() const = 0;

  // Gets the upper dof value system limits as defined in the skeleton.
  virtual eigenmath::VectorXd GetUpperDofValueSystemLimits() const = 0;

  // Gets the lower dof value application limits as defined in the skeleton.
  virtual eigenmath::VectorXd GetLowerDofValueApplicationLimits() const = 0;

  // Gets the upper dof value application limits as defined in the skeleton.
  virtual eigenmath::VectorXd GetUpperDofValueApplicationLimits() const = 0;

  // Gets the <lower, upper≥ dof value application limits as defined in the
  // skeleton.
  virtual std::pair<eigenmath::VectorXd, eigenmath::VectorXd>
  GetDofValueApplicationLimits() const = 0;

  // Return the application position, velocity, acceleration and jerk limits for
  // all Dofs. These are used primarily in trajectory planning.
  virtual JointLimitsXd GetDofApplicationLimits() const = 0;

  // Return the system position, velocity, acceleration and jerk limits for
  // all Dofs. These are strict limits enforced by the robot controller.
  // Violating them triggers a fault.
  virtual JointLimitsXd GetDofSystemLimits() const = 0;

  // Sets the application position, velocity, acceleration and jerk limits for
  // all Dofs. Returns an error if current values are outside of the new limits.
  virtual absl::Status SetDofApplicationLimits(const JointLimitsXd& limits,
                                               bool enforce_limits) = 0;

  // Sets the system position, velocity, acceleration and jerk limits for
  // all Dofs. Returns an error if current values are outside of the new limits.
  virtual absl::Status SetDofSystemLimits(const JointLimitsXd& limits,
                                          bool enforce_limits) = 0;

  virtual ConfigurationValidator GetConfigurationLimitsValidator(
      ConfigurationValidatorOptions options) const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_DOF_KINEMATIC_VIEW_H_
