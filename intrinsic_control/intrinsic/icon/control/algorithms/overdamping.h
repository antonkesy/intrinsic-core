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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_OVERDAMPING_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_OVERDAMPING_H_

#include <limits>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

static constexpr double kMinimumStiffness = 1.0e-12;

// Different options of overdamping constant criterion, based on the paper
// D. Surdilovic, "Contact stability issues in position based impedance control:
// theory and experiments," Proceedings of IEEE International Conference on
// Robotics and Automation, 1996, pp. 1675-1680 vol.2,
// doi: 10.1109/ROBOT.1996.506953.
enum OverdampingConstantCriterion {
  kBenady = 0,            // Eq. (22) in the paper.
  kHInfinity,             // Eq. (21) in the paper.
  kStrictlyPositiveReal,  // Eq. (15) in the paper.
};

struct OverdampingFromStiffnessScalarsInputParameters {
  double desired_virtual_stiffness_contact;
  double stiffness_compliance_device;
  double stiffness_robot_joints = std::numeric_limits<double>::infinity();
  double stiffness_environment = std::numeric_limits<double>::infinity();
  OverdampingConstantCriterion criterion =
      OverdampingConstantCriterion::kBenady;
};

// Computes the overdamping constant based on the selected
// `OverdampingConstantCriterion` above, with stiffness scalars as inputs.
// Returns `kInvalidArgument` if the specified criterion is unknown, or if any
// of the stiffness scalars (either in Newton/meter (for translational
// stiffness) or in (Newton meter/radian) (for rotational stiffness)) is less
// than the `kMinimumStiffness`.
RealtimeStatusOr<double> ComputeOverdampingConstantFromStiffnessScalars(
    const OverdampingFromStiffnessScalarsInputParameters& input_parameters);

// Computes damping matrix following the Surdilovic overdamping criterion.
// `inertia` is the cartesian inertia matrix and `environment_stiffness` is the
// stiffness of the mechanical chain including the tool itself and the
// environment.
RealtimeStatusOr<eigenmath::MatrixNd> ComputeOverdamping(
    const eigenmath::MatrixNd& inertia, double environment_stiffness);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_OVERDAMPING_H_
