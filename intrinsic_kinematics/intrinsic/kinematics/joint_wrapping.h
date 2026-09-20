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

#ifndef INTRINSIC_KINEMATICS_JOINT_WRAPPING_H_
#define INTRINSIC_KINEMATICS_JOINT_WRAPPING_H_

#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic {
namespace kinematics {

// The minuscule numerical tolerance for joint-wrapping near the boundary
// (either the lower-bound or the upper-bound).
static constexpr double kJointWrappingBoundaryTolerance = 1.0e-8;

// Check if joint_state is within the positional limits after trying to bringing
// the values within 2π of the limits.
icon::RealtimeStatusOr<bool> IsWithinLimitsAfterWrapping(
    const ModelInterface& model, const JointStateP& joint_state,
    const JointLimits& joint_limits);

// Wrap val to be between upper and lower, by applying multiples of 2 * M_PI,
// such that the closest value to 'nearby' will be found.
// If no solution is found, std::numeric_limits<double>::quiet_NaN() will be
// returned.
// Will die if inputs are invalid.
// TODO(jeanfrancoisd): Hide behind other methods.
double WrapJoint(double val, double upper, double lower, double nearby);

double WrapJoint(double val, double upper, double lower);

// Wrap q within limits, by applying multiples of 2 * M_PI to each joint.
// The solutions closest to nearby_q will be returned.
// Returns true if all joints could be wrapped within limits and returns false
// if it fails to wrap within the limits.
// Failing to wrap between the limits is not considered an error at this level.
// It's left to the caller to decide if failing to wrap should be an error or
// not. Typical error case include mismatch size of arguments.
icon::RealtimeStatusOr<bool> Wrap(const ModelInterface& model,
                                  const JointLimits& dof_limits,
                                  const eigenmath::VectorNd& nearby_q,
                                  eigenmath::VectorNd* q);

// Sorts joint states within qs according to distance to nearby_q after
// unwrapping to the closest solution. Closest solution is first element.
// Important: if two solutions are equidistant to nearby_q, they are not
// specifically ordered, which can lead to unexpected switches of kinematic
// branches.
icon::RealtimeStatus WrapAndSort(const ModelInterface& model,
                                 const JointLimits& dof_limits,
                                 const eigenmath::VectorNd& nearby_q,
                                 absl::Span<eigenmath::VectorNd>* qs);

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_JOINT_WRAPPING_H_
