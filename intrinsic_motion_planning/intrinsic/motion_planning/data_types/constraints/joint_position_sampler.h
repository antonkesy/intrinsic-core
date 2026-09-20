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

#ifndef INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_SAMPLER_H_
#define INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_SAMPLER_H_

#include <vector>

#include "intrinsic/eigenmath/types.h"

namespace intrinsic {
namespace motion_planning {

// Samples joint positions from a constrained space.
class JointPositionSamplerInterface {
 public:
  virtual ~JointPositionSamplerInterface() = default;

  // Returns a random set of joint positions from the sample space.
  //
  // `num_samples` defines the desired number of sampled joint positions to
  // return. Fewer than this number may be returned if the sample space has a
  // finite number of points or if points within the space cannot be found.
  // Returning fewer than `num_samples` joint positions does not imply that all
  // points in the sample space have been found, and returning no joint
  // positions does not imply that the sample space is empty. Subsequent calls
  // may still find new points within the space.
  //
  // If the sample space is a discrete set of N points, then whenever
  // `N <= num_samples`, all points are returned. Otherwise this function
  // returns random subsets of points.
  virtual std::vector<eigenmath::VectorXd> SampleJointPositions(
      int num_samples) = 0;
};

}  // namespace motion_planning
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_SAMPLER_H_
