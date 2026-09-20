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

#include "intrinsic/kinematics/math.h"

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

using eigenmath::Matrix3d;
using eigenmath::Matrix6d;
using eigenmath::Vector3d;

Matrix6d VelocityRotationMatrix(const Pose3d& a_pose_b) {
  Matrix6d a_W_b = Matrix6d::Zero();
  const Matrix3d R_a_b = a_pose_b.rotationMatrix();
  a_W_b.block<3, 3>(0, 0) = R_a_b;
  a_W_b.block<3, 3>(3, 3) = R_a_b;
  return a_W_b;
}

}  // namespace kinematics
}  // namespace intrinsic
