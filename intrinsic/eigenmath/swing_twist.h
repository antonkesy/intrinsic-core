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

#ifndef INTRINSIC_EIGENMATH_SWING_TWIST_H_
#define INTRINSIC_EIGENMATH_SWING_TWIST_H_

#include <cmath>

#include "Eigen/Core"
#include "intrinsic/eigenmath/so3.h"

namespace intrinsic {
namespace eigenmath {

// Decomposes q into two swing and twist quaternions such that:
// - swing * twist = so3
// - twist.axis = twist_axis
// - swing.axis is perpendicular to twist_axis
//
// epsilon is used for floating point comparison to zero.
template <class Scalar, int Options>
std::pair<SO3<Scalar, Options>, SO3<Scalar, Options>> SwingTwistDecomposition(
    const SO3<Scalar, Options>& so3, const Vector3<Scalar>& twist_axis,
    double epsilon = 1e-9) {
  // This algorithm is based upon https://arxiv.org/pdf/1506.05481.pdf,
  // specifically Algorithm 1 on page 22, where p corresponds to the swing
  // quaternion and q corresponds to the twist quaternion.
  const Quaternion<Scalar>& q = so3.quaternion();
  double dot = twist_axis.dot(Vector3d(q.x(), q.y(), q.z()));

  if (std::fabs(dot) < epsilon) {
    // q's axis is already perpendicular to twist_axis, so return swing = q and
    // twist = identity.
    return {so3, SO3<Scalar, Options>()};
  }
  SO3<Scalar, Options> twist(
      eigenmath::Quaterniond(q.w(), dot * twist_axis.x(), dot * twist_axis.y(),
                             dot * twist_axis.z()),
      /* do_normalize = */ true);
  return {so3 * twist.inverse(), twist};
}

}  // namespace eigenmath
}  // namespace intrinsic

#endif  // INTRINSIC_EIGENMATH_SWING_TWIST_H_
