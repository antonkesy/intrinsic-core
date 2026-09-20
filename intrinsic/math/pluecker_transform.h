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

#ifndef INTRINSIC_MATH_PLUECKER_TRANSFORM_H_
#define INTRINSIC_MATH_PLUECKER_TRANSFORM_H_

#include "intrinsic/eigenmath/skew_symmetric_matrix.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {

// Returns a 6d Pluecker motion transformation matrix (c.f. Featherstone, "Rigid
// Body Dynamics Algorithms", Chapter 2, 'Spatial Vector Algebra'). We follow
// the convention "position before orientation". This transformation matrix also
// forms the adjoint matrix for the Special Euclidean Group SE(3).  Usage
// example: coordinate transform of spatial velocities (twists).
template <typename Scalar, int Options>
eigenmath::Matrix6<Scalar> PlueckerMotionTransform(
    const Pose3<Scalar, Options>& pose) {
  eigenmath::Matrix3<Scalar> p_skew =
      eigenmath::SkewSymmetricMatrix(pose.translation());
  const eigenmath::Matrix3<Scalar> R = pose.rotationMatrix();
  eigenmath::Matrix6<Scalar> T;
  T << R, p_skew * R, eigenmath::Matrix3<Scalar>::Zero(), R;
  return T;
}

// Returns closed-form inverse of the 6d Pluecker motion transformation matrix.
// For efficiency, use this operation rather than numerically inverting the
// Pluecker motion transform matrix.
template <typename Scalar, int Options>
eigenmath::Matrix6<Scalar> PlueckerMotionTransformInverse(
    const Pose3<Scalar, Options>& pose) {
  return PlueckerForceTransform(pose).transpose();
}

// Returns a 6d Pluecker force transformation matrix (c.f. Featherstone,
// "Rigid Body Dynamics Algorithms", Chapter 2, 'Spatial Vector Algebra'). We
// follow the convention "position before orientation". Usage example:
// coordinate transform of spatial forces (wrenches).
template <typename Scalar, int Options>
eigenmath::Matrix6<Scalar> PlueckerForceTransform(
    const Pose3<Scalar, Options>& pose) {
  eigenmath::Matrix3<Scalar> p_skew =
      eigenmath::SkewSymmetricMatrix(pose.translation());
  const eigenmath::Matrix3<Scalar> R = pose.rotationMatrix();
  eigenmath::Matrix6<Scalar> T;
  T << R, eigenmath::Matrix3<Scalar>::Zero(), p_skew * R, R;
  return T;
}

// Returns closed-form inverse of the 6d Pluecker force transformation matrix.
// For efficiency, use this operation rather than numerically inverting the
// Pluecker force transform matrix.
template <typename Scalar, int Options>
eigenmath::Matrix6<Scalar> PlueckerForceTransformInverse(
    const Pose3<Scalar, Options>& pose) {
  return PlueckerMotionTransform(pose).transpose();
}

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_PLUECKER_TRANSFORM_H_
