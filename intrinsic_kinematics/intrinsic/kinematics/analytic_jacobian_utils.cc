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

#include "intrinsic/kinematics/analytic_jacobian_utils.h"

#include <cmath>
#include <limits>

#include "intrinsic/eigenmath/manifolds.h"
#include "intrinsic/eigenmath/skew_symmetric_matrix.h"
#include "intrinsic/eigenmath/so3.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/ipow.h"

namespace intrinsic {
namespace kinematics {

eigenmath::Matrix6d ComputeAnalyticToGeometricJacobianMapper(
    const eigenmath::SO3d& base_R_target) {
  // Default numerical epsilon to avoid division-by-zero.
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

  // The rotation vector or Euler vector representation (which is the angle
  // times the axis from angle-axis representation) of `base_R_target`.
  const eigenmath::Vector3d rotation_vector = eigenmath::logSO3(base_R_target);
  const double norm_rotation_vector = rotation_vector.norm();
  const double norm_rotation_vector_squared =
      intrinsic::IPow(norm_rotation_vector, 2);
  const double norm_rotation_vector_cubed =
      intrinsic::IPow(norm_rotation_vector, 3);
  const eigenmath::Matrix3d skew_symm_rotation_vector =
      eigenmath::SkewSymmetricMatrix(rotation_vector);
  const eigenmath::Matrix3d analytic_to_geometric_orientation_jacobian_mapper =
      eigenmath::Matrix3d::Identity() +
      (skew_symm_rotation_vector * (1.0 - std::cos(norm_rotation_vector)) /
       (norm_rotation_vector_squared + kEpsilon)) +
      (skew_symm_rotation_vector * skew_symm_rotation_vector *
       (norm_rotation_vector - std::sin(norm_rotation_vector)) /
       (norm_rotation_vector_cubed + kEpsilon));
  eigenmath::Matrix6d analytic_to_geometric_jacobian_mapper =
      eigenmath::Matrix6d::Identity();
  analytic_to_geometric_jacobian_mapper.bottomRightCorner(3, 3) =
      analytic_to_geometric_orientation_jacobian_mapper;
  return analytic_to_geometric_jacobian_mapper;
}

eigenmath::Matrix6d ComputeGeometricToAnalyticJacobianMapper(
    const eigenmath::SO3d& base_R_target) {
  // Default numerical epsilon to avoid division-by-zero.
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

  // The rotation vector or Euler vector representation (which is the angle
  // times the axis from angle-axis representation) of `base_R_target`.
  const eigenmath::Vector3d rotation_vector = eigenmath::logSO3(base_R_target);
  const double norm_rotation_vector = rotation_vector.norm();
  const double norm_rotation_vector_squared =
      intrinsic::IPow(norm_rotation_vector, 2);
  const eigenmath::Matrix3d skew_symm_rotation_vector =
      eigenmath::SkewSymmetricMatrix(rotation_vector);
  const eigenmath::Matrix3d geometric_to_analytic_orientation_jacobian_mapper =
      eigenmath::Matrix3d::Identity() - (0.5 * skew_symm_rotation_vector) +
      (skew_symm_rotation_vector * skew_symm_rotation_vector *
       (1.0 / (norm_rotation_vector_squared + kEpsilon)) *
       (1.0 - (0.5 * norm_rotation_vector * std::sin(norm_rotation_vector) /
               (1.0 - std::cos(norm_rotation_vector) + kEpsilon))));
  eigenmath::Matrix6d geometric_to_analytic_jacobian_mapper =
      eigenmath::Matrix6d::Identity();
  geometric_to_analytic_jacobian_mapper.bottomRightCorner(3, 3) =
      geometric_to_analytic_orientation_jacobian_mapper;
  return geometric_to_analytic_jacobian_mapper;
}

}  // namespace kinematics
}  // namespace intrinsic
