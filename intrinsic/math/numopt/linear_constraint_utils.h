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

#ifndef INTRINSIC_MATH_NUMOPT_LINEAR_CONSTRAINT_UTILS_H_
#define INTRINSIC_MATH_NUMOPT_LINEAR_CONSTRAINT_UTILS_H_

#include "Eigen/Core"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Checks if the linear constraint normal vectors in `matrix_a` contain opposing
// axis-aligned bounds in all dimensions (e.g. bounding box bounds +/- `e_d` for
// all dimensions `d = 0, ..., point_dimension - 1`).
//
// When a constraint system contains opposing bounds along every coordinate axis
// (such as a bounding box), the normal vectors trivially span the full
// N-dimensional space, guaranteeing full column rank. This fast check allows
// bypassing an expensive QR rank decomposition.
//
// Geometric Alignment Test:
// For each constraint row normal `a_i`, we decompose it into its on-axis
// component `a_{i,d}` along coordinate axis `d` and its orthogonal off-axis
// component `a_{i,\perp}`:
//
//         ^ Axis d (e.g., `e_d`)
//         |
//  \      |      /
//   \    a_i    /  <- Allowed cone around axis
//    \    |    /     (||a_{i,\perp}||^2 <
//     \   |   /             max_off_axis_squared_ratio * ||a_i||^2)
//      \  |  /
//       \ | /
//        \|/
//  -------+-------------------------> Other axes
//
// Alignment is validated using a relative threshold:
// `max_off_axis_squared_ratio` (e.g., `kDefaultMaxOffAxisSquaredRatio`): Bounds
// the relative squared magnitude of all orthogonal components:
//   `||a_{i,\perp}||^2 / ||a_i||^2 = (||a_i||^2 - a_{i,d}^2) / ||a_i||^2 <
//   max_off_axis_squared_ratio`
// which is equivalent to:
//   `||a_{i,\perp}||^2 < max_off_axis_squared_ratio * ||a_i||^2`
// Using a relative ratio makes the check independent of vector scaling. It
// ensures the normal vector is parallel to axis `d` within an angular tolerance
// of:
//   theta = arcsin(sqrt(max_off_axis_squared_ratio))
// (for default `kDefaultMaxOffAxisSquaredRatio`, theta = arcsin(0.01) ~= 0.57
// deg / 0.01 rad). Reasonable values range from 1.0e-6 to 1.0e-3 to absorb
// numerical roundoff from upstream coordinate transforms while rejecting
// tilted/diagonal faces.
//
// Default threshold for detecting axis-aligned normal vectors.
constexpr double kDefaultMaxOffAxisSquaredRatio = 1.0e-4;

// Returns true if opposing axis-aligned bounds are present for all dimensions
// (1D, 2D, or 3D), false otherwise.
template <typename DerivedA>
[[nodiscard]] bool HasOpposingBoundsOnAllAxes(
    const Eigen::MatrixBase<DerivedA>& matrix_a,
    const double max_off_axis_squared_ratio = kDefaultMaxOffAxisSquaredRatio) {
  const Eigen::Index point_dimension = matrix_a.cols();
  const Eigen::Index num_constraints = matrix_a.rows();

  // Fast check is only applied for low dimensions (1D to 3D) and when there
  // are at least `2 * point_dimension` constraints (at least one positive and
  // negative bound per axis).
  if (point_dimension < 1 || point_dimension > 3 ||
      num_constraints < 2 * point_dimension) {
    return false;
  }

  const uint32_t target_mask = (1 << point_dimension) - 1;

  uint32_t pos_axes_mask = 0;
  uint32_t neg_axes_mask = 0;
  for (Eigen::Index i = 0; i < num_constraints; ++i) {
    const double sq_norm = matrix_a.row(i).squaredNorm();
    if (sq_norm <= 0.0) {
      continue;
    }

    for (Eigen::Index d = 0; d < point_dimension; ++d) {
      const double val = matrix_a(i, d);
      const double off_axis_sq_norm = sq_norm - val * val;
      if (off_axis_sq_norm < max_off_axis_squared_ratio * sq_norm) {
        if (val > 0.0) {
          pos_axes_mask |= (1 << d);
        } else {
          neg_axes_mask |= (1 << d);
        }
        break;
      }
    }
    if (pos_axes_mask == target_mask && neg_axes_mask == target_mask) {
      return true;
    }
  }

  return false;
}

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_NUMOPT_LINEAR_CONSTRAINT_UTILS_H_
