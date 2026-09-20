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

#ifndef INTRINSIC_EIGENMATH_COMMON_NORMAL_BETWEEN_LINES_H_
#define INTRINSIC_EIGENMATH_COMMON_NORMAL_BETWEEN_LINES_H_

#include <algorithm>
#include <utility>

#include "intrinsic/eigenmath/types.h"

namespace intrinsic {
namespace eigenmath {

/* Computes the common normal between lines A (defined by a0 and a_dir) and B
 * (defined by b0 and b_dir). a_dir and b_dir do not need to be normalized.
 *
 * The common normal is returned as a pair of points, which are guaranteed to
 * lie on A and B, respectively. The distance between these points is the
 * shortest distance between the lines.
 *
 * If A and B intersect, the intersection point will be returned for both points
 * in the pair. If A and B are parallel, the first point of the pair will be set
 * to a0, and the second point will be closest point to a0 on B.
 *
 * @param a0 Point on line A.
 * @param a_dir Direction vector (does not need to be normalized) of line A.
 * @param a0 Point on line B.
 * @param b_dir Direction vector (does not need to be normalized) of line B.
 * @param epsilon If the angle in radians between a_dir and b_dir is within this
 *     epsilon of 0 or PI/2, the vectors will be considered parallel.
 * @return Pair of points on A and B respectively that define the common normal.
 */
template <class Scalar, int Options>
::std::pair<Vector<Scalar, Options>, Vector<Scalar, Options>>
CommonNormalBetweenLines(const Vector<Scalar, Options>& a0,
                         const Vector<Scalar, Options>& a_dir,
                         const Vector<Scalar, Options>& b0,
                         const Vector<Scalar, Options>& b_dir,
                         Scalar epsilon = static_cast<Scalar>(1e-4)) {
  // Consider the parameterized definitions of lines A and B, and let t_a and
  // t_b be the values that define the common normal. Thus:
  //
  //   [1] a0 + a_dir * t_a = 0
  //   [2] b0 + b_dir * t_b = 0
  //
  // That means the direction of the common normal is:
  //
  //   [3] cn_dir = b0 + b_dir * t_b - a0 - a_dir * t_a
  //
  // This direction will be perpendicular to both lines, so:
  //
  //   [4] cn_dir.dot(a_dir) = 0
  //   [5] cn_dir.dot(b_dir) = 0
  //
  // Substituting [3] into [4] and [5] lets us solve for t_a and t_b. For
  // compactness, let:
  //
  //   A = a_dir.dot(a_dir)
  //   B = b_dir.dot(b_dir)
  //   C = a_dir.dot(b_dir)
  //   D = a_dir.dot(b0 - a0)
  //   E = b_dir.dot(b0 - a0)
  //
  // Then:
  //
  //   [6] t_a = B * D - C * E / (A * B - C^2)
  //   [7] t_b = C * D - A * E / (A * B - C^2)
  Vector<Scalar, Options> a_to_b_dir = b0 - a0;
  Scalar a_norm = a_dir.norm();
  Scalar a_sqnorm = a_norm * a_norm;
  Scalar b_norm = b_dir.norm();
  Scalar b_sqnorm = b_norm * b_norm;
  Scalar a_dot_b = a_dir.dot(b_dir);
  Scalar a_dot_a_to_b = a_dir.dot(a_to_b_dir);
  Scalar denom = a_sqnorm * b_sqnorm - a_dot_b * a_dot_b;
  if (std::acos(std::clamp(std::abs(a_dot_b) / (a_norm * b_norm), -1.0, 1.0)) <=
      epsilon) {
    // Lines parallel -- set out_a to a0 and compute out_b by perpendicular
    // projection.
    return ::std::make_pair(Vector<Scalar, Options>(a0),
                            a0 + a_to_b_dir - a_dir * a_dot_a_to_b / a_sqnorm);
  }
  denom = 1.0 / denom;
  Scalar b_dot_a_to_b = b_dir.dot(a_to_b_dir);
  Scalar t_a = (b_sqnorm * a_dot_a_to_b - a_dot_b * b_dot_a_to_b) * denom;
  Scalar t_b = (a_dot_b * a_dot_a_to_b - a_sqnorm * b_dot_a_to_b) * denom;
  return ::std::make_pair(a0 + t_a * a_dir, b0 + t_b * b_dir);
}

}  // namespace eigenmath
}  // namespace intrinsic

#endif  // INTRINSIC_EIGENMATH_COMMON_NORMAL_BETWEEN_LINES_H_
