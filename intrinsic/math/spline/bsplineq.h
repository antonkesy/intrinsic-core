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


#ifndef INTRINSIC_MATH_SPLINE_BSPLINEQ_H_
#define INTRINSIC_MATH_SPLINE_BSPLINEQ_H_

#include <cstddef>
#include <vector>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/spline/bspline_base.h"

namespace intrinsic {

// Quaternion B-Spline curve implementation.
//
// This implementation uses cumulative B-Spline basis functions to
// define the quaternion B-Spline as described in
// 'A General Construction Scheme for Unit Quaternion Curves with Simple High
// Order Derivatives' by Kim, Kim & Shin.
//
// Limitations: derivative calculations are not yet implemented
//
// This class is thread-unsafe.
class BSplineQ : public BSplineBase {
 public:
  // Initialize the spline. Not real-time safe, will allocate memory.
  // degree: polynomial degree
  // max_num_knots: maximum length of the knot vector
  // return false on error, true on success
  bool Init(size_t degree, size_t max_num_knots);
  // Set control points. This function will check if the number of points is
  // consistent with the knot vector, so call BSplineBase::setKnotVector first.
  // points: quaternion control points
  // return true on success, false on error
  bool SetControlPoints(const std::vector<eigenmath::Quaterniond>& quats);
  // Evaluate b-spline curve
  // u: curve parameter
  // quat: curve at u
  // return true on success, false on error
  bool EvalCurve(double u, eigenmath::Quaterniond* quat) const;

  // Evaluate angular velocity dq/ds at u
  // param[in] u curve parameter
  // return angular velocity dq/ds
  double EvalAngularVelocity(double u);

 private:
  // Compute all cumulative basis functions Nc of degree p at u
  //    Nc_{0, p}(u) ... Nc_{n, p}(u)
  // This sets cumulative_basis_[j] = Nc_{j, p}(u)
  bool UpdateCumulativeBasis(size_t span, double u) const;

  std::vector<eigenmath::Quaterniond> points_;
  mutable std::vector<double> cumulative_basis_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_BSPLINEQ_H_
