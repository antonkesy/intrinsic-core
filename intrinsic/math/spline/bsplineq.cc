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


#include "intrinsic/math/spline/bsplineq.h"

#include <math.h>

#include <algorithm>
#include <cstddef>
#include <vector>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/math/spline/bspline_base.h"

namespace intrinsic {

static constexpr double kEpsilon = 1e-12;

// TODO(elir) duplicated from intrinsic/utils/eigen - move to shared/?
// b/34073179
eigenmath::Quaterniond QuatLog(const eigenmath::Quaterniond& quat_in) {
  double r = sqrt(quat_in.x() * quat_in.x() + quat_in.y() * quat_in.y() +
                  quat_in.z() * quat_in.z());

  double t = 0.0;
  if (r > kEpsilon) {
    t = atan2(r, quat_in.w()) / r;
  }

  eigenmath::Quaterniond quat_out(quat_in);
  // Note: avoid calculation of log(x) for certain values of x close to 1 as
  //   those calculations can take up to 4 usec on the rtpc. b/37896233
  double norm = quat_in.squaredNorm();
  quat_out.w() = fabs(1 - norm) < kEpsilon ? 0 : 0.5 * log(norm);
  quat_out.x() *= t;
  quat_out.y() *= t;
  quat_out.z() *= t;

  return quat_out;
}

eigenmath::Quaterniond QuatExp(const eigenmath::Quaterniond& quat_in) {
  double r = sqrt(quat_in.x() * quat_in.x() + quat_in.y() * quat_in.y() +
                  quat_in.z() * quat_in.z());
  // Note: avoid calculation of exp(x) for certain small values of x as those
  //   calculations can take up to 12 usec on the rtpc. b/37896233
  double et = fabs(quat_in.w()) < kEpsilon ? 1 : exp(quat_in.w());

  double s = 0.0;
  if (r > kEpsilon) {
    s = et * sin(r) / r;
  }

  eigenmath::Quaterniond quat_out(quat_in);

  quat_out.w() = et * cos(r);
  quat_out.x() *= s;
  quat_out.y() *= s;
  quat_out.z() *= s;

  return quat_out;
}

eigenmath::Quaterniond QuatPower(const eigenmath::Quaterniond& quat,
                                 const double power) {
  eigenmath::Quaterniond quat_in(quat);

  // Transform quaternion to canonical form
  if (quat_in.w() < 0) {
    quat_in.w() *= -1;
    quat_in.x() *= -1;
    quat_in.y() *= -1;
    quat_in.z() *= -1;
  }

  if (fabs(quat_in.norm() - 1) > kEpsilon) {
    INTRINSIC_RT_LOG(WARNING)
        << "Normalizing quaternion before executing "
        << "log function. " << quat_in.w() << " " << quat_in.x() << " "
        << quat_in.y() << " " << quat_in.z();
    quat_in.normalize();
  }

  // q^p = exp(p * log(q))
  eigenmath::Quaterniond quat_out = QuatLog(quat_in);

  quat_out.w() *= power;
  quat_out.x() *= power;
  quat_out.y() *= power;
  quat_out.z() *= power;

  quat_out = QuatExp(quat_out);

  return quat_out;
}

bool BSplineQ::Init(size_t degree, size_t max_num_knots) {
  init_ = false;
  if (!BSplineBase::Init(degree, max_num_knots)) {
    return false;
  }

  num_points_ = NumPoints(max_num_knots, degree);

  points_.resize(num_points_);
  cumulative_basis_.resize(num_points_, 0.0);

  init_ = true;
  return true;
}

bool BSplineQ::SetControlPoints(
    const std::vector<eigenmath::Quaterniond>& points) {
  if (!init_) {
    INTRINSIC_RT_LOG(ERROR) << "Call init first.";
    return false;
  }
  if (points.size() > points_.capacity()) {
    INTRINSIC_RT_LOG(ERROR) << "Too many points: got " << points.size()
                            << " points, but capacity= " << points_.capacity();
    return false;
  }
  if (points.size() != NumPoints(num_knots_, degree_)) {
    INTRINSIC_RT_LOG(ERROR)
        << "Number of points inconsistent with knot vector and degree "
           " num points = "
        << points.size() << ", but knots= " << num_knots_
        << ", degree= " << degree_ << ", so points should be "
        << NumPoints(num_knots_, degree_);
    return false;
  }
  num_points_ = points.size();
  points_ = points;
  return true;
}

bool BSplineQ::EvalCurve(double u, eigenmath::Quaterniond* quat) const {
  if (!init_) {
    INTRINSIC_RT_LOG(ERROR) << "Call init first.";
    return false;
  }
  if (u < umin_ || u > umax_) {
    INTRINSIC_RT_LOG(ERROR)
        << "Parameter out of range (u= " << u << ", umin= " << umin_
        << ", umax= " << umax_ << ")";
    return false;
  }

  int span_index = KnotSpan(u);
  UpdateCumulativeBasis(span_index, u);

  *quat = QuatPower(points_[0], cumulative_basis_[0]);
  for (size_t i = 1; i < num_points_; i++) {
    *quat *=
        QuatPower(points_[i - 1].inverse() * points_[i], cumulative_basis_[i]);
  }

  return true;
}

double BSplineQ::EvalAngularVelocity(double u) {
  // TODO(elir) calculate analytical derivatives b/34102244
  // Calculate angular velocity omega as finite differences using the angular
  // distance between the two quaternions as difference
  double du = 1e-6;

  eigenmath::Quaterniond quat_left, quat_right;

  double u_left = std::max(umin_, u - du);
  double u_right = std::min(umax_, u + du);

  EvalCurve(u_left, &quat_left);
  EvalCurve(u_right, &quat_right);

  return quat_left.angularDistance(quat_right) / (u_right - u_left);
}

bool BSplineQ::UpdateCumulativeBasis(size_t span_index, double u) const {
  // The cumulative basis functions Bc are non-constant only inside the bounds
  // knots_[span_index] and knots_[span_index + degree].
  // Inside those bounds, the functions are calculated as the sum of all
  // non-zero normal basis functions at the given u value.
  // Outside those bounds, the values are as follows:
  //    Bc = 0   for u <= knots_[span_index]
  //    Bc = 1   for u >= knots_[span_index + degree]

  const auto& basis = UpdateBasis(span_index, degree_, u);

  for (size_t j = 0; j < num_points_; j++) {
    if (u < knots_[j]) {
      cumulative_basis_[j] = 0.0;
    } else if (u >= knots_[j + degree_]) {
      cumulative_basis_[j] = 1.0;
    } else {
      cumulative_basis_[j] = 0.0;
      for (size_t k = degree_ - span_index + j; k <= degree_; k++) {
        cumulative_basis_[j] += basis[k];
      }
    }
  }
  return true;
}

}  // namespace intrinsic
