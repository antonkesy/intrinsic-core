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


#include "intrinsic/math/spline/bspline_base.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <utility>
#include <valarray>
#include <vector>

#include "absl/log/check.h"
#include "absl/strings/str_format.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/math/almost_equals.h"

namespace intrinsic {

constexpr size_t kMaxDegree = 128;
constexpr size_t kMaxNumKnots = 100 * 1024;
constexpr size_t kMinNumKnots = 3;

void BSplineBase::BasisData::resize(size_t dim) {
  basis.resize(dim);
  basis = 0.0;

  left.resize(dim);
  left = 0.0;
  right.resize(dim);
  right = 0.0;

  basis_ders.resize((dim) * (dim));

  basis_du.resize((dim) * (dim));
  basis_du = 0.0;

  a.resize(2 * (dim));
  a = 0.0;

  tmp.resize(dim);
  tmp = 0.0;
}

bool BSplineBase::Init(size_t degree, size_t max_num_knots) {
  if (max_num_knots < kMinNumKnots || max_num_knots > kMaxNumKnots) {
    INTRINSIC_RT_LOG(ERROR)
        << "Invalid input: max_num_knots= " << max_num_knots << ", must be in ["
        << kMinNumKnots << ", " << kMaxNumKnots << "]";
    return false;
  }

  if (degree > kMaxDegree) {
    INTRINSIC_RT_LOG(ERROR) << "Invalid input: degree= " << degree
                            << ", should be <= " << kMaxDegree;
    return false;
  }

  // p = degree
  // m+1: number of knots (0,.., m) = num_knots
  // n+1: number of basis functions/points,
  //      n+1 = m-p-1, n=m-p-2
  //      ==> num_points_ = (num_knots-1)-degree-1 num_knots-degree-2
  degree_ = degree;

  num_knots_ = 0;
  num_points_ = 0;
  if (NumPoints(max_num_knots, degree) < 1) {
    INTRINSIC_RT_LOG(ERROR)
        << "Invalid parameters: max_num_knots= " << max_num_knots
        << ", degree= " << degree
        << ", so num_points= " << NumPoints(max_num_knots, degree_)
        << ". Need num_points >=1";
    return false;
  }

  data_.resize(degree + 1);

  knots_.resize(max_num_knots);
  knots_ = 0.0;

  return true;
}

int BSplineBase::MaxNumPoints() const {
  // If the knot vector is empty (e.g. B-spline not initialized),
  // NumPoints(knots_.size(), degree_) can be negative.
  return std::max(0, static_cast<int>(NumPoints(knots_.size(), degree_)));
}

bool BSplineBase::SetUniformKnotVector(size_t num_knots) {
  umin_ = 0;
  umax_ = 1.0;

  if (num_knots > knots_.size()) {
    INTRINSIC_RT_LOG(ERROR)
        << "Knot vector too large, input dimension " << num_knots
        << ", but max length= " << knots_.size();
    return false;
  }
  const size_t min_num_knots = (degree_ + 1) * 2;
  if (num_knots < min_num_knots) {
    INTRINSIC_RT_LOG(ERROR)
        << "Knot vector must have at least " << min_num_knots
        << " elements for spline of degree " << degree_ << ", but got "
        << num_knots << " element.";
    return false;
  }

  knots_[std::slice(0, degree_ + 1, 1)] = 0.0;
  knots_[std::slice(num_knots - (degree_ + 1), degree_ + 1, 1)] = 1.0;
  double spacing = 1.0 / (num_knots - 2 * (degree_ + 1) + 1);
  for (int idx = degree_ + 1; idx < num_knots - degree_ - 1; idx++) {
    knots_[idx] = knots_[idx - 1] + spacing;
  }
  num_knots_ = num_knots;

  return true;
}

bool BSplineBase::GetValidUniqueKnots(
    std::vector<double>& unique_knots,
    const double knot_equality_threshold) const {
  const size_t min_num_knots = (degree_ + 1) * 2;
  if (num_knots_ < min_num_knots) {
    INTRINSIC_RT_LOG(ERROR)
        << "Knot vector must have at least " << min_num_knots << " knots. Got "
        << num_knots_ << ".";
    return false;
  }
  if (knots_.size() < num_knots_) {
    INTRINSIC_RT_LOG(ERROR)
        << "Got inconsistent num_knots_ > knots_.size(): " << num_knots_
        << " > " << knots_.size() << ".";
    return false;
  }
  if (knot_equality_threshold < 0.0) {
    INTRINSIC_RT_LOG(ERROR)
        << "Knot equality threshold must be non-negative. Got "
        << knot_equality_threshold << ".";
    return false;
  }

  // Retrieve the knots within the valid domain [degree_, num_knots_ - degree_)
  // and remove duplicates up to a certain precision. We assume that the number
  // of duplicates is small, so the overhead of the vector operations remains
  // low.
  unique_knots.assign(std::begin(knots_) + degree_,
                      std::begin(knots_) + (num_knots_ - degree_));
  unique_knots.erase(std::unique(unique_knots.begin(), unique_knots.end(),
                                 [knot_equality_threshold](double x, double y) {
                                   return intrinsic::AlmostEquals(
                                       x, y, knot_equality_threshold);
                                 }),
                     unique_knots.end());

  return true;
}

bool BSplineBase::SetKnotVector(const double* knots, const size_t num_knots,
                                const double umin, const double umax) {
  if (num_knots > knots_.size()) {
    INTRINSIC_RT_LOG(ERROR)
        << "Knot vector too large, input dimension " << num_knots
        << ", but max length= " << knots_.size();
    return false;
  }
  const size_t min_num_knots = (degree_ + 1) * 2;
  if (num_knots < min_num_knots) {
    INTRINSIC_RT_LOG(ERROR)
        << "Knot vector must have at least " << min_num_knots
        << " elements for spline of degree " << degree_ << ", but got "
        << num_knots << " element.";
    return false;
  }

  // verify non-decreasing knots
  for (size_t i = 1; i < num_knots; i++) {
    if (knots[i - 1] > knots[i]) {
      INTRINSIC_RT_LOG(ERROR)
          << "Knots must be non-decreasing, but knot[" << (i - 1)
          << "]= " << knots[i - 1] << " and knot[" << i << "]= " << knots[i];
      return false;
    }
  }

  num_knots_ = num_knots;
  // knots_[std::slice(0, num_knots_, 1)] = knots;
  std::copy(knots, knots + num_knots_, std::begin(knots_));
  num_points_ = NumPoints(num_knots_, degree_);

  if (!std::isfinite(umin)) {
    umin_ = knots[0];
  } else {
    if (umin >= knots[0]) {
      umin_ = umin;
    } else {
      INTRINSIC_RT_LOG(WARNING) << "Got umin<knots[0], setting to knots[0]";
      umin_ = knots[0];
    }
  }
  if (!std::isfinite(umax)) {
    umax_ = knots[num_knots - 1];
  } else {
    if (umax <= knots[num_knots - 1]) {
      umax_ = umax;
    } else {
      umax_ = knots[num_knots - 1];
      INTRINSIC_RT_LOG(WARNING)
          << "Got umax > knots[num_knots - 1], setting to knots[num_knots - 1]";
    }
  }

  return true;
}

// This is alg. 2.1 from the NURBS book
// Implements a binary search for the knot span index i in which u lies:
// u \in [knots_[i], knots_[i+1])
size_t BSplineBase::KnotSpan(double u) const {
  // spline not initialized ==> return 0;
  if (num_knots_ == 0) {
    return 0;
  }
  // special case (upper bound): return last span to avoid special case in other
  // functions.
  // Return largest possible knot span index.
  // The spline values are computed by
  // \sum{basis[i] * points_[span - degree + i], i={0 ... degree}},
  // so the largest value is num_points_
  if (u >= knots_[num_points_]) {
    return num_points_ - 1;
  }

  // assert u in min/max knot range.
  CHECK_GE(u, knots_[0]) << absl::StrFormat("u: %.18e; knots_[0]= %.18e", u,
                                            knots_[0]);
  CHECK_LE(u, knots_[num_knots_ - 1])
      << absl::StrFormat("u: %.18e; knots_[%zu]= %.18e", u, num_knots_ - 1,
                         knots_[num_knots_ - 1]);

  // this is the lower bound (see equation for spline above)
  size_t low = degree_;
  size_t high = num_points_;
  size_t mid = (low + high) / 2;
  // binary search for interval
  while ((u < knots_[mid] || u >= knots_[mid + 1]) && low != high) {
    if (u < knots_[mid]) {
      high = mid;
    } else {
      low = mid;
    }
    mid = (low + high) / 2;
  }
  return mid;
}

//  this is alg. 2.2 from the NURBS book
const std::valarray<double>& BSplineBase::UpdateBasis(size_t knot_span_idx,
                                                      size_t p,
                                                      double u) const {
  data_.basis = 0.0;
  data_.basis[0] = 1.0;
  for (size_t j = 1; j <= p; j++) {
    data_.left[j] = u - knots_[knot_span_idx + 1 - j];
    data_.right[j] = knots_[knot_span_idx + j] - u;
    double saved = 0.0;
    for (size_t r = 0; r < j; r++) {
      double tmp = data_.basis[r] / (data_.right[r + 1] + data_.left[j - r]);
      data_.basis[r] = saved + data_.right[r + 1] * tmp;
      saved = data_.left[j - r] * tmp;
    }
    data_.basis[j] = saved;
  }
  return data_.basis;
}

//  corresponds to algorithm 2.3 from the NURBS book
const std::valarray<double>& BSplineBase::UpdateBasisAndDerivatives(
    size_t knot_span_idx, size_t p, size_t der, double u) const {
  CHECK_LE(der, p);
  CHECK_LE(p, degree_);
  const size_t N = degree_ + 1;
  data_.basis_ders = 0.0;
  data_.basis_du = 0.0;
  data_.basis_du[0 * N + 0] = 1.0;

  for (size_t j = 1; j <= p; j++) {
    data_.left[j] = u - knots_[knot_span_idx + 1 - j];
    data_.right[j] = knots_[knot_span_idx + j] - u;
    double saved = 0.0;
    for (size_t r = 0; r < j; r++) {
      // cache knot differences in lower triangle of basis_du_
      data_.basis_du[N * j + r] = data_.right[r + 1] + data_.left[j - r];
      double tmp = data_.basis_du[N * r + j - 1] / data_.basis_du[N * j + r];
      // cache basis function in upper triangle of basis_du_
      data_.basis_du[N * r + j] = saved + data_.right[r + 1] * tmp;
      saved = data_.left[j - r] * tmp;
    }
    data_.basis_du[N * j + j] = saved;
  }
  // copy basis functions to first row of basis_ders_
  data_.basis_ders[std::slice(0, p + 1, 1)] =
      data_.basis_du[std::slice(p, p + 1, N)];

  // compute derivatives
  int j1, j2;
  for (int r = 0; r <= p; r++) {
    int s1 = 0, s2 = 1;
    data_.a[0 * N + 0] = 1.0;
    // kth derivative
    for (int k = 1; k <= der; k++) {
      double d = 0.0;
      int rk = r - k;
      int pk = p - k;
      if (r >= k) {
        data_.a[N * s2 + 0] =
            data_.a[N * s1 + 0] / data_.basis_du[N * (pk + 1) + rk];
        d = data_.a[N * s2 + 0] * data_.basis_du[N * rk + pk];
      }
      if (rk >= -1) {
        j1 = 1;
      } else {
        j1 = -rk;
      }
      if (r - 1 <= pk) {
        j2 = k - 1;
      } else {
        j2 = p - r;
      }
      for (size_t j = j1; j <= j2; j++) {
        data_.a[N * s2 + j] = (data_.a[N * s1 + j] - data_.a[N * s1 + j - 1]) /
                              data_.basis_du[N * (pk + 1) + rk + j];
        d += data_.a[N * s2 + j] * data_.basis_du[N * (rk + j) + pk];
      }
      if (r <= pk) {
        data_.a[N * s2 + k] =
            -data_.a[N * s1 + k - 1] / data_.basis_du[N * (pk + 1) + r];
        d += data_.a[N * s2 + k] * data_.basis_du[N * r + pk];
        CHECK(std::isfinite(d));
      }
      data_.basis_ders[N * k + r] = d;
      CHECK(std::isfinite(d));
      // switch rows
      std::swap(s1, s2);
    }
  }

  // multiply by factors
  data_.tmp = p;
  for (size_t k = 1; k <= der; k++) {
    data_.basis_ders[std::slice(N * k, p + 1, 1)] *= data_.tmp;
    data_.tmp *= (p - k);
  }

  return data_.basis_ders;
}

double UniformKnotSpacing(int num_knots, size_t degree) {
  const double denom = num_knots - 2.0 * (degree + 1.0) + 1.0;
  CHECK_GT(denom, 0.0);
  return 1.0 / denom;
}

bool BSplineBase::MakeUniformKnotVector(int num_points,
                                        std::vector<double>* knots,
                                        size_t degree) {
  if (num_points < MinNumPoints(degree)) {
    INTRINSIC_RT_LOG(ERROR)
        << "Too few points: need >= " << MinNumPoints(degree) << ", but got "
        << num_points;
    return false;
  }
  const auto nknots = NumKnots(num_points, degree);
  if (nknots > knots->capacity()) {
    INTRINSIC_RT_LOG(ERROR) << "Too many knots: nknots= " << nknots
                            << ", but capacity= " << knots->capacity();
    return false;
  }
  knots->resize(nknots);
  std::fill(knots->begin(), knots->begin() + degree + 1, 0.0);
  const double spacing = UniformKnotSpacing(nknots, degree);
  std::fill(knots->end() - degree - 1, knots->end(), 1.0);
  for (size_t idx = degree + 1; idx < nknots - degree - 1; idx++) {
    (*knots)[idx] = (*knots)[idx - 1] + spacing;
  }
  return true;
}

}  // namespace intrinsic
