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


#ifndef INTRINSIC_MATH_SPLINE_BSPLINE_BASE_H_
#define INTRINSIC_MATH_SPLINE_BSPLINE_BASE_H_

#include <cstddef>
#include <iterator>
#include <limits>
#include <valarray>
#include <vector>

#include "absl/log/check.h"
#include "intrinsic/icon/utils/log.h"

namespace intrinsic {

// Base class for b-spline implementations.
// This class contains methods for calculating basis functions and derivatives,
// that a derived class can use to implement a curve in a desired space.
// For details on B-Splines see "The Nurbs Book" by Les Piegl, Wayne Tiller,
// which this implementation closely follows.
//
// This class is thread-unsafe.
class BSplineBase {
 public:
  // Default tolerance for knot equality check.
  static constexpr double kDefaultKnotEqualityThreshold = 1.0e-10;

  BSplineBase() = default;
  ~BSplineBase() = default;
  BSplineBase(const BSplineBase&) = delete;
  BSplineBase& operator=(const BSplineBase&) = delete;

  // Initializes the B-spline with the given `degree` and `max_num_knots`. Not
  // real-time safe, will allocate memory. Returns true on success, false on
  // error.
  bool Init(size_t degree, size_t max_num_knots);

  // Get the degree of this curve.
  size_t Degree() const { return degree_; }

  // Get the number of knots in the knot vector.
  size_t NumKnots() const { return num_knots_; }

  // Get the number of control points for this curve.
  int NumPoints() const { return num_points_; }

  // Get the maximum number of points in the control polygon. Returns 0 if the
  // B-spline knot vector is not initialized.
  int MaxNumPoints() const;

  // Sets the knot vector. The provided `knots` must be a non-decreasing
  // sequence (multiple knots decrease continuity of the curve). The knot vector
  // `knots` should support [] subscript operator and have a size() function. It
  // is additionally possible to specify the minimum and maximum curve parameter
  // `umin` and `umax` (Default to smalles/largest value in `knots`). Returns
  // true on success, false on error.
  template <typename VecType>
  bool SetKnotVector(const VecType& knots,
                     double umin = std::numeric_limits<double>::quiet_NaN(),
                     double umax = std::numeric_limits<double>::quiet_NaN());

  // Sets a uniform knot vector of size `num_knots` for curve parameter in
  // [0,1]. Returns true on success, false on error.
  bool SetUniformKnotVector(size_t num_knots);

  // Gets the knot vector and stores it in `knots` (size must be correct).
  // Returns: false on error, true on success.
  template <typename VecType>
  bool GetKnotVector(VecType* knots) const;

  // Gets the unique knots of the spline within its valid domain and stores
  // them in `unique_knots`. Knots outside the valid domain (the first and
  // last `degree` knots for unclamped splines) and duplicate knots are removed.
  // Returns: false on error, true on success.
  bool GetValidUniqueKnots(
      std::vector<double>& unique_knots,
      double knot_equality_threshold = kDefaultKnotEqualityThreshold) const;

  // Returns the number of control points for given `num_knots` and `degree`.
  static constexpr ssize_t NumPoints(size_t num_knots, size_t degree) {
    return num_knots - degree - 1;
  }

  // Returns the number of knots for given control polygon and degree.
  static constexpr size_t NumKnots(size_t points, size_t degree) {
    return points + degree + 1;
  }

  // Returns the minimum number of points for a given degree.
  static constexpr size_t MinNumPoints(size_t degree) { return degree + 1; }

  // Returns the list of uniformly spaced knots for a given number `num_points`
  // of control points and `degree`.
  static bool MakeUniformKnotVector(int num_points, std::vector<double>* knots,
                                    size_t degree);
  // Find and return knot span index with non-vanishing basis functions.
  // Asserts if u is not inside min/max knot values.
  // u: function/curve parameter
  size_t KnotSpan(double u) const;

 protected:
  // Computes and returns non-vanishing basis functions \f$N_{i-p,p}(u), \dots,
  // N_{i,p}(u)\f$. This sets basis_[j]=N_{i-j,p}(u).
  // i: knot span index
  // p: degree of basis functions
  // u: curve/function parameter
  const std::valarray<double>& UpdateBasis(size_t knot_span_idx, size_t p,
                                           double u) const;

  // Computes and returns non-vanishing basis functions and derivatives
  // \f$\frac{partial^k N_{i-p,p}(u)}{partial u^k}, \dots \f$. This sets
  // basis_deriv_[j][k]=d^kN_{i-j,p}(u)/du^k.
  // i: knot span index p: degree of basis functions
  // u: curve/function parameter
  // der: highest derivative to calculate
  const std::valarray<double>& UpdateBasisAndDerivatives(size_t knot_span_idx,
                                                         size_t p, size_t der,
                                                         double u) const;

  bool init_ = false;
  size_t degree_ = 0;
  size_t num_knots_ = 0;
  size_t num_points_ = 0;
  std::valarray<double> knots_;
  double umin_ = 0;
  double umax_ = 0;

 private:
  // Mutable data buffer used for intermediate computation. The use of mutable
  // avoids memory allocation and enable use in real-time context.
  struct BasisData {
    std::valarray<double> basis;
    std::valarray<double> basis_ders;
    std::valarray<double> basis_du;
    std::valarray<double> a;
    std::valarray<double> tmp;
    std::valarray<double> left;
    std::valarray<double> right;

    void resize(size_t dim);
  };

  mutable BasisData data_;

  bool SetKnotVector(const double* knots, size_t num_knots, double umin,
                     double umax);
};

template <typename VecType>
bool BSplineBase::SetKnotVector(const VecType& knots, const double umin,
                                const double umax) {
  return SetKnotVector(&knots[0], knots.size(), umin, umax);
}

template <typename VecType>
bool BSplineBase::GetKnotVector(VecType* knots) const {
  CHECK(nullptr != knots);
  if (knots->size() != num_knots_) {
    INTRINSIC_RT_LOG(ERROR) << "Size error: knots.size()= " << knots->size()
                            << ", but num_knots= " << num_knots_;
    return false;
  }
  std::copy(std::begin(knots_), std::begin(knots_) + num_knots_, &(*knots)[0]);
  return true;
}

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_BSPLINE_BASE_H_
