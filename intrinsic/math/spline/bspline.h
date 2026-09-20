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


#ifndef INTRINSIC_MATH_SPLINE_BSPLINE_H_
#define INTRINSIC_MATH_SPLINE_BSPLINE_H_

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/math/proto/bspline.pb.h"
#include "intrinsic/math/spline/bspline_base.h"

namespace intrinsic {

// B-Spline curve implementation.
// The dimension of the space the curve is in can be configured using the
// Traits
// template argument.
// It is expected to define:
// - Point type supporting [], +, *, += operations.
// - a SizeOk function to check point dimensions.
// - a Size() function to query point size.
// - a Zero(Point) function to initialize points.
// - a Dot(Point, Point) function to compute dot product of two points.
// - a kPointDim element defining point dimensions (only for fixed-size types).
//
// The order of the spline is also configurable.
//
// BSplines have some desirable advantages, such as
// - Local support
// - C^k smoothness, where k = degree - 1.
// - Coordinate system invariant.
// - The curve transforms with its control points.
//
// Note, however, that control points are not interpolated.
// For details on B-Splines see "The Nurbs Book" by Les Piegl, Wayne Tiller,
// which this implementation closely follows.
template <typename Traits>
class BSplineT : public BSplineBase {
 public:
  using Point = typename Traits::Point;

  // The dimension of the spline control points as defined by the spline Traits.
  // TODO(b/351978904) Note that this only exists for fixed-size types.
  static constexpr int kPointDim = Traits::kPointDim;

  BSplineT() = default;
  ~BSplineT() = default;

  // Returns the dimension of the spline control points, once set.
  size_t PointDim() const { return points_dim_; }

  // Initializes the B-spline curve with the given `degree`, setting
  // `max_num_knots` as the maximum length of the knot vector and `points_dim`
  // as dimension of the space the curve is in. Not real-time safe, will
  // allocate memory. Returns true on success, false on error.
  bool Init(size_t degree, size_t max_num_knots,
            size_t points_dim = Traits::kPointDim);

  // Sets the control `points` of the B-spline. This function will check if the
  // number of points is consistent with the knot vector, so call
  // BSplineBase::setKnotVector first. Returns true on success, false on error.
  bool SetControlPoints(absl::Span<const Point> points);

  // Evaluates the B-spline curve at the given curve parameter `u`, storing the
  // result in `value`. Returns true on success, false on error.
  bool EvalCurve(double u, Point* value) const;

  // Evaluates the B-spline curve and derivatives at the given curve parameter
  // `u`. The computed values are stored in `values`. The length of `values`
  // determines the number of derivatives to calculate. Returns true on success,
  // false on error.
  bool EvalCurveAndDerivatives(double u, std::vector<Point>* values) const;

  // Evaluates B-spline curve and derivatives up to `num_derivatives` at all
  // (non-duplicate) knots. Duplicate knot values are ignored during evaluation.
  // If `num_derivatives` is set to zero, only the curve value is returned.
  // Stores results in a nested vector of vectors of Points, where the inner
  // vector holds evaluated curve and derivatives, and the outer vector holds
  // the evaluations at knots. Stores the evaluated knots in `values_at_knots`.
  // Warning: this function is not realtime-compatible.
  // Returns: true on success, false on error
  bool EvalCurveAndDerivativesAtKnots(
      int num_derivatives, std::vector<std::vector<Point>>& values_at_knots,
      std::vector<double>& unique_knots) const;

  // Returns the control point vector and stores it in `points` (size
  // must be correct). Returns: false on error, true on success.
  bool GetControlPoints(std::vector<Point>* points) const;

 private:
  int points_dim_ = 0;
  std::vector<Point> points_;
};

template <typename Traits>
absl::StatusOr<std::unique_ptr<BSplineT<Traits>>> FromProto(
    const intrinsic_proto::BSpline& spline_proto);

template <typename Traits>
intrinsic_proto::BSpline ToProto(const BSplineT<Traits>& spline);

template <typename Traits>
bool BSplineT<Traits>::Init(size_t degree, size_t max_num_knots,
                            size_t points_dim) {
  init_ = false;
  if (!BSplineBase::Init(degree, max_num_knots)) {
    return false;
  }

  points_.reserve(NumPoints(max_num_knots, degree));

  if (!Traits::SizeOk(points_dim)) {
    INTRINSIC_RT_LOG(ERROR) << "Points size not valid (" << points_dim << ")";
    return false;
  }
  points_dim_ = points_dim;
  init_ = true;
  return true;
}

template <typename Traits>
bool BSplineT<Traits>::SetControlPoints(absl::Span<const Point> points) {
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
  points_.resize(points.size());
  num_points_ = points.size();
  for (size_t i = 0; i < num_points_; i++) {
    if (Traits::Size(points[i]) != points_dim_) {
      INTRINSIC_RT_LOG(ERROR)
          << "size(points[" << i << "])= " << Traits::Size(points[i])
          << ", but curve needs size= " << points_dim_;
      return false;
    }
  }
  points_.assign(points.begin(), points.end());
  return true;
}

// Corresponds to algorithm 3.1 in NURBS Book.
template <typename Traits>
bool BSplineT<Traits>::EvalCurve(double u, Point* value) const {
  CHECK(nullptr != value);

  if (Traits::Size(*value) != points_dim_) {
    INTRINSIC_RT_LOG(ERROR) << "size(value)= " << Traits::Size(*value)
                            << ", but curve needs size= " << points_dim_;
    return false;
  }

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

  size_t span = KnotSpan(u);
  const auto& basis = UpdateBasis(span, degree_, u);
  Traits::Zero(*value);
  for (int i = 0; i <= degree_; i++) {
#ifdef DEBUG
    CHECK_LT(span - degree_ + i, points_.size()) << absl::StrFormat(
        "span= %d, degree_= %zd, i= %d, num_points_= %zd; u= %e", span, degree_,
        i, num_points_, u);
#endif
    *value += basis[i] * points_[span - degree_ + i];
  }
  return true;
}

// Corresponds to algorithm 3.2 in NURBS Book.
template <typename Traits>
bool BSplineT<Traits>::EvalCurveAndDerivatives(
    double u, std::vector<Point>* values) const {
  CHECK(nullptr != values);

  if (values->empty()) {
    INTRINSIC_RT_LOG(ERROR) << "Vector of values must be >=1";
    return false;
  }
  for (int i = 0; i < values->size(); ++i) {
    if (Traits::Size((*values)[i]) != points_dim_) {
      INTRINSIC_RT_LOG(ERROR)
          << "size(values[" << i << "])= " << Traits::Size((*values)[i])
          << ", but curve needs size= " << points_dim_;
      return false;
    }
  }

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

  const size_t der = values->size() - 1;
  for (size_t k = degree_ + 1; k < der; k++) {
    Traits::Zero((*values)[k]);
  }
  size_t du = std::min(der, degree_);
  size_t span = KnotSpan(u);
  const auto& basis_ders = UpdateBasisAndDerivatives(span, degree_, der, u);
  for (int k = 0; k <= du; k++) {
    Traits::Zero((*values)[k]);
    for (int j = 0; j <= degree_; j++) {
      (*values)[k] +=
          basis_ders[k * (degree_ + 1) + j] * points_[span - degree_ + j];
    }
  }
  return true;
}

template <typename Traits>
bool BSplineT<Traits>::EvalCurveAndDerivativesAtKnots(
    int num_derivatives, std::vector<std::vector<Point>>& values_at_knots,
    std::vector<double>& unique_knots) const {
  if (!GetValidUniqueKnots(unique_knots)) {
    INTRINSIC_RT_LOG(ERROR) << "Failed to get unique knots vector.";
    return false;
  }

  const int num_unique_knots = unique_knots.size();
  values_at_knots.resize(num_unique_knots,
                         std::vector<Point>(num_derivatives + 1));
  for (int i = 0; i < num_unique_knots; ++i) {
    values_at_knots[i].resize(num_derivatives + 1);
    if (!EvalCurveAndDerivatives(unique_knots[i], &values_at_knots[i])) {
      INTRINSIC_RT_LOG(ERROR)
          << "EvalCurveAndDerivatives failed for unique knot "
          << unique_knots[i];
      return false;
    }
  }
  return true;
}

template <typename Traits>
bool BSplineT<Traits>::GetControlPoints(std::vector<Point>* points) const {
  CHECK(nullptr != points);

  if (points->size() != points_.size()) {
    INTRINSIC_RT_LOG(ERROR) << "Size error: points.size()=" << points->size()
                            << ", but num_points=" << points_.size();
    return false;
  }

  *points = points_;
  return true;
}

// Implementation of FromProto() for B-spline.
template <typename Traits>
absl::StatusOr<std::unique_ptr<BSplineT<Traits>>> FromProto(
    const intrinsic_proto::BSpline& spline_proto) {
  if (spline_proto.control_points().empty()) {
    return absl::InvalidArgumentError(
        "B-spline proto must have at least one control point.");
  }
  const int point_dim = spline_proto.control_points(0).point_size();
  auto spline = std::make_unique<BSplineT<Traits>>();
  if (!spline->Init(spline_proto.degree(), spline_proto.knots_size(),
                    point_dim)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Couldn't initialize B-spline from proto with degree ",
        spline_proto.degree(), " and number of knots ",
        spline_proto.knots_size(), " and point dimension ", point_dim, "."));
  }

  if (!spline->SetKnotVector(std::vector<double>(
          {spline_proto.knots().begin(), spline_proto.knots().end()}))) {
    return absl::InvalidArgumentError(
        "Couldn't set knot vector for B-spline from proto.");
  }

  std::vector<typename Traits::Point> control_points;
  control_points.reserve(spline_proto.control_points_size());
  for (const auto& control_point_proto : spline_proto.control_points()) {
    typename Traits::Point point;
    point.resize(point_dim);
    for (int id = 0; id < point_dim; ++id) {
      if (point_dim != control_point_proto.point_size()) {
        return absl::InvalidArgumentError(
            absl::StrCat("The control point dimension is not consistent. Got ",
                         control_point_proto.point_size(), " and expected was ",
                         point_dim, "."));
      }
      point[id] = control_point_proto.point(id);
    }
    control_points.emplace_back(point);
  }
  if (!spline->SetControlPoints(control_points)) {
    return absl::InvalidArgumentError(
        "Couldn't set control points for B-spline from proto.");
  }

  return spline;
}

// Implementation of ToProto() for B-spline.
template <typename Traits>
intrinsic_proto::BSpline ToProto(const BSplineT<Traits>& spline) {
  intrinsic_proto::BSpline spline_proto;
  spline_proto.set_degree(spline.Degree());

  std::vector<double> knots(spline.NumKnots());
  spline.GetKnotVector(&knots);
  spline_proto.mutable_knots()->Add(knots.begin(), knots.end());

  std::vector<typename Traits::Point> control_points(spline.NumPoints());
  spline.GetControlPoints(&control_points);
  const int point_dim = control_points.front().size();
  spline_proto.set_control_point_size(point_dim);
  for (int ctrl_id = 0; ctrl_id < spline.NumPoints(); ++ctrl_id) {
    auto* control_point_proto = spline_proto.add_control_points();
    for (int id = 0; id < point_dim; ++id)
      control_point_proto->add_point(control_points[ctrl_id][id]);
  }

  return spline_proto;
}

// Implementations for commonly used point types.

// Traits for scalar b-spline, using double "points".
struct SplineTraits1d {
  using Point = eigenmath::Vectord<1, Eigen::DontAlign>;
  static constexpr int kPointDim = 1;
  static constexpr bool SizeOk(const int size) { return size == 1; }
  static constexpr int Size(const Point& point) { return 1; }
  static void Zero(Point& point) { point.setZero(); }
  static double Dot(const Point& a, const Point& b) { return a.dot(b); }
};
// B-spline using points of type double.
using BSpline1d = BSplineT<SplineTraits1d>;

// Traits for B-spline in R^2, using Vector2d points.
struct SplineTraits2d {
  using Point = eigenmath::Vector2d;
  static constexpr int kPointDim = 2;
  static constexpr bool SizeOk(const int size) { return size == 2; }
  static constexpr int Size(const Point& point) { return 2; }
  static void Zero(Point& point) { point.setZero(); }
  static double Dot(const Point& a, const Point& b) { return a.dot(b); }
};
// B-spline using Vector2d points
using BSpline2d = BSplineT<SplineTraits2d>;

// Traits for b-spline in R^3, using Vector3d points
struct SplineTraits3d {
  using Point = eigenmath::Vector3d;
  static constexpr int kPointDim = 3;
  static constexpr bool SizeOk(const int size) { return size == 3; }
  static constexpr int Size(const Point& point) { return 3; }
  static void Zero(Point& point) { point.setZero(); }
  static double Dot(const Point& a, const Point& b) { return a.dot(b); }
};
// B-spline using Vector3d points.
using BSpline3d = BSplineT<SplineTraits3d>;

// Traits for b-spline in R^n, using VectorNd points.
template <int N = eigenmath::MAX_EIGEN_VECTOR_SIZE>
struct SplineTraitsNd {
  using Point = eigenmath::VectorNdWithMaxSize<N>;
  static constexpr bool SizeOk(const int size) { return size > 0 && size < N; }
  static int Size(const Point& point) { return point.rows(); }
  static void Zero(Point& point) { point.setZero(); }
  static double Dot(const Point& a, const Point& b) { return a.dot(b); }
};
// B-spline using VectorNd points.
using BSplineNd = BSplineT<SplineTraitsNd<>>;

template <int N>
using BSplineNdWithMaxSize = BSplineT<SplineTraitsNd<N>>;

// A B-Spline on eigenmath::VectorXd.
struct SplineTraitsXd {
  typedef eigenmath::VectorXd Point;
  static constexpr bool SizeOk(const int size) { return size > 0; }
  static int Size(const Point& point) { return point.rows(); }
  static void Zero(Point& point) { point.setZero(); }
  static double Dot(const Point& a, const Point& b) { return a.dot(b); }
};
typedef BSplineT<SplineTraitsXd> BSplineXd;

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_BSPLINE_H_
