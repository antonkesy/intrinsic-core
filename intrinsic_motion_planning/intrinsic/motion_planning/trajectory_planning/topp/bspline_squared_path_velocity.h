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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_BSPLINE_SQUARED_PATH_VELOCITY_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_BSPLINE_SQUARED_PATH_VELOCITY_H_

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "Eigen/SparseCore"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/spline/bspline.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/bspline_squared_path_velocity.pb.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/squared_path_velocity_interface.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomials.h"

namespace intrinsic::topp {

// Number of derivatives to compute for the basis functions of squared path
// velocities.
constexpr int kNumSquaredPathVelocityDerivatives = 3;

// Minimum number of nodes used to approximate the numerical integral of the
// `BSplineSquareRootOfSquaredPathVelocityInverseIntegrand` function.
// TODO(b/387482079): Optimize the number of nodes for computation efficiency
// and numerical robustness.
constexpr int kMinNodesOfSquareRootOfSquaredPathVelocityInverseIntegrand = 1000;

// Returns the integrand `1.0/sqrt(spline value)` of b-spline squared path
// velocity parameterization.
inline std::function<absl::StatusOr<double>(double knot_curve_parameter)>
BSplineSquareRootOfSquaredPathVelocityInverseIntegrand(
    const BSpline1d& spline) {
  return [&spline](double knot_curve_parameter) -> absl::StatusOr<double> {
    BSpline1d::Point spline_point;
    if (!spline.EvalCurve(knot_curve_parameter, &spline_point)) {
      return absl::StatusOr<double>(
          absl::InternalError("Could not evaluate the B-spline."));
    }
    return absl::StatusOr<double>(1.0 / sqrt(spline_point[0]));
  };
}

// Clamps the `path_variable` to the range `[path_start, path_end]` with a small
// numerical tolerance to increase robustness.
inline absl::StatusOr<double> ClampPathVariable(const double path_variable,
                                                const double path_start,
                                                const double path_end) {
  // Tolerance for the path positions at computing squared path velocities
  // (derivatives and timings). Path positions at most `kPathPositionTolerance`
  // distance away from the boundaries are valid and will be clamped to the
  // limits. Values further away will incur in an error.
  constexpr double kPathPositionTolerance = 1e-4;
  if (path_variable < path_start - kPathPositionTolerance ||
      path_variable > path_end + kPathPositionTolerance) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The path_variable ", path_variable, " is outside the valid range [",
        path_start - kPathPositionTolerance, ", ",
        path_end + kPathPositionTolerance, "]."));
  }
  return std::clamp(path_variable, path_start, path_end);
}

// Implementation of `SquaredPathVelocityInterface` that uses a B-Spline
// parameterization.
class BSplineSquaredPathVelocity : public SquaredPathVelocityInterface {
 public:
  // Each consecutive pair of values from `path_variables` is connected via a
  // BSpline polynomial of degree `spline_degree`. This implies that the number
  // of basis functions is `path_variables.size() + spline_degree - 1`.
  static absl::StatusOr<std::unique_ptr<BSplineSquaredPathVelocity>> Create(
      absl::Span<const double> path_variables, int spline_degree);

  // Computes the basis functions for the trajectory of squared path velocities
  // and its derivatives (first and second order). When calling
  // `ComputeBasisFunctions` without argument, they are computed at the BSpline
  // unique knots defined at initialization by `path_variables`. The basis
  // functions' matrices constructed via this function are guaranteed to fill
  // each row with values corresponding to each knot interval for all
  // derivatives.
  absl::StatusOr<std::vector<Eigen::SparseMatrix<double>>>
  ComputeBasisFunctions();

  // Same as above, but instead evaluates the basis functions' matrices at
  // `query_values`.
  absl::StatusOr<std::vector<Eigen::SparseMatrix<double>>>
  ComputeBasisFunctions(absl::Span<const double> query_values);

  // Computes the timings at the BSpline unique knots given by the
  // `path_variables` provided at initialization.
  absl::StatusOr<std::vector<double>> EvaluateTimes() const override;

  // Computes the path variable corresponding to a given `query_time`.
  icon::RealtimeStatusOr<double> GetPathVariableForTime(
      double query_time) const;

  // Computes the time corresponding to a given `path_variable`.
  absl::StatusOr<double> GetTimeForPathVariable(
      double path_variable) const override;

  // Computes the squared path velocities and its derivatives at the BSpline
  // unique knots given by the `path_variables` provided at initialization.
  absl::StatusOr<std::vector<std::vector<double>>>
  EvaluateSquaredPathVelocitiesAndDerivatives() const override;

  // Computes the squared path velocities and its derivatives at
  // `query_values`.
  absl::StatusOr<std::vector<std::vector<double>>>
  EvaluateSquaredPathVelocitiesAndDerivatives(
      absl::Span<const double> query_values) const override;

  int Degree() const { return spline_degree_; }
  int NumBasisFunctions() const;
  FixedVector<double, 2> Domain() const override;
  const BSpline1d& Spline() const { return *spline_; }

  // Populates the squared path velocity of a `ToppTrajectoryResult` proto.
  absl::Status PopulateProto(
      intrinsic_proto::topp::ToppTrajectoryResult* proto_msg) const override;

  // Computes the piecewise polynomial that defines the overall trajectory of
  // squared path velocities, given the `weights` for all basis functions. The
  // integrals over each knot interval and cumulative integrals are precomputed,
  // so that queries about trajectory durations can be more easily computed.
  absl::Status PrecomputePolynomialsAndIntegrals(
      absl::Span<const double> weights);

 private:
  BSplineSquaredPathVelocity(int num_basis_functions, int spline_degree,
                             absl::Span<const double> knots,
                             std::unique_ptr<BSpline1d> spline);

  // Storage for the number of basis functions, spline degree and BSpline knots
  // to represent the squared path velocities trajectory.
  int num_basis_functions_;
  int spline_degree_;
  std::vector<double> knots_;

  // Contains all knots values except the repeated values at the begin and end.
  std::vector<double> search_ranges_;

  // Storage for the values of cumulative integrals of nonzero measure knot
  // intervals. It has size `search_ranges_.size()`.
  std::vector<double> cumulative_integrals_;

  // Storage for the values of the overall polynomials of the squared path
  // velocity trajectory.
  std::vector<CubicPolynomial> polynomials_;

  // Underlying BSpline basis functions to construct basis function derivatives.
  std::unique_ptr<BSpline1d> spline_;
};

// Creator of B-spline based of a squared path velocity of degree one. It takes
// in the `path_variables` that define the domain of the basis functions and the
// `squared_path_velocities` via which the basis functions need to pass. Thus,
// this function computes the weights to achieve that.
absl::StatusOr<std::unique_ptr<BSplineSquaredPathVelocity>>
CreateDegreeOneBSplineSquaredPathVelocity(
    absl::Span<const double> path_variables,
    absl::Span<const double> squared_path_velocities);

intrinsic_proto::topp::BSplineSquaredPathVelocity ToProto(
    const BSplineSquaredPathVelocity& squared_path_velocity);

absl::StatusOr<std::unique_ptr<BSplineSquaredPathVelocity>> FromProto(
    const intrinsic_proto::topp::BSplineSquaredPathVelocity&
        squared_path_velocity_proto);

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_BSPLINE_SQUARED_PATH_VELOCITY_H_
