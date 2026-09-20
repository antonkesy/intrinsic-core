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

#include "intrinsic/motion_planning/trajectory_planning/topp/bspline_squared_path_velocity.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/SparseCore"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/math/signals/gauss_legendre_quadrature.h"
#include "intrinsic/math/spline/bspline.h"
#include "intrinsic/math/time_series_utils.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/bspline_squared_path_velocity.pb.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/integrate_cubic_squared_path_velocity_polynomial.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/integrate_squared_path_velocity_polynomial.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_analytical_polynomial_integrals.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomials.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_solver_commons.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_trajectory_result.pb.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::topp {

namespace {

// Spline degree for functions valid only for spline degree 1.
const int kSplineDegreeOne = 1;

// Spline degree for functions valid only for spline degree 2.
const int kSplineDegreeTwo = 2;

// Spline degree for functions valid only for spline degree 3.
const int kSplineDegreeThree = 3;

// Struct to store the analytical form of the cubic polynomials and the
// corresponding cumulative integrals.
struct CumulativeIntegralsResult {
  std::vector<CubicPolynomial> polynomials;
  std::vector<double> cumulative_integrals;
};

// Specialization for spline degree one.
absl::StatusOr<std::vector<double>>
ComputeCumulativeIntegralsForSplineDegreeOne(
    absl::Span<const double> search_ranges, const BSpline1d& spline) {
  // Integrate the square root of the inverse of the squared path velocity along
  // each range defined by a consecutive pair of `search_ranges`.
  std::vector<double> squared_path_velocities(search_ranges.size(), 0.0);
  for (int id = 0; id < search_ranges.size(); ++id) {
    BSpline1d::Point spline_point;
    if (!spline.EvalCurve(search_ranges[id], &spline_point)) {
      return absl::InternalError("Could not evaluate the B-spline.");
    }
    squared_path_velocities[id] = spline_point[0];
  }
  std::vector<double> path_sample_steps(search_ranges.size() - 1);
  for (int id = 0; id < search_ranges.size() - 1; ++id) {
    path_sample_steps[id] = search_ranges[id + 1] - search_ranges[id];
  }

  // Compute piecewise segment integrals and then accumulate them.
  INTR_ASSIGN_OR_RETURN(std::vector<double> piecewise_segment_integrals,
                        ReconstructPathTimeStepsForLinearModel(
                            absl::MakeConstSpan(path_sample_steps),
                            absl::MakeConstSpan(squared_path_velocities)));
  std::vector<double> cumulative_integrals(search_ranges.size(), 0.0);
  for (int i = 0; i < piecewise_segment_integrals.size(); ++i) {
    cumulative_integrals[i + 1] =
        cumulative_integrals[i] + piecewise_segment_integrals[i];
  }

  return cumulative_integrals;
}

absl::StatusOr<double> ComputePartialTimingAtIntervalForSplineDegreeOne(
    const double range_start, const double range_end, const BSpline1d& spline) {
  std::vector<double> squared_path_velocities;
  for (const double path_values : {range_start, range_end}) {
    BSpline1d::Point spline_point;
    if (!spline.EvalCurve(path_values, &spline_point)) {
      return absl::InternalError("Could not evaluate the B-spline.");
    }
    squared_path_velocities.push_back(spline_point[0]);
  }
  // Squared path velocities are non-negative, except for the path start and
  // end. Thus, one of the `avg_path_velocity` should be non-negative;
  const double avg_path_velocity =
      0.5 * (std::sqrt(std::abs(squared_path_velocities.front())) +
             std::sqrt(std::abs(squared_path_velocities.back())));
  const double analytical_integral =
      (range_end - range_start) / avg_path_velocity;

  return analytical_integral;
}

// Returns the analytical form of the quadratic polynomial for a given basis
// function `basis_function_id`. As the underlying BSpline basis functions have
// degree `kSplineDegreeTwo`, each basis function affects `kSplineDegreeTwo` + 1
// knot intervals. The desired knot interval for which the polynomial is to be
// constructed is specified by the `knot_interval_id` ranging from [0,
// `kSplineDegreeTwo`]. The returned vector groups the coefficients as
// [quadratic, linear, constant], such that the corresponding polynomial
// expression is "quadratic * x^2 + linear * x + constant". Each polynomial
// describes the curve behavior between consecutive pairs of `knots` that define
// the BSpline. This function is only valid for `kSplineDegreeTwo`.
absl::StatusOr<eigenmath::Vector3d> GetQuadraticPolynomial(
    int basis_function_id, int knot_interval_id, absl::Span<const double> knots,
    double range_start) {
  // The `basis_function_id` ranges from [0, knots.size() -
  // `kSplineDegreeTwo` - 1).
  if (basis_function_id < 0 ||
      basis_function_id >= knots.size() - kSplineDegreeTwo - 1) {
    return absl::InvalidArgumentError(
        absl::StrCat("The `basis_function_id` to construct the quadratic "
                     "polynomial is outside the range [",
                     0, ", ", knots.size() - kSplineDegreeTwo, "]. Got ",
                     basis_function_id, "."));
  }
  // The `knot_interval_id` ranges from [0, `kSplineDegreeTwo`].
  if (knot_interval_id < 0 || knot_interval_id > kSplineDegreeTwo) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The `knot_interval_id` to construct the quadratic "
        "polynomial is outside the range [",
        0, ", ", kSplineDegreeTwo, "]. Got ", knot_interval_id, "."));
  }
  // The knots vector should have size at least `kSplineDegreeTwo` * 2
  if (knots.size() < (kSplineDegreeTwo + 1) * 2) {
    return absl::InvalidArgumentError(
        absl::StrCat("The `knots` vector should have size at least ",
                     (kSplineDegreeTwo + 1) * 2, ". Got ", knots.size(), "."));
  }

  double knot0 = knots[basis_function_id + 0] - range_start;
  double knot1 = knots[basis_function_id + 1] - range_start;
  double knot2 = knots[basis_function_id + 2] - range_start;
  double knot3 = knots[basis_function_id + 3] - range_start;
  eigenmath::Vector3d coeffs = eigenmath::Vector3d::Zero();
  // The `knot_interval_id` allows to select which one of the quadratic
  // polynomials that define a single BSpline basis function to construct. Given
  // that the basis functions are quadratic, each of them affects three knot
  // intervals as detailed in the following cases. The knot0 is denoted x0,
  // knot1 is denoted x1, knot2 is denoted x2 and knot3 is denoted x3.
  switch (knot_interval_id) {
    case 0: {
      //    (x-x0)^2
      // ---------------- for x0 <= x < x1
      //  (x2-x0)(x1-x0)
      double denom = (knot2 - knot0) * (knot1 - knot0);
      coeffs[0] = 1.0 / denom;
      coeffs[1] = -2.0 * knot0 / denom;
      coeffs[2] = knot0 * knot0 / denom;
      break;
    }
    case 1: {
      //   (x-x0)(x2-x)       (x3-x)(x-x1)
      // ---------------- + ---------------- for x1 <= x < x2
      //  (x2-x0)(x2-x1)     (x3-x1)(x2-x1)
      double denom1 = (knot2 - knot0) * (knot2 - knot1);
      double denom2 = (knot3 - knot1) * (knot2 - knot1);
      coeffs[0] = -1.0 / denom1 - 1.0 / denom2;
      coeffs[1] = (knot0 + knot2) / denom1 + (knot1 + knot3) / denom2;
      coeffs[2] = -knot0 * knot2 / denom1 - knot1 * knot3 / denom2;
      break;
    }
    case 2: {
      //    (x3-x)^2
      // ---------------- for x2 <= x < x3
      //  (x3-x1)(x3-x2)
      double denom = (knot3 - knot1) * (knot3 - knot2);
      coeffs[0] = 1.0 / denom;
      coeffs[1] = -2.0 * knot3 / denom;
      coeffs[2] = knot3 * knot3 / denom;
      break;
    }
  }
  return coeffs;
}

// Constructs the overall quadratic polynomials for the given vector of
// `weights` for all nonzero measure knot intervals. This means that for the
// first `kSplineDegreeTwo` and the last `kSplineDegreeTwo` intervals of the
// knot vector `knots` no polynomial is constructed. The total number of
// polynomials is then `knots.size() - 2 * `kSplineDegreeTwo` - 1`. This
// function is only valid for `kSplineDegreeTwo`.
absl::StatusOr<std::vector<QuadraticPolynomial>>
GetOverallQuadraticTrajectoryPolynomials(absl::Span<const double> weights,
                                         absl::Span<const double> knots) {
  if (weights.size() != knots.size() - kSplineDegreeTwo - 1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The weights vector should have size ",
        knots.size() - kSplineDegreeTwo - 1, ". Got ", weights.size(), "."));
  }

  const int num_polynomials = knots.size() - 2 * kSplineDegreeTwo - 1;
  std::vector<QuadraticPolynomial> polynomials(num_polynomials);
  for (int poly_id = 0; poly_id < num_polynomials; ++poly_id) {
    const double& range_start = knots[kSplineDegreeTwo + poly_id];
    const double& range_end = knots[kSplineDegreeTwo + poly_id + 1];

    // Note that the polynomials are shifted to start from zero so that
    // regardless of the value of range_start, they are numerically stable.
    polynomials[poly_id].range_start = 0.0;
    polynomials[poly_id].range_end = range_end - range_start;
    for (int knot_interval_id = 0; knot_interval_id <= kSplineDegreeTwo;
         ++knot_interval_id) {
      const int basis_function_id =
          kSplineDegreeTwo + poly_id - knot_interval_id;
      INTR_ASSIGN_OR_RETURN(
          eigenmath::Vector3d coeffs,
          GetQuadraticPolynomial(basis_function_id, knot_interval_id, knots,
                                 range_start));
      polynomials[poly_id].coeffs += weights[basis_function_id] * coeffs;
    }
  }

  return polynomials;
}

// Specialization for spline degree two.
// Constructs an analytical piecewise quadratic polynomial representation of the
// squared path velocity trajectory. This is determined uniquely by 1) the
// vector of `weights` for each basis function and 2) the set of basis functions
// which are implied by the vector of `knots`. It then computes the cumulative
// integrals by integrating each quadratic polynomial.
absl::StatusOr<std::vector<double>>
ComputeCumulativeIntegralsForSplineDegreeTwo(absl::Span<const double> weights,
                                             absl::Span<const double> knots) {
  INTR_ASSIGN_OR_RETURN(
      const std::vector<QuadraticPolynomial> polynomials,
      GetOverallQuadraticTrajectoryPolynomials(weights, knots));

  std::vector<double> cumulative_integrals(polynomials.size() + 1, 0.0);
  for (int poly_id = 0; poly_id < polynomials.size(); ++poly_id) {
    const bool is_last_polynomial = poly_id == polynomials.size() - 1;
    INTR_ASSIGN_OR_RETURN(
        double interval_integral,
        IntegrateSquaredPathVelocityPolynomial(
            polynomials[poly_id], /*truncate=*/is_last_polynomial));
    cumulative_integrals[poly_id + 1] =
        cumulative_integrals[poly_id] + interval_integral;
  }
  return cumulative_integrals;
}

// Returns the analytical form of the cubic polynomial for a given basis
// function `basis_function_id`. As the underlying BSpline basis functions have
// degree `kSplineDegreeThree`, each basis function affects `kSplineDegreeThree`
// + 1 knot intervals. The desired knot interval for which the polynomial is to
// be constructed is specified by the `knot_interval_id` ranging from [0,
// `kSplineDegreeThree`]. The returned vector groups the coefficients as [cubic,
// quadratic, linear, constant], such that the corresponding polynomial
// expression is "cubic * x^3 + quadratic * x^2 + linear * x + constant". Each
// polynomial describes the curve behavior between consecutive pairs of `knots`
// that define the BSpline. This function is only valid for
// `kSplineDegreeThree`.
absl::StatusOr<eigenmath::Vector4d> GetCubicPolynomial(
    int basis_function_id, int knot_interval_id, absl::Span<const double> knots,
    double range_start) {
  // The `basis_function_id` ranges from [0, knots.size() -
  // `kSplineDegreeThree` - 1).
  if (basis_function_id < 0 ||
      basis_function_id >= knots.size() - kSplineDegreeThree - 1) {
    return absl::InvalidArgumentError(
        absl::StrCat("The `basis_function_id` to construct the cubic "
                     "polynomial is outside the range [",
                     0, ", ", knots.size() - kSplineDegreeThree, "]. Got ",
                     basis_function_id, "."));
  }
  // The `knot_interval_id` ranges from [0, `kSplineDegreeThree`].
  if (knot_interval_id < 0 || knot_interval_id > kSplineDegreeThree) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The `knot_interval_id` to construct the cubic "
        "polynomial is outside the range [",
        0, ", ", kSplineDegreeThree, "]. Got ", knot_interval_id, "."));
  }
  // The knots vector should have size at least `kSplineDegreeThree` * 2
  if (knots.size() < (kSplineDegreeThree + 1) * 2) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The `knots` vector should have size at least ",
        (kSplineDegreeThree + 1) * 2, ". Got ", knots.size(), "."));
  }

  const double knot0 = knots[basis_function_id + 0] - range_start;
  const double knot1 = knots[basis_function_id + 1] - range_start;
  const double knot2 = knots[basis_function_id + 2] - range_start;
  const double knot3 = knots[basis_function_id + 3] - range_start;
  const double knot4 = knots[basis_function_id + 4] - range_start;
  eigenmath::Vector4d coeffs = eigenmath::Vector4d::Zero();
  // The `knot_interval_id` allows to select which one of the cubic polynomials
  // that define a single BSpline basis function to construct is to be returned.
  // Given that the basis functions are cubic, each of them affects four knot
  // intervals as detailed in the following cases. The knot0 is denoted x0,
  // knot1 is denoted x1, knot2 is denoted x2, knot3 is denoted x3 and knot4 is
  // denoted x4.
  switch (knot_interval_id) {
    case 0: {
      //         (x-x0)^3
      // ----------------------- for x0 <= x < x1
      //  (x3-x0)(x2-x0)(x1-x0)
      const double denom = (knot3 - knot0) * (knot2 - knot0) * (knot1 - knot0);
      coeffs[0] = 1.0 / denom;
      coeffs[1] = -3.0 * knot0 / denom;
      coeffs[2] = 3.0 * ::intrinsic::IPow(knot0, 2) / denom;
      coeffs[3] = -::intrinsic::IPow(knot0, 3) / denom;
      break;
    }
    case 1: {
      //      (x-x0)^2(x2-x)          (x-x0)(x3-x)(x-x1)
      // ----------------------- + ----------------------- +
      //  (x3-x0)(x2-x0)(x2-x1)     (x3-x0)(x3-x1)(x2-x1)
      //
      //      (x4-x)(x-x1)^2
      // ----------------------- for x1 <= x < x2
      //  (x4-x1)(x3-x1)(x2-x1)
      const double denom1 = (knot3 - knot0) * (knot2 - knot0) * (knot2 - knot1);
      const double denom2 = (knot3 - knot0) * (knot3 - knot1) * (knot2 - knot1);
      const double denom3 = (knot4 - knot1) * (knot3 - knot1) * (knot2 - knot1);
      const double knot0_squared = ::intrinsic::IPow(knot0, 2);
      const double knot1_squared = ::intrinsic::IPow(knot1, 2);
      coeffs[0] = -(1.0 / denom1 + 1.0 / denom2 + 1.0 / denom3);
      coeffs[1] = (2.0 * knot0 + knot2) / denom1 +
                  (knot0 + knot1 + knot3) / denom2 +
                  (2.0 * knot1 + knot4) / denom3;
      coeffs[2] = -(2.0 * knot0 * knot2 + knot0_squared) / denom1 -
                  (knot0 * knot1 + knot0 * knot3 + knot1 * knot3) / denom2 -
                  (2.0 * knot1 * knot4 + knot1_squared) / denom3;
      coeffs[3] = knot0_squared * knot2 / denom1 +
                  knot0 * knot1 * knot3 / denom2 +
                  knot1_squared * knot4 / denom3;
      break;
    }
    case 2: {
      //      (x-x0)(x3-x)^2         (x4-x)(x-x1)(x3-x)
      // ----------------------- + ----------------------- +
      //  (x3-x0)(x3-x1)(x3-x2)     (x4-x1)(x3-x1)(x3-x2)
      //
      //      (x4-x)^2(x-x2)
      // ----------------------- for x2 <= x < x3
      //  (x4-x1)(x4-x2)(x3-x2)
      const double denom1 = (knot3 - knot0) * (knot3 - knot1) * (knot3 - knot2);
      const double denom2 = (knot4 - knot1) * (knot3 - knot1) * (knot3 - knot2);
      const double denom3 = (knot4 - knot1) * (knot4 - knot2) * (knot3 - knot2);
      const double knot3_squared = ::intrinsic::IPow(knot3, 2);
      const double knot4_squared = ::intrinsic::IPow(knot4, 2);
      coeffs[0] = 1.0 / denom1 + 1.0 / denom2 + 1.0 / denom3;
      coeffs[1] = -(knot0 + 2.0 * knot3) / denom1 -
                  (knot1 + knot3 + knot4) / denom2 -
                  (knot2 + 2.0 * knot4) / denom3;
      coeffs[2] = (2.0 * knot0 * knot3 + knot3_squared) / denom1 +
                  (knot1 * knot3 + knot1 * knot4 + knot3 * knot4) / denom2 +
                  (2.0 * knot2 * knot4 + knot4_squared) / denom3;
      coeffs[3] = -knot0 * knot3_squared / denom1 -
                  knot1 * knot3 * knot4 / denom2 -
                  knot2 * knot4_squared / denom3;
      break;
    }
    case 3: {
      //         (x4-x)^3
      // ----------------------- for x3 <= x < x4
      //  (x4-x1)(x4-x2)(x4-x3)
      const double denom = (knot4 - knot1) * (knot4 - knot2) * (knot4 - knot3);
      coeffs[0] = -1.0 / denom;
      coeffs[1] = 3.0 * knot4 / denom;
      coeffs[2] = -3.0 * ::intrinsic::IPow(knot4, 2) / denom;
      coeffs[3] = ::intrinsic::IPow(knot4, 3) / denom;
      break;
    }
  }
  return coeffs;
}

// Constructs the overall cubic polynomials for the given vector of `weights`
// for all nonzero measure knot intervals. This means that for the first
// `kSplineDegreeThree` and the last `kSplineDegreeThree` intervals of the knot
// vector `knots` no polynomial is constructed. The total number of polynomials
// is then `knots.size() - 2 * `kSplineDegreeThree` - 1`. This function is only
// valid for `kSplineDegreeThree`.
absl::StatusOr<std::vector<CubicPolynomial>>
GetOverallCubicTrajectoryPolynomials(absl::Span<const double> weights,
                                     absl::Span<const double> knots) {
  if (weights.size() != knots.size() - kSplineDegreeThree - 1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The weights vector should have size ",
        knots.size() - kSplineDegreeThree - 1, ". Got ", weights.size(), "."));
  }

  const int num_polynomials = knots.size() - 2 * kSplineDegreeThree - 1;
  std::vector<CubicPolynomial> polynomials(num_polynomials);
  for (int poly_id = 0; poly_id < num_polynomials; ++poly_id) {
    const double& range_start = knots[kSplineDegreeThree + poly_id];
    const double& range_end = knots[kSplineDegreeThree + poly_id + 1];

    // Note that the polynomials are shifted to start from zero so that
    // regardless of the value of range_start, they are numerically stable.
    polynomials[poly_id].range_start = 0.0;
    polynomials[poly_id].range_end = range_end - range_start;
    for (int knot_interval_id = 0; knot_interval_id <= kSplineDegreeThree;
         ++knot_interval_id) {
      const int basis_function_id =
          kSplineDegreeThree + poly_id - knot_interval_id;
      INTR_ASSIGN_OR_RETURN(
          eigenmath::Vector4d coeffs,
          GetCubicPolynomial(basis_function_id, knot_interval_id, knots,
                             range_start));
      polynomials[poly_id].coeffs += weights[basis_function_id] * coeffs;
    }
  }

  return polynomials;
}

// Specialization for spline degree three.
// Constructs an analytical piecewise cubic polynomial representation of the
// squared path velocity trajectory. This is determined uniquely by 1) the
// vector of `weights` for each basis function and 2) the set of basis functions
// which are implied by the vector of `knots`. It then computes the cumulative
// integrals by integrating each cubic polynomial.
absl::StatusOr<CumulativeIntegralsResult>
ComputeCumulativeIntegralsForSplineDegreeThree(absl::Span<const double> weights,
                                               absl::Span<const double> knots) {
  INTR_ASSIGN_OR_RETURN(const std::vector<CubicPolynomial> polynomials,
                        GetOverallCubicTrajectoryPolynomials(weights, knots));

  std::vector<double> cumulative_integrals(polynomials.size() + 1, 0.0);
  for (int poly_id = 0; poly_id < polynomials.size(); ++poly_id) {
    INTR_ASSIGN_OR_RETURN(
        double interval_integral,
        IntegrateCubicSquaredPathVelocityPolynomial(polynomials[poly_id]));
    cumulative_integrals[poly_id + 1] =
        cumulative_integrals[poly_id] + interval_integral;
  }
  return CumulativeIntegralsResult{
      .polynomials = std::move(polynomials),
      .cumulative_integrals = std::move(cumulative_integrals)};
}

}  // namespace

absl::StatusOr<std::unique_ptr<BSplineSquaredPathVelocity>>
CreateDegreeOneBSplineSquaredPathVelocity(
    absl::Span<const double> path_variables,
    absl::Span<const double> squared_path_velocities) {
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<BSplineSquaredPathVelocity> squared_path_velocity,
      BSplineSquaredPathVelocity::Create(path_variables, /*spline_degree=*/1));

  // The `squared_path_velocities` coincide with the weights in the case of
  // a spline of degree one. The basis function matrix of squared path
  // velocities at the path variables corresponds to the Identity matrix.
  INTR_RETURN_IF_ERROR(squared_path_velocity->PrecomputePolynomialsAndIntegrals(
      absl::MakeConstSpan(squared_path_velocities)));

  return std::move(squared_path_velocity);
}

/* static */
absl::StatusOr<std::unique_ptr<BSplineSquaredPathVelocity>>
BSplineSquaredPathVelocity::Create(absl::Span<const double> path_variables,
                                   int spline_degree) {
  if (path_variables.empty()) {
    return absl::InvalidArgumentError(
        "The `path_variables` vector cannot be empty.");
  }
  if (path_variables.back() <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The path length must be > 0.0. Got ", path_variables.back(), "."));
  }

  // Compute the expected number of basis functions for the chosen degree.
  int num_basis_functions = path_variables.size() + spline_degree - 1;

  // Initialize underlying spline with the appropriate number of knots so that
  // it supports the desired `num_basis_functions_` for the desired spline order
  // `spline_degree`. We further initialize the knots to match the path
  // variables.
  const int num_knots = BSpline1d::NumKnots(num_basis_functions, spline_degree);
  auto spline = std::make_unique<BSpline1d>();
  spline->Init(spline_degree, num_knots);

  std::vector<double> knots(num_knots);
  for (int i = 0; i < spline_degree; ++i) {
    knots[i] = path_variables.front();
    knots[i + spline_degree + path_variables.size()] = path_variables.back();
  }
  for (int i = 0; i < path_variables.size(); ++i) {
    knots[i + spline_degree] = path_variables[i];
  }
  spline->SetKnotVector(knots);

  // Using absl::WrapUnique() below because of the private constructor of
  // QuadraticBSplineSquaredPathVelocity.
  return absl::WrapUnique(new BSplineSquaredPathVelocity(
      num_basis_functions, spline_degree, absl::MakeConstSpan(knots),
      std::move(spline)));
}

BSplineSquaredPathVelocity::BSplineSquaredPathVelocity(
    int num_basis_functions, int spline_degree, absl::Span<const double> knots,
    std::unique_ptr<BSpline1d> spline)
    : num_basis_functions_(num_basis_functions),
      spline_degree_(spline_degree),
      knots_({knots.begin(), knots.end()}),
      search_ranges_(
          {knots.begin() + spline_degree, knots.end() - spline_degree}),
      spline_(std::move(spline)) {};

absl::StatusOr<std::vector<Eigen::SparseMatrix<double>>>
BSplineSquaredPathVelocity::ComputeBasisFunctions() {
  return ComputeBasisFunctions(search_ranges_);
}

absl::StatusOr<std::vector<Eigen::SparseMatrix<double>>>
BSplineSquaredPathVelocity::ComputeBasisFunctions(
    absl::Span<const double> query_values) {
  if (query_values.empty()) {
    return absl::InvalidArgumentError(
        "The query_values vector cannot be empty.");
  }

  // Evaluation of basis functions at unique knots.
  const int num_path_vars = query_values.size();
  const int num_available_derivatives =
      std::min(spline_degree_ + 1, kNumSquaredPathVelocityDerivatives);
  std::vector<BSpline1d::Point> values_at_path_var(num_available_derivatives);
  std::vector<std::vector<Eigen::Triplet<double>>> triplets_set(
      kNumSquaredPathVelocityDerivatives);

  BSpline1d::Point zero_spline_point;
  zero_spline_point[0] = 0.0;
  for (int path_var_id = 0; path_var_id < num_path_vars; ++path_var_id) {
    const double path_var = query_values[path_var_id];

    // Get the knot index for query path variable.
    const int bf_id = GetLowerIndexForValue<double>(
        absl::MakeConstSpan(search_ranges_), path_var);
    const int min_ctrl_id =
        std::max(0, bf_id - (bf_id == search_ranges_.size() - 1 ? 1 : 0));
    const int max_ctrl_id =
        std::min(min_ctrl_id + spline_degree_ + 1, num_basis_functions_);

    for (int ctrl_id = min_ctrl_id; ctrl_id < max_ctrl_id; ++ctrl_id) {
      std::vector<BSpline1d::Point> control_points(num_basis_functions_,
                                                   zero_spline_point);
      control_points[ctrl_id][0] = 1.0;
      spline_->SetControlPoints(control_points);

      // If the absolute value of a spline is smaller than this value, it is
      // considered zero.
      constexpr double kZeroToleranceForBSplineCoefficient = 1e-10;
      spline_->EvalCurveAndDerivatives(path_var, &values_at_path_var);
      for (int deg_id = 0; deg_id < num_available_derivatives; ++deg_id) {
        if (std::abs(values_at_path_var[deg_id][0]) >
            kZeroToleranceForBSplineCoefficient) {
          triplets_set[deg_id].push_back(Eigen::Triplet<double>(
              path_var_id, ctrl_id, values_at_path_var[deg_id][0]));
        }
      }
    }
  }

  std::vector<Eigen::SparseMatrix<double>> basis_functions;
  for (const std::vector<Eigen::Triplet<double>>& triplets : triplets_set) {
    basis_functions.push_back(
        Eigen::SparseMatrix<double>(num_path_vars, num_basis_functions_));
    basis_functions.back().reserve(triplets.size());
    basis_functions.back().setFromTriplets(triplets.begin(), triplets.end());
  }
  return basis_functions;
}

absl::StatusOr<std::vector<double>> BSplineSquaredPathVelocity::EvaluateTimes()
    const {
  if (cumulative_integrals_.empty()) {
    return absl::FailedPreconditionError(
        "No integrals have been precomputed yet. Call "
        "PrecomputePolynomialsAndIntegrals first.");
  }
  return cumulative_integrals_;
}

icon::RealtimeStatusOr<double>
BSplineSquaredPathVelocity::GetPathVariableForTime(
    const double query_time) const {
  // The functionality is only implemented for `kSplineDegreeThree`.
  if (spline_degree_ != kSplineDegreeThree) {
    return icon::UnimplementedError(
        "This function is only implemented for spline degree 3.");
  }
  if (cumulative_integrals_.empty() || polynomials_.empty()) {
    return icon::FailedPreconditionError(
        "No integrals have been precomputed yet. Call "
        "PrecomputePolynomialsAndIntegrals first.");
  }
  const double kTimeTolerance = 1e-4;
  if ((query_time < cumulative_integrals_.front() - kTimeTolerance) ||
      (query_time > cumulative_integrals_.back() + kTimeTolerance)) {
    return icon::InvalidArgumentError(
        "The query time is outside the range of the spline.");
  }
  const double clamped_query_time = std::clamp(
      query_time, cumulative_integrals_.front(), cumulative_integrals_.back());

  // Get the right interval to be used for evaluation.
  const int interval_index = GetLowerIndexForValue<double>(
      absl::MakeConstSpan(cumulative_integrals_), clamped_query_time);

  // If the `interval_index` equals `cumulative_integrals_.size()-1`, we are
  // at the end of the cumulative integral vector, thus we return the last
  // search range knot representing the corresponding path variable.
  if (interval_index >= cumulative_integrals_.size() - 1) {
    return search_ranges_.back();
  }

  // Otherwise, check if `clamped_query_time` matches any of the boundaries of
  // the `interval_index` of the cumulative integrals, in which case we can
  // return early the corresponding search range (corresponding path variable).
  if (intrinsic::AlmostEquals(clamped_query_time,
                              cumulative_integrals_[interval_index])) {
    return search_ranges_[interval_index];
  }
  if (intrinsic::AlmostEquals(clamped_query_time,
                              cumulative_integrals_[interval_index + 1])) {
    return search_ranges_[interval_index + 1];
  }

  // If none of the above, we search within the `interval_index` spline
  // polynomial for the path variable value corresponding to the query time.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double path_variable_delta,
      UpperLimitOfIntegralOfOneOverSqrtOfStructuredCubicPolynomial(
          polynomials_[interval_index],
          clamped_query_time - cumulative_integrals_[interval_index]));
  return search_ranges_[interval_index] + path_variable_delta;
}

absl::StatusOr<double> BSplineSquaredPathVelocity::GetTimeForPathVariable(
    double path_variable) const {
  if (cumulative_integrals_.empty() ||
      (spline_degree_ == kSplineDegreeThree && polynomials_.empty())) {
    return icon::FailedPreconditionError(
        "No integrals have been precomputed yet. Call "
        "PrecomputePolynomialsAndIntegrals first.");
  }

  INTR_ASSIGN_OR_RETURN(const double clamped_path_variable,
                        ClampPathVariable(path_variable, search_ranges_.front(),
                                          search_ranges_.back()));

  // Get the right interval to be used for evaluation.
  const int interval_index = GetLowerIndexForValue<double>(
      absl::MakeConstSpan(search_ranges_), clamped_path_variable);

  // If the `interval_index` equals `search_ranges_.size()-1`, we are at the
  // end of the last search range, thus we return the last cumulative
  // integral.
  if (interval_index >= search_ranges_.size() - 1) {
    return cumulative_integrals_.back();
  }

  // Otherwise, get the corresponding polynomial and check if the
  // `clamped_path_variable` is at the start or end boundaries, in which case
  // we can return early the corresponding cumulative integral.
  if (AlmostEquals(clamped_path_variable, search_ranges_[interval_index])) {
    return cumulative_integrals_[interval_index];
  }
  if (AlmostEquals(clamped_path_variable, search_ranges_[interval_index + 1])) {
    return cumulative_integrals_[interval_index + 1];
  }

  // If none of the above, we integrate over the corresponding interval and
  // add the cumulative value up to the start of the polynomial interval.
  double partial_integral;
  if (spline_degree_ == kSplineDegreeOne) {
    // Compute integral analytically.
    INTR_ASSIGN_OR_RETURN(
        partial_integral,
        ComputePartialTimingAtIntervalForSplineDegreeOne(
            search_ranges_[interval_index], clamped_path_variable, *spline_));
  } else if (spline_degree_ == kSplineDegreeThree) {
    CubicPolynomial polynomial = polynomials_[interval_index];
    polynomial.range_start = 0.0;
    polynomial.range_end =
        clamped_path_variable - search_ranges_[interval_index];
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        partial_integral,
        IntegrateOneOverSqrtOfStructuredCubicPolynomial(polynomial));
  } else {
    // If none of the above, we integrate over the corresponding interval and
    // add the cumulative value up to the start of the polynomial interval.
    INTR_ASSIGN_OR_RETURN(
        partial_integral,
        GaussLegendreQuadrature(
            BSplineSquareRootOfSquaredPathVelocityInverseIntegrand(*spline_),
            /*start=*/search_ranges_[interval_index],
            /*end=*/clamped_path_variable,
            kMinNodesOfSquareRootOfSquaredPathVelocityInverseIntegrand));
  }
  const double total_integral =
      cumulative_integrals_[interval_index] + partial_integral;

  return std::clamp(total_integral, cumulative_integrals_[interval_index],
                    cumulative_integrals_[interval_index + 1]);
}

absl::StatusOr<std::vector<std::vector<double>>>
BSplineSquaredPathVelocity::EvaluateSquaredPathVelocitiesAndDerivatives()
    const {
  if (cumulative_integrals_.empty()) {
    return absl::FailedPreconditionError(
        "No integrals have been precomputed yet. Call "
        "PrecomputePolynomialsAndIntegrals first.");
  }
  return EvaluateSquaredPathVelocitiesAndDerivatives(search_ranges_);
}

absl::StatusOr<std::vector<std::vector<double>>>
BSplineSquaredPathVelocity::EvaluateSquaredPathVelocitiesAndDerivatives(
    absl::Span<const double> query_values) const {
  if (cumulative_integrals_.empty()) {
    return absl::FailedPreconditionError(
        "No integrals have been precomputed yet. Call "
        "PrecomputePolynomialsAndIntegrals first.");
  }

  std::vector<std::vector<double>> derivatives_at_query_values;
  derivatives_at_query_values.reserve(query_values.size());
  for (const double path_variable : query_values) {
    INTR_ASSIGN_OR_RETURN(
        const double clamped_path_variable,
        ClampPathVariable(path_variable, search_ranges_.front(),
                          search_ranges_.back()));

    const int num_available_derivatives =
        std::min(spline_degree_ + 1, kNumSquaredPathVelocityDerivatives);
    std::vector<BSpline1d::Point> values_at_path_var(num_available_derivatives);
    std::vector<double> path_variable_derivatives(
        kNumSquaredPathVelocityDerivatives, 0.0);

    spline_->EvalCurveAndDerivatives(clamped_path_variable,
                                     &values_at_path_var);
    for (int deg_id = 0; deg_id < num_available_derivatives; ++deg_id) {
      path_variable_derivatives[deg_id] = values_at_path_var[deg_id][0];
    }
    derivatives_at_query_values.push_back(path_variable_derivatives);
  }
  return derivatives_at_query_values;
}

int BSplineSquaredPathVelocity::NumBasisFunctions() const {
  return num_basis_functions_;
}

FixedVector<double, 2> BSplineSquaredPathVelocity::Domain() const {
  return {search_ranges_.front(), search_ranges_.back()};
}

absl::Status BSplineSquaredPathVelocity::PrecomputePolynomialsAndIntegrals(
    absl::Span<const double> weights) {
  if (weights.size() != num_basis_functions_) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The number of weights must be equal to the number of basis "
        "functions. Got ",
        weights.size(), " weights and ", num_basis_functions_,
        " basis functions."));
  }

  // Set control points.
  std::vector<BSpline1d::Point> control_points;
  control_points.reserve(num_basis_functions_);
  for (int ctrl_id = 0; ctrl_id < num_basis_functions_; ++ctrl_id) {
    BSpline1d::Point spline_point;
    spline_point[0] = weights[ctrl_id];
    control_points.push_back(spline_point);
  }
  spline_->SetControlPoints(control_points);

  if (spline_degree_ == kSplineDegreeOne) {
    // Analytically compute cumulative integrals for spline degree 1.
    INTR_ASSIGN_OR_RETURN(
        cumulative_integrals_,
        ComputeCumulativeIntegralsForSplineDegreeOne(search_ranges_, *spline_));
  } else if (spline_degree_ == kSplineDegreeTwo) {
    // Analytically compute cumulative integrals for spline degree 2.
    INTR_ASSIGN_OR_RETURN(
        cumulative_integrals_,
        ComputeCumulativeIntegralsForSplineDegreeTwo(weights, knots_));
  } else if (spline_degree_ == kSplineDegreeThree) {
    // Analytically compute cumulative integrals for spline degree 3.
    INTR_ASSIGN_OR_RETURN(
        CumulativeIntegralsResult result,
        ComputeCumulativeIntegralsForSplineDegreeThree(weights, knots_));
    cumulative_integrals_ = std::move(result.cumulative_integrals);
    polynomials_ = std::move(result.polynomials);
  } else {
    // Numerically integrate the square root of the inverse of the squared path
    // velocity along each range defined by a consecutive pair of
    // `search_ranges_`.
    cumulative_integrals_ = std::vector<double>(search_ranges_.size(), 0.0);
    for (int poly_id = 0; poly_id < search_ranges_.size() - 1; ++poly_id) {
      INTR_ASSIGN_OR_RETURN(
          double gauss_legendre_integral,
          GaussLegendreQuadrature(
              BSplineSquareRootOfSquaredPathVelocityInverseIntegrand(*spline_),
              /*start=*/search_ranges_[poly_id],
              /*end=*/search_ranges_[poly_id + 1],
              kMinNodesOfSquareRootOfSquaredPathVelocityInverseIntegrand));
      cumulative_integrals_[poly_id + 1] =
          cumulative_integrals_[poly_id] + gauss_legendre_integral;
    }
  }

  return absl::OkStatus();
}

absl::Status BSplineSquaredPathVelocity::PopulateProto(
    intrinsic_proto::topp::ToppTrajectoryResult* proto_msg) const {
  if (proto_msg == nullptr) {
    return absl::InvalidArgumentError("The `proto_msg` cannot be nullptr.");
  }
  *proto_msg->mutable_squared_path_velocity() = ToProto(*this);
  return absl::OkStatus();
}

intrinsic_proto::topp::BSplineSquaredPathVelocity ToProto(
    const BSplineSquaredPathVelocity& squared_path_velocity) {
  intrinsic_proto::topp::BSplineSquaredPathVelocity proto;
  *proto.mutable_spline() = ToProto(squared_path_velocity.Spline());
  return proto;
}

absl::StatusOr<std::unique_ptr<BSplineSquaredPathVelocity>> FromProto(
    const intrinsic_proto::topp::BSplineSquaredPathVelocity&
        squared_path_velocity_proto) {
  // The squared path velocity constructs an analytical representation of the
  // B-Spline. To that end, it takes a vector of search ranges, that represent
  // the unique knots of the B-Spline. In other words, the search ranges define
  // the intervals for which the B-Spline is defined and removes the repeated
  // knots at the begin and end.
  std::vector<double> search_ranges{
      squared_path_velocity_proto.spline().knots().begin(),
      squared_path_velocity_proto.spline().knots().end()};
  auto last = std::unique(search_ranges.begin(), search_ranges.end());
  search_ranges.erase(last, search_ranges.end());

  std::vector<double> weights;
  weights.reserve(squared_path_velocity_proto.spline().control_points_size());
  for (const auto& control_point :
       squared_path_velocity_proto.spline().control_points()) {
    if (control_point.point_size() != 1) {
      return absl::InvalidArgumentError(
          "Control point size must be 1 for squared path velocity.");
    }
    weights.push_back(control_point.point(0));
  }

  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<BSplineSquaredPathVelocity> squared_path_velocity,
      BSplineSquaredPathVelocity::Create(
          search_ranges, squared_path_velocity_proto.spline().degree()));
  INTR_RETURN_IF_ERROR(
      squared_path_velocity->PrecomputePolynomialsAndIntegrals(weights));
  return squared_path_velocity;
}

}  // namespace intrinsic::topp
