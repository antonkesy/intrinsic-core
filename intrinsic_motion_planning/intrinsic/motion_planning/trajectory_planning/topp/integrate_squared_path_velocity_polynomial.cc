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

#include "intrinsic/motion_planning/trajectory_planning/topp/integrate_squared_path_velocity_polynomial.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/to_string.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::topp {

namespace {

// Minimal perturbation that can be applied to the evaluation path parameters so
// as to be able to cope with numerical singularities.
constexpr double kInfinitesimal = 1e-12;

// For a quadratic polynomial `P(s) = a * s^2 + b * s + c` valid in the interval
// `[0, ds]`, it represents the maximum relative quadratic contribution `|a| *
// ds^2 / max(P(0), P(ds))` below which the polynomial is treated as effectively
// linear over the integration interval. Below this threshold, linear secant
// integration stays within this same relative error of the exact integral,
// while the analytical formulas suffer from floating-point cancellation.
constexpr double kRelativeQuadraticContributionThreshold = 1.0e-7;

// Evaluates the quadratic polynomial first derivative with given `coeffs` at
// the given `path_position`.
double EvaluateQuadraticFirstDerivative(const eigenmath::Vector3d& coeffs,
                                        double path_position) {
  return std::fma(2.0 * coeffs[0], path_position, coeffs[1]);
}

// Computes `b^2 - 4 * a * c` using Kahan's FMA algorithm to avoid catastrophic
// cancellation when `b^2` and `4 * a * c` are nearly equal (i.e. near a
// repeated root or when the parabola vertex is far from the origin).
double ComputeDiscriminant(const eigenmath::Vector3d& coeffs) {
  const double four_ac = 4.0 * coeffs[0] * coeffs[2];
  const double four_ac_error = std::fma(4.0 * coeffs[0], coeffs[2], -four_ac);
  return std::fma(coeffs[1], coeffs[1], -four_ac) - four_ac_error;
}

// Evaluates the quadratic polynomial with given `coeffs` and precomputed
// `discriminant` at `path_position`.
//
// Evaluating in the monomial basis loses all significant digits near a repeated
// root, where `a * s^2`, `b * s` and `c` cancel each other. For a parabola
// whose vertex lies far from the origin this is total: each one of the three
// terms can be of order `1e10` while their sum is of order `1e-6`. The vertex
// form
//   P(s) = a * (s - s_vertex)^2 - discriminant / (4 * a)
//        = ((2 * a * s + b)^2 - discriminant) / (4 * a)
// does not suffer from that cancellation, because `(2 * a * s + b) = P'(s)` is
// evaluated via a single FMA and the remaining subtraction carries exactly the
// (small) `discriminant`. The vertex form is in turn badly conditioned when the
// vertex is far outside the integration range, which happens for a small
// leading coefficient, so the better conditioned of the two is selected by
// comparing the magnitude of the terms that each of them adds up.
double SafeEvaluateQuadratic(const eigenmath::Vector3d& coeffs,
                             double discriminant, double path_position) {
  if (coeffs[0] == 0.0) {
    return std::fma(coeffs[1], path_position, coeffs[2]);
  }
  const double derivative =
      EvaluateQuadraticFirstDerivative(coeffs, path_position);
  const double vertex_form_magnitude =
      std::fabs(derivative * derivative / (4.0 * coeffs[0])) +
      std::fabs(discriminant / (4.0 * coeffs[0]));

  const double quadratic_term = coeffs[0] * intrinsic::IPow(path_position, 2);
  const double linear_term = coeffs[1] * path_position;
  const double constant_term = coeffs[2];
  const double monomial_form_magnitude = std::fabs(quadratic_term) +
                                         std::fabs(linear_term) +
                                         std::fabs(constant_term);
  if (vertex_form_magnitude <= monomial_form_magnitude) {
    return std::fma(derivative, derivative, -discriminant) / (4.0 * coeffs[0]);
  }
  // Evaluate quadratic in monomial basis.
  return quadratic_term + linear_term + constant_term;
}

// Returns the root of the quadratic polynomial given by
// `(-b - sqrt(discriminant)) / (2 * a)`, which is the leftmost root of a convex
// polynomial and the rightmost root of a concave one.
//
// That expression cancels when `b < 0`, because `sqrt(discriminant)` then
// approaches `|b|`; for `b^2 >> 4 * a * c` it returns exactly zero instead of
// the true root. The algebraically equivalent partner
//   `(-b - sqrt(discriminant)) / (2 * a) == 2 * c / (sqrt(discriminant) - b)`
// is a sum of like-signed terms in that regime, so each form is used in the
// half-plane of `b` where it is well conditioned.
double StableNegativeBranchRoot(const eigenmath::Vector3d& coeffs,
                                double sqrt_discriminant) {
  if (coeffs[1] < 0.0) {
    const double denominator = sqrt_discriminant - coeffs[1];
    if (denominator != 0.0) {
      return 2.0 * coeffs[2] / denominator;
    }
  }
  return (-coeffs[1] - sqrt_discriminant) / (2.0 * coeffs[0]);
}

// Computes the integral for a quadratic expression whose leading coefficient is
// negative, given its `coeffs` and precomputed `discriminant`. It constructs
// the integral for the following expression:
//         path_end                             1.0
//   integral         --------------------------------------------------- ds
//     s = path_start  sqrt(coeffs[0] * s^2 + coeffs[1] * s + coeffs[2])
// where coeffs[0] is negative. If `truncate` equals true, then the integral is
// truncated at the point where the quadratic polynomial becomes zero and is
// going negative, as at that point the integral is not well defined.
absl::StatusOr<double> IntegrateConcaveQuadraticExpression(
    double path_start, double path_end, const eigenmath::Vector3d& coeffs,
    double discriminant, bool truncate) {
  const double a = std::fabs(coeffs[0]);
  const double sqrt_a = std::sqrt(a);

  // If the discriminant is less than zero for a concave quadratic polynomial,
  // it means that there is no region in which it takes positive values. Thus,
  // the integral is not well defined.
  if (discriminant <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Concave polynomial roots cannot be imaginary. Its "
        "discriminant (b^2 -4.0*a*c) should be positive. Got ",
        discriminant, ", for coefficients ", eigenmath::ToString(coeffs), "."));
  }

  // Neither endpoint may sit where the polynomial is negative, because the
  // integrand is not defined there. A negative value at the end of the
  // interval is recoverable when `truncate` is set, by integrating only up to
  // the zero crossing; a negative value at the start is not, because the
  // integral would have to start outside the region where it exists.
  double polynomial_at_start =
      SafeEvaluateQuadratic(coeffs, discriminant, path_start);
  if (polynomial_at_start < 0.0) {
    path_start = std::min(path_start + kInfinitesimal, path_end);
    polynomial_at_start =
        SafeEvaluateQuadratic(coeffs, discriminant, path_start);
  }
  if (polynomial_at_start < 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to integrate concave expression: the polynomial is negative at "
        "the start of the interval, where the integrand 1/sqrt(P) does not "
        "exist. Got P(",
        path_start, ") = ", polynomial_at_start, " for coefficients ",
        eigenmath::ToString(coeffs), "."));
  }

  double polynomial_at_end =
      SafeEvaluateQuadratic(coeffs, discriminant, path_end);
  if (polynomial_at_end < 0.0) {
    path_end = std::max(path_end - kInfinitesimal, path_start);
    polynomial_at_end = SafeEvaluateQuadratic(coeffs, discriminant, path_end);
  }
  if (polynomial_at_end < 0.0) {
    if (!truncate) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Failed to integrate concave expression: the polynomial is negative "
          "at the end of the interval, where the integrand 1/sqrt(P) does not "
          "exist. Got P(",
          path_end, ") = ", polynomial_at_end, " for coefficients ",
          eigenmath::ToString(coeffs),
          ". Pass `truncate` to integrate up to the zero crossing instead."));
    }
    // Truncate at the zero crossing considering that the polynomial is concave.
    path_end = StableNegativeBranchRoot(coeffs, std::sqrt(discriminant));
    polynomial_at_end = 0.0;
  }

  // The integral is evaluated through the half-angle identity
  //   integral from `s0` to `s1` of `ds / sqrt(P(s))`
  //     = `(2 / sqrt(a)) * atan(sqrt(a) * (s1 - s0) /
  //                             (sqrt(P(s0)) + sqrt(P(s1))))`
  // This identity degenerates gracefully: when both endpoints are roots the
  // denominator is zero, the argument is an infinity and `atan` returns
  // `pi/2`, giving the exact `pi / sqrt(a)` for the full support.
  const double ds = path_end - path_start;
  if (ds == 0.0) {
    return 0.0;
  }
  auto clamped_path_velocity = [](double polynomial_value) -> double {
    return polynomial_value > 0.0 ? std::sqrt(polynomial_value) : 0.0;
  };
  const double sum_of_path_velocities =
      clamped_path_velocity(polynomial_at_start) +
      clamped_path_velocity(polynomial_at_end);
  const double definite_integral =
      2.0 / sqrt_a * std::atan(sqrt_a * ds / sum_of_path_velocities);

  // We notify the failure if the integral was not possible to compute.
  if (std::isnan(definite_integral)) {
    return absl::InternalError(absl::StrCat(
        "The definite_integral is NaN. Got P(", path_start, ") = ",
        polynomial_at_start, " and P(", path_end, ") = ", polynomial_at_end,
        " for coefficients ", eigenmath::ToString(coeffs), "."));
  }

  return definite_integral;
}

// The antiderivative of the convex integrand is
//   `log(g(s)) / sqrt(a)`,  with  `g(s) = 2 * sqrt(a) * sqrt(P(s)) + P'(s)`,
// so the definite integral is `log(g(path_end) / g(path_start)) / sqrt(a)`.
// The identity
//   (2 * sqrt(a) * sqrt(P) + P') * (2 * sqrt(a) * sqrt(P) - P')
//       = 4 * a * P - P'^2
//       = -(b^2 - 4 * a * c)
// shows that the lost magnitude is exactly the discriminant, which is why
// the cancellation is severe precisely for a (near) repeated root.
struct EndpointTerms {
  // `2 * sqrt(a) * sqrt(P(s))`, which is non-negative (or NaN).
  double scaled_sqrt = 0.0;
  // `P'(s)`.
  double derivative = 0.0;
};

struct LogRatioTerms {
  double numerator = 0.0;
  double denominator = 0.0;
};

// Forms `g(path_end) / g(path_start)` as `numerator / denominator` without ever
// evaluating a cancelling sum. Note that `P'` is strictly increasing for a
// convex polynomial, so `start_terms.derivative <= end_terms.derivative` always
// holds.
LogRatioTerms ComputeLogRatioTerms(const EndpointTerms& start_terms,
                                   const EndpointTerms& end_terms,
                                   double discriminant) {
  if (start_terms.derivative >= 0.0) {
    // `P' >= 0` over the whole interval: both terms of `g` are non-negative,
    // so `g` can be evaluated directly.
    return LogRatioTerms{
        .numerator = end_terms.scaled_sqrt + end_terms.derivative,
        .denominator = start_terms.scaled_sqrt + start_terms.derivative};
  }
  if (end_terms.derivative < 0.0) {
    // `P' < 0` over the whole interval. Substituting `g = -discriminant / h`
    // with `h(s) = 2 * sqrt(a) * sqrt(P(s)) - P'(s)` (a sum of positive terms)
    // makes the discriminant cancel identically in the ratio. This is exact
    // even for a repeated root, where the discriminant is zero.
    return LogRatioTerms{
        .numerator = start_terms.scaled_sqrt - start_terms.derivative,
        .denominator = end_terms.scaled_sqrt - end_terms.derivative};
  }
  // `P'` changes sign, so the interval contains the vertex. As the
  // polynomial is positive there, it has no real root and the discriminant
  // is strictly negative. Using the direct form at the end and the
  // substituted form at the start keeps both factors cancellation free.
  return LogRatioTerms{
      .numerator = (end_terms.scaled_sqrt + end_terms.derivative) *
                   (start_terms.scaled_sqrt - start_terms.derivative),
      .denominator = -discriminant};
}

// Computes the integral for a quadratic expression whose leading coefficient is
// positive, given its `coeffs` and precomputed `discriminant`. It constructs
// the integral for the following expression:
//         path_end                             1.0
//   integral         --------------------------------------------------- ds
//     s = path_start  sqrt(coeffs[0] * s^2 + coeffs[1] * s + coeffs[2])
// where coeffs[0] is positive.
// It assumes that the first input coefficient coeffs[0] is positive and will
// not perform any further checks, because this function should only be used to
// integrate a convex quadratic expression.
absl::StatusOr<double> IntegrateConvexQuadraticExpression(
    double path_start, double path_end, const eigenmath::Vector3d& coeffs,
    double discriminant, bool truncate) {
  // This integration case is only called with a positive value of the first
  // coefficient `a = coeffs[0]`. Thus, the square root must exist.
  const double sqrt_a = std::sqrt(coeffs[0]);

  // Evaluates the square root of the convex quadratic polynomial at
  // `path_variable` safely with numerical tolerance. If the evaluated
  // polynomial value is slightly negative within `-kInfinitesimal`, it is
  // clamped to 0.0 to prevent numerical NaN. If it is strictly below
  // `-kInfinitesimal`, NaN is returned, which is subsequently handled as an
  // error (or truncated if `truncate` is true).
  auto safe_sqrt_evaluation = [&](const double path_variable) -> double {
    const double polynomial_value =
        SafeEvaluateQuadratic(coeffs, discriminant, path_variable);
    if (polynomial_value < -kInfinitesimal) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return std::sqrt(std::max(polynomial_value, 0.0));
  };

  auto evaluate_terms = [&](double path_variable) -> EndpointTerms {
    return EndpointTerms{
        .scaled_sqrt = 2.0 * sqrt_a * safe_sqrt_evaluation(path_variable),
        .derivative = EvaluateQuadraticFirstDerivative(coeffs, path_variable)};
  };
  // `g` and its cancellation-free partner both vanish only where the
  // polynomial has a repeated root, i.e. where the polynomial and its
  // derivative vanish simultaneously. The integral is singular there.
  auto is_at_repeated_root = [](const EndpointTerms& terms) -> bool {
    return terms.scaled_sqrt == 0.0 && terms.derivative == 0.0;
  };
  auto is_degenerate = [&](const EndpointTerms& terms) -> bool {
    return std::isnan(terms.scaled_sqrt) || is_at_repeated_root(terms);
  };

  // If the terms at the start are degenerate (repeated root, or NaN because
  // the polynomial is negative there), the integral is not computable. In this
  // case, we add an infinitesimal perturbation to the path parameter to
  // attempt to evaluate the integral.
  EndpointTerms start_terms = evaluate_terms(path_start);
  if (is_degenerate(start_terms)) {
    start_terms = evaluate_terms(path_start + kInfinitesimal);
  }

  // Same treatment for the end of the interval.
  EndpointTerms end_terms = evaluate_terms(path_end);
  if (is_degenerate(end_terms)) {
    end_terms = evaluate_terms(path_end - kInfinitesimal);
  }

  // If the terms at the end continue to be degenerate, due to a larger
  // numerical error, we truncate the integration until the zero crossing. The
  // truncation is performed only if `truncate` equals true.
  if (is_degenerate(end_terms) && truncate) {
    // Find the root of the polynomial that is causing the zero crossing.
    // For the case when the discriminant is zero, the root reduces to the
    // vertex. For the case when it is negative, then there is no zero crossing
    // and we should not reach this case.
    const double sqrt_discriminant =
        discriminant > 0.0 ? std::sqrt(discriminant) : 0.0;

    // The polynomial is zero at its root by definition, and its derivative
    // there is available in closed form: substituting
    // `root = (-b - sqrt(D)) / (2 * a)` into `P'(s) = 2 * a * s + b` leaves
    // exactly `P'(root) = -sqrt(D)`, so the derivative is read off the
    // discriminant.
    end_terms.scaled_sqrt = 0.0;
    end_terms.derivative = -sqrt_discriminant;

    // A zero discriminant means a repeated root, at which the integrand is
    // genuinely non-integrable and the exact integral diverges. Rather than
    // reporting an infinity, the result is capped by nudging the derivative
    // off zero. `epsilon` is used instead of `kInfinitesimal` because it
    // achieves a better precision in the computation of the integral.
    if (end_terms.derivative == 0.0) {
      end_terms.derivative = -std::numeric_limits<double>::epsilon();
    }
  }

  const LogRatioTerms log_ratio_terms =
      ComputeLogRatioTerms(start_terms, end_terms, discriminant);

  // We notify the failure if the integral was not possible to compute.
  const double ratio =
      std::fabs(log_ratio_terms.numerator / log_ratio_terms.denominator);
  const double definite_integral = std::fabs(std::log(ratio) / sqrt_a);
  if (std::isnan(definite_integral)) {
    return absl::InternalError(absl::StrCat(
        "The definite_integral is NaN. Got log(", log_ratio_terms.numerator,
        "/", log_ratio_terms.denominator, ") / ", sqrt_a, "."));
  }
  if (std::isinf(definite_integral)) {
    return absl::InternalError(absl::StrCat(
        "The definite_integral is infinite, the integrand is singular over the "
        "requested range. Got log(",
        log_ratio_terms.numerator, "/", log_ratio_terms.denominator, ") / ",
        sqrt_a, "."));
  }
  return definite_integral;
}

// Computes the integral for a linear expression, given its `coeffs` and
// precomputed `discriminant`. It constructs the integral for the following
// expression:
//         path_end                  1.0
//   integral         --------------------------------- ds
//     s = path_start  sqrt(coeffs[1] * s + coeffs[2])
// If `truncate` equals true and the expression becomes negative at `path_end`,
// the integral is truncated at the zero crossing, mirroring the behavior of the
// quadratic integrals.
absl::StatusOr<double> IntegrateLinearExpression(
    double path_start, double path_end, const eigenmath::Vector3d& coeffs,
    double discriminant, const bool truncate) {
  double squared_sd_start =
      SafeEvaluateQuadratic(coeffs, discriminant, path_start);
  if (squared_sd_start < -kInfinitesimal) {
    return absl::InvalidArgumentError(
        absl::StrCat("Failed to integrate linear expression as "
                     "squared_sd_start is below zero. Got ",
                     squared_sd_start, "."));
  }
  squared_sd_start = std::max(squared_sd_start, 0.0);
  const double sd_start = std::sqrt(squared_sd_start);

  double squared_sd_end = SafeEvaluateQuadratic(coeffs, discriminant, path_end);
  if (squared_sd_end < -kInfinitesimal) {
    if (!truncate) {
      return absl::InvalidArgumentError(
          absl::StrCat("Failed to integrate linear expression as "
                       "squared_sd_end is below zero. Got ",
                       squared_sd_end, "."));
    }
    // Truncate at the zero crossing. Over this interval the polynomial is
    // approximated by the secant through both endpoints, so the crossing of
    // that secant is used for consistency with the value of the integral. The
    // denominator is strictly positive here, because `squared_sd_start` is
    // non-negative and `squared_sd_end` is strictly negative.
    path_end = path_start + (path_end - path_start) * squared_sd_start /
                                (squared_sd_start - squared_sd_end);
    squared_sd_end = 0.0;
  }
  const double sd_end = std::sqrt(std::max(squared_sd_end, 0.0));

  const double average_vel = 0.5 * (sd_start + sd_end);
  if (average_vel <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("Failed to integrate linear expression as average_vel is "
                     "zero. Got ",
                     average_vel, "."));
  }
  return (path_end - path_start) / average_vel;
}

}  // namespace

// Integrates a general quadratic expression between `polynomial.range_start`
// and `polynomial.range_end` for the polynomial given by the coefficient vector
// `polynomial.coeffs`.
absl::StatusOr<double> IntegrateSquaredPathVelocityPolynomial(
    const QuadraticPolynomial& polynomial, bool truncate) {
  const double quadratic_coeff = polynomial.coeffs[0];
  const double ds = polynomial.range_end - polynomial.range_start;
  const double discriminant = ComputeDiscriminant(polynomial.coeffs);
  const double polynomial_value_at_start = SafeEvaluateQuadratic(
      polynomial.coeffs, discriminant, polynomial.range_start);
  const double polynomial_value_at_end = SafeEvaluateQuadratic(
      polynomial.coeffs, discriminant, polynomial.range_end);
  const double max_endpoint_polynomial_value =
      std::max({polynomial_value_at_start, polynomial_value_at_end, 0.0});

  // The secant shortcut requires a valid interval start, as neither branch can
  // integrate from a point where the polynomial is negative. A negative value
  // at the end of the interval is fine, because the linear integration
  // truncates at the zero crossing just like the quadratic ones do.
  const bool is_strictly_linear = (quadratic_coeff == 0.0);
  const bool is_relatively_linear =
      (polynomial_value_at_start >= -kInfinitesimal) &&
      (max_endpoint_polynomial_value > 0.0) &&
      (std::abs(quadratic_coeff) * ds * ds <=
       kRelativeQuadraticContributionThreshold * max_endpoint_polynomial_value);
  const bool is_effectively_linear = is_strictly_linear || is_relatively_linear;
  if (is_effectively_linear) {
    return IntegrateLinearExpression(polynomial.range_start,
                                     polynomial.range_end, polynomial.coeffs,
                                     discriminant, truncate);
  } else if (quadratic_coeff < 0.0) {
    return IntegrateConcaveQuadraticExpression(
        polynomial.range_start, polynomial.range_end, polynomial.coeffs,
        discriminant, truncate);
  }
  return IntegrateConvexQuadraticExpression(
      polynomial.range_start, polynomial.range_end, polynomial.coeffs,
      discriminant, truncate);
}

}  // namespace intrinsic::topp
