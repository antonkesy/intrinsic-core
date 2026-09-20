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

#include "intrinsic/motion_planning/trajectory_planning/topp/topp_analytical_polynomial_integrals.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <limits>
#include <optional>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/elliptic_integral.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomial_roots.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomials.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::topp {

namespace {

// Imaginary unit.
constexpr std::complex<double> kImaginaryUnit(0.0, 1.0);

// Represents the minimum and maximum bounds (`min_value` and `max_value`) in
// which a polynomial can be integrated in the form of the pseudo-equation
// (one over the square root of the polynomial value). This is computed
// based on the valid, non-negative domain of the polynomial between its roots.
struct PolynomialIntegrationBounds {
  double min_value = -std::numeric_limits<double>::infinity();
  double max_value = std::numeric_limits<double>::infinity();
};

// Returns true if the `value` is positive infinity.
bool IsPositiveInf(const double value) {
  return std::isinf(value) && value > 0.0;
}

// Returns true if the `value` is negative infinity.
bool IsNegativeInf(const double value) {
  return std::isinf(value) && value < 0.0;
}

// Finds the maximally overlapping `PolynomialIntegrationBounds` between the
// `polynomial`'s integration range and the possible integration ranges given by
// consecutive pairs of `real_roots`.
template <typename StructuredPolynomialType>
icon::RealtimeStatusOr<PolynomialIntegrationBounds>
TruncateBoundsForPolynomialIntegrationRange(
    const StructuredPolynomialType& polynomial,
    absl::Span<const double> real_roots) {
  double max_overlap = 0.0;
  std::optional<PolynomialIntegrationBounds> best_bounds;

  // Calculates the overlap of the possible integration `interval_bounds` with
  // the desired integration range [`polynomial.range_start`,
  // `polynomial.range_end`], and verifies its positivity.
  auto evaluate_interval_for_truncation =
      [&polynomial, &max_overlap,
       &best_bounds](const PolynomialIntegrationBounds& interval_bounds)
      -> icon::RealtimeStatus {
    // Calculate the intersection's length between the `interval_bounds` and the
    // polynomial's requested integration range.
    const double& min_val = interval_bounds.min_value;
    const double& max_val = interval_bounds.max_value;
    const double overlap_start = std::max(min_val, polynomial.range_start);
    const double overlap_end = std::min(max_val, polynomial.range_end);
    const double overlap = overlap_end - overlap_start;

    if (overlap > max_overlap) {
      // Find a test point inside the interval to check for positivity.
      double test_point = 0.0;
      if (IsNegativeInf(min_val) && IsPositiveInf(max_val)) {
        test_point = 0.5 * (polynomial.range_start + polynomial.range_end);
      } else if (IsNegativeInf(min_val)) {
        test_point = max_val - 1.0;
      } else if (IsPositiveInf(max_val)) {
        test_point = min_val + 1.0;
      } else {
        test_point = 0.5 * (min_val + max_val);
      }

      // If the `polynomial_evaluation` is strictly positive, this becomes the
      // best candidate bounds found so far.
      INTRINSIC_RT_ASSIGN_OR_RETURN(const double polynomial_evaluation,
                                    polynomial.EvaluatePolynomial(test_point));
      if (polynomial_evaluation > 0.0) {
        max_overlap = overlap;
        best_bounds = interval_bounds;
      }
    }
    return icon::OkStatus();
  };

  // Test all possible intervals defined by the roots
  const int real_roots_count = real_roots.size();
  if (real_roots_count == 0) {
    INTRINSIC_RT_RETURN_IF_ERROR(evaluate_interval_for_truncation(
        {-std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity()}));
  } else {
    INTRINSIC_RT_RETURN_IF_ERROR(evaluate_interval_for_truncation(
        {-std::numeric_limits<double>::infinity(), real_roots.front()}));
    for (int i = 1; i < real_roots_count; ++i) {
      INTRINSIC_RT_RETURN_IF_ERROR(
          evaluate_interval_for_truncation({real_roots[i - 1], real_roots[i]}));
    }
    INTRINSIC_RT_RETURN_IF_ERROR(evaluate_interval_for_truncation(
        {real_roots.back(), std::numeric_limits<double>::infinity()}));
  }

  // Return the `best_bounds` found if they are valid.
  if (best_bounds.has_value()) {
    return *best_bounds;
  }

  return icon::InvalidArgumentError(
      "Truncation failed: the integration range does not overlap with any "
      "positive region of the polynomial.");
}

// Computes the `PolynomialIntegrationBounds` for the given structured
// `polynomial` (supports both Quadratic and Cubic variants).
template <typename StructuredPolynomialType>
icon::RealtimeStatusOr<PolynomialIntegrationBounds>
ComputeBoundsForPolynomialIntegrationRange(
    const StructuredPolynomialType& polynomial, const bool truncate = false) {
  // Only real roots matter to discriminate positive and negative ranges of a
  // polynomial, as complex roots do not imply a zero crossing. Use a fixed
  // array size of `kNumRootsCubicPolynomial`, which safely covers both
  // quadratic and cubic polynomials.
  std::array<double, kNumRootsCubicPolynomial> real_roots;
  int real_roots_count = 0;
  for (int id = 0; id < polynomial.structured_roots.size(); ++id) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot& root,
                                  polynomial.structured_roots.Get(id));
    if (root.type == RootType::kReal) {
      real_roots[real_roots_count] = root.value.real();
      ++real_roots_count;
    }
  }

  // Returns true if the `polynomial.range_start` and
  // `polynomial.range_end` values are within the `bounds` with
  // the given `kRangeTolerance`.
  const double kRangeTolerance = 1.0e-4;
  auto are_evaluation_points_in_range =
      [&](const PolynomialIntegrationBounds& bounds) -> bool {
    return polynomial.range_start >= bounds.min_value - kRangeTolerance &&
           polynomial.range_start <= bounds.max_value + kRangeTolerance &&
           polynomial.range_end >= bounds.min_value - kRangeTolerance &&
           polynomial.range_end <= bounds.max_value + kRangeTolerance;
  };

  if (!truncate) {
    PolynomialIntegrationBounds bounds_for_ranges;
    if (real_roots_count == 0) {
      // Complex roots imply no zero crossings on the real line. This check is
      // only valid for a quadratic polynomial, as a cubic polynomial with real
      // coefficients always has a real root.
      if (are_evaluation_points_in_range(bounds_for_ranges)) {
        return bounds_for_ranges;
      }
    } else {
      bounds_for_ranges.max_value = real_roots.front();
      if (are_evaluation_points_in_range(bounds_for_ranges)) {
        return bounds_for_ranges;
      }
      for (int i = 1; i < real_roots_count; ++i) {
        bounds_for_ranges.min_value = real_roots[i - 1];
        bounds_for_ranges.max_value = real_roots[i];
        if (are_evaluation_points_in_range(bounds_for_ranges)) {
          return bounds_for_ranges;
        }
      }
      bounds_for_ranges = PolynomialIntegrationBounds{
          .min_value = real_roots[real_roots_count - 1],
          .max_value = std::numeric_limits<double>::infinity()};
      if (are_evaluation_points_in_range(bounds_for_ranges)) {
        return bounds_for_ranges;
      }
    }

    return icon::InternalError(
        "Negative polynomial between or on an evaluation point (`range_start` "
        "and `range_end`).");
  }

  // In the case of truncation, we find a valid range with maximum overlap over
  // the desired polynomial's integration range.
  return TruncateBoundsForPolynomialIntegrationRange(
      polynomial, absl::MakeConstSpan(real_roots.data(), real_roots_count));
}

// Returns the input `value` clamped to the range [`bounds.min_value`,
// `bounds.max_value`] with a numerical `tolerance`. This clamping is needed to
// guarantee that the integrals are not evaluated exactly at the roots and thus
// lead to a division by zero for example.
double ClampValue(
    double value, const PolynomialIntegrationBounds& bounds,
    const double tolerance = std::numeric_limits<double>::epsilon()) {
  return std::clamp(value, bounds.min_value + tolerance,
                    bounds.max_value - tolerance);
}

// This check makes sure that the evaluation interval for the integration of the
// polynomial is valid. To that end, it evaluates that the `polynomial` is
// positive at the middle between `polynomial.range_start` and
// `polynomial.range_end` (clamped to the `bounds`). If not, it returns an
// error.
template <typename StructuredPolynomialType>
icon::RealtimeStatus CheckPolynomialAtMiddleRangePointIsPositive(
    const StructuredPolynomialType& polynomial,
    const PolynomialIntegrationBounds& bounds) {
  const double clamped_start = ClampValue(polynomial.range_start, bounds);
  const double clamped_end = ClampValue(polynomial.range_end, bounds);
  const double range_middle_point = 0.5 * (clamped_start + clamped_end);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double polynomial_value,
      polynomial.EvaluatePolynomial(range_middle_point));

  if (polynomial_value <= 0.0) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Negative polynomial at the `range_middle_point` ", range_middle_point,
        " with value ", polynomial_value, "."));
  }
  return icon::OkStatus();
}

std::complex<double> ComputeFactorNumerator(const double x,
                                            const std::complex<double>& root0,
                                            const std::complex<double>& root1,
                                            const std::complex<double>& root2) {
  return (2.0 * (x - root2) *
          std::sqrt(std::complex<double>((root0 - x) / (root0 - root2))) *
          std::sqrt(std::complex<double>((root1 - x) / (root1 - root2))));
}

std::complex<double> ComputeFactorDenominator(
    const double x, const std::complex<double>& root1,
    const std::complex<double>& root2, const eigenmath::Vector4d& coeffs) {
  return (std::sqrt(coeffs[0] * ::intrinsic::IPow(x, 3) +
                    coeffs[1] * ::intrinsic::IPow(x, 2) + coeffs[2] * x +
                    coeffs[3]) *
          std::sqrt(std::complex<double>((x - root2) / (root1 - root2))));
}

std::complex<double> ComputeEllipticIntegralFactor(
    const double x, const std::complex<double>& root0,
    const std::complex<double>& root1, const std::complex<double>& root2,
    const eigenmath::Vector4d& coeffs) {
  return ComputeFactorNumerator(x, root0, root1, root2) /
         ComputeFactorDenominator(x, root1, root2, coeffs);
}

// Sanity check that the `definite_integral` contains a valid number.
icon::RealtimeStatus SanityCheckForDefiniteIntegral(
    const double definite_integral) {
  if (std::isnan(definite_integral) || std::isinf(definite_integral)) {
    // This should never happen, but it is just a safety check.
    return icon::InternalError(
        "Finiteness check for the definite integral failed.");
  }
  return icon::OkStatus();
}

// Computes the inverse of the tangent function for a complex number `z`.
std::complex<double> ArcTangent(const std::complex<double>& z) {
  return 0.5 * kImaginaryUnit *
         (std::log(1.0 - kImaginaryUnit * z) -
          std::log(1.0 + kImaginaryUnit * z));
}

// Computes the tangent function for a complex number `z`.
std::complex<double> Tangent(const std::complex<double>& z) {
  const std::complex<double> exp_iz = std::exp(kImaginaryUnit * z);
  const std::complex<double> exp_neg_iz = std::exp(-kImaginaryUnit * z);
  return -kImaginaryUnit * ((exp_iz - exp_neg_iz) / (exp_iz + exp_neg_iz));
}

}  // namespace

namespace one_over_sqrt_of_linear_polynomial {

// Computes the integral for a linear expression. It constructs the integral for
// the following expression:
//         range_end                    1.0
//   integral         ----------------------------------------- ds
//     s = range_start sqrt(linear_coeff * s + constant_coeff)
icon::RealtimeStatusOr<double> IntegrateOneOverSqrtOfLinearPolynomial(
    const double range_start, const double range_end, const double linear_coeff,
    const double constant_coeff) {
  if (range_start > range_end) {
    return icon::InvalidArgumentError(
        "The `range_start` must be <= than `range_end`.");
  }
  const double b = linear_coeff;
  const double c = constant_coeff;

  // Tolerance to allow for numerical inaccuracies near zero.
  const double kInfinitesimal = 1e-12;

  const double squared_sd_start = b * range_start + c;
  if (squared_sd_start < -kInfinitesimal) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Failed to integrate linear expression as `squared_sd_start` is "
        "below zero. Got ",
        squared_sd_start, "."));
  }
  const double sd_start = std::sqrt(std::max(squared_sd_start, 0.0));

  const double squared_sd_end = b * range_end + c;
  if (squared_sd_end < -kInfinitesimal) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Failed to integrate linear expression as `squared_sd_end` is "
        "below zero. Got ",
        squared_sd_end, "."));
  }
  const double sd_end = std::sqrt(std::max(squared_sd_end, 0.0));

  const double average_vel = 0.5 * (sd_start + sd_end);
  if (average_vel <= 0.0) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Failed to integrate linear expression as `average_vel` is zero. Got ",
        average_vel, "."));
  }
  return (range_end - range_start) / average_vel;
}

}  // namespace one_over_sqrt_of_linear_polynomial

namespace one_over_sqrt_of_quadratic_polynomial {

// Evaluates the indefinite integral: int 1 / sqrt(a * (x - r)^2) dx.
//
// Mathematically, the integrand simplifies to 1 / (sqrt(a) * |x - r|).
// Because a square root represents a positive distance, the absolute value
// causes the antiderivative to branch depending on which side of the root
// `x` is located:
//   For x > r:  +ln(x - r) / sqrt(a)
//   For x < r:  -ln(r - x) / sqrt(a)
//
// This implementation computes both branches compactly using a sign multiplier:
// sgn(x - r) * ln(|x - r|) / sqrt(a). This function assumes `a > 0`. Otherwise,
// the integral would be undefined.
//
// Note: The integral diverges at the asymptote x = r. Evaluating this function
// exactly at the root will attempt to compute ln(0) and result in -infinity.
//
// - `a` is the scaling coefficient of the quadratic polynomial. Must be
//   strictly positive (a > 0.0) for the integral to be real.
// - `r` is the single real root of multiplicity two.
// - `x` is the point along the x-axis at which to evaluate the integral.
// Returns the evaluated indefinite integral at point `x`.
icon::RealtimeStatusOr<double> IndefiniteIntegralRealRootWithMultiplicityTwo(
    const double a, const double r, const double x) {
  // Mathematically, if a <= 0, the polynomial a*(x-r)^2 is <= 0 everywhere,
  // making 1/sqrt(P(x)) undefined or imaginary.
  if (a <= 0.0) {
    return icon::InvalidArgumentError(
        "Scaling coefficient must be strictly positive for a quadratic with "
        "a repeated real root to yield a valid real integral.");
  }
  const double sgn = (x >= r) ? 1.0 : -1.0;
  return sgn * std::log(std::abs(x - r)) / std::sqrt(a);
}

// Evaluates the indefinite integral: int 1 / sqrt(a * (x - r1) * (x - r2)) dx.
//
// This computes the antiderivative for a quadratic polynomial with two distinct
// real roots. The geometric shape of the parabola dictates the integration
// formula:
//
// 1. When a > 0 (Convex Parabola):
//    The polynomial is positive outside the roots. The integral evaluates to a
//    logarithmic form:
//      `(1 / sqrt(a)) * ln|2x - r1 - r2 + 2*sqrt((x-r1)(x-r2))|`.
//
// 2. When a < 0 (Concave Parabola):
//    The polynomial is positive only between the roots. The integral evaluates
//    to an inverse trigonometric form:
//      `(1 / sqrt(-a)) * arcsin((2x - r1 - r2) / (r2 - r1))`.
//    This mathematically relies on the roots being distinct and sorted (r1 <
//    r2) so that the denominator (r2 - r1) is strictly positive.
//
// - `a` is the scaling coefficient of the quadratic polynomial and is expected
//   to be non-zero as it is the leading coefficient of a quadratic function.
// - `r1` is the first real root (assumed to be the smaller root, r1 < r2).
// - `r2` is the second real root (assumed to be the larger root).
// - `x` is the point along the x-axis at which to evaluate the integral.
// Returns the evaluated indefinite integral at point `x`.
icon::RealtimeStatusOr<double>
IndefiniteIntegralOnlyRealRootsWithMultiplicityOne(const double a,
                                                   const double r1,
                                                   const double r2,
                                                   const double x) {
  // Mathematically, a == 0 makes it a linear polynomial, not quadratic.
  // We reject this to avoid division by zero in the indefinite integrals.
  if (AlmostEquals(a, 0.0)) {
    return icon::InvalidArgumentError(
        "Scaling coefficient must be non-zero for a valid quadratic integral.");
  }

  if (a > 0.0) {
    const double term = std::max((x - r1) * (x - r2), 0.0);
    return (1.0 / std::sqrt(a)) *
           std::log(std::abs(2.0 * x - r1 - r2 + 2.0 * std::sqrt(term)));
  } else {
    const double arg = std::clamp((2.0 * x - r1 - r2) / (r2 - r1), -1.0, 1.0);
    return (1.0 / std::sqrt(-a)) * std::asin(arg);
  }
}

// Evaluates the indefinite integral: int 1 / sqrt(a * ((x - alpha)^2 + beta^2))
// dx.
//
// This computes the antiderivative for a quadratic polynomial with complex
// conjugate roots. Through trigonometric substitution, the integral evaluates
// to: (1 / sqrt(a)) * ln((x - alpha) + sqrt((x - alpha)^2 + beta^2)).
//
// We assume `a > 0`. For a quadratic with complex roots, if `a <= 0`, the
// parabola is submerged entirely below the x-axis. The polynomial would be
// strictly negative everywhere, making a real-valued integral impossible.
// If `a <= 0` an error is returned.
//
// Unlike the real-roots implementations, this formula does not require
// an absolute value inside the logarithm. Because `beta^2` is strictly
// positive, the square root term is mathematically guaranteed to be larger than
// `|x - alpha|`. Thus, the argument to logarithmic function is strictly
// positive for all real values of `x`.
//
// - `a` is the scaling coefficient of the quadratic polynomial (must be > 0).
// - `alpha` is the real part of the complex conjugate roots.
// - `beta` is the imaginary part (magnitude) of the complex conjugate roots.
// - `x` is the point along the x-axis at which to evaluate the integral.
// Returns the evaluated indefinite integral at point `x`.
icon::RealtimeStatusOr<double> IndefiniteIntegralComplexConjugateRoots(
    const double a, const double alpha, const double beta, const double x) {
  if (a <= 0.0) {
    return icon::InvalidArgumentError(
        "Scaling coefficient must be strictly positive for a quadratic with "
        "complex roots to yield a valid real integral.");
  }

  const double term = (x - alpha) * (x - alpha) + beta * beta;
  return (1.0 / std::sqrt(a)) * std::log(x - alpha + std::sqrt(term));
}

icon::RealtimeStatusOr<double> IntegrateRealRootWithMultiplicityTwo(
    const StructuredQuadraticPolynomial& polynomial,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                polynomial.structured_roots.Get(0));
  const double root_r = root0.value.real();

  const double clamped_range_start =
      ClampValue(polynomial.range_start, bounds_for_ranges);
  const double clamped_range_end =
      ClampValue(polynomial.range_end, bounds_for_ranges);

  const double a = polynomial.scaling_coefficient;
  INTRINSIC_RT_ASSIGN_OR_RETURN(const double start_val,
                                IndefiniteIntegralRealRootWithMultiplicityTwo(
                                    a, root_r, clamped_range_start));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const double end_val,
                                IndefiniteIntegralRealRootWithMultiplicityTwo(
                                    a, root_r, clamped_range_end));

  return std::abs(end_val - start_val);
}

icon::RealtimeStatusOr<double> IntegrateOnlyRealRootsWithMultiplicityOne(
    const StructuredQuadraticPolynomial& polynomial,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                polynomial.structured_roots.Get(0));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root1,
                                polynomial.structured_roots.Get(1));
  const double root_r1 = root0.value.real();
  const double root_r2 = root1.value.real();

  const double clamped_range_start =
      ClampValue(polynomial.range_start, bounds_for_ranges);
  const double clamped_range_end =
      ClampValue(polynomial.range_end, bounds_for_ranges);

  const double a = polynomial.scaling_coefficient;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double start_val,
      IndefiniteIntegralOnlyRealRootsWithMultiplicityOne(a, root_r1, root_r2,
                                                         clamped_range_start));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double end_val, IndefiniteIntegralOnlyRealRootsWithMultiplicityOne(
                                a, root_r1, root_r2, clamped_range_end));

  return std::abs(end_val - start_val);
}

icon::RealtimeStatusOr<double> IntegrateComplexConjugateRoots(
    const StructuredQuadraticPolynomial& polynomial,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                polynomial.structured_roots.Get(0));
  const double alpha = root0.value.real();
  const double beta = std::abs(root0.value.imag());

  const double clamped_range_start =
      ClampValue(polynomial.range_start, bounds_for_ranges);
  const double clamped_range_end =
      ClampValue(polynomial.range_end, bounds_for_ranges);

  const double a = polynomial.scaling_coefficient;
  INTRINSIC_RT_ASSIGN_OR_RETURN(const double start_val,
                                IndefiniteIntegralComplexConjugateRoots(
                                    a, alpha, beta, clamped_range_start));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const double end_val,
                                IndefiniteIntegralComplexConjugateRoots(
                                    a, alpha, beta, clamped_range_end));

  return std::abs(end_val - start_val);
}

}  // namespace one_over_sqrt_of_quadratic_polynomial

namespace one_over_sqrt_of_cubic_polynomial {

// Represents the result of the integration of the square root of a cubic
// polynomial. It contains the `integral_value` (result of the integration) and
// also the `branch_cut_factor`, which is a power of the real or imaginary unit.
// It is a sign change happening due to the handling of square roots of possibly
// negative numbers (leading to powers of the imaginary unit).
struct IntegralResult {
  std::complex<double> integral_value;
  std::complex<double> branch_cut_factor;
};

// Helper function to compute the indefinite integral of a cubic polynomial with
// a single real root of multiplicity three. It does not compute the constant
// factor of the integral, as it is assumed that it will be used in the context
// of a definite integral, for which that is not required. The coefficient `a`
// is the leading coefficient of the cubic polynomial. The `root` is the single
// real root of multiplicity three of the cubic polynomial. `x` is the
// evaluation point for the indefinite integral. It represents the polynomial:
//  a * (x - p)^3, where `p` is the real root of multiplicity three.
inline IntegralResult IndefiniteIntegralSingleRealRootWithMultiplicityThree(
    const double a, const double p, const double x) {
  const double branch_cut_factor = (x - p >= 0.0 ? 1.0 : -1.0);
  return IntegralResult{
      .integral_value = -2.0 * branch_cut_factor / std::sqrt(a * (x - p)),
      .branch_cut_factor = branch_cut_factor};
}

// Helper function to compute the indefinite integral of a cubic polynomial with
// a real root of multiplicity two. It does not compute the constant factor of
// the integral, as it is assumed that it will be used in the context of a
// definite integral, for which that is not required. The coefficient `a` is the
// leading coefficient of the cubic polynomial. It represents the polynomial:
//  a * (x - p) * (x - q)^2, where `q` is the real root of multiplicity two and
//  `p` is the real root of multiplicity one.
inline IntegralResult IndefiniteIntegralRealRootWithMultiplicityTwo(
    const double a, const double p, const double q, const double x) {
  const std::complex<double> x_minus_p = x - p;
  const std::complex<double> p_minus_q = p - q;

  const double branch_cut_factor = (q - x >= 0.0 ? 1.0 : -1.0);
  return IntegralResult{
      .integral_value = -2.0 * branch_cut_factor / std::sqrt(a * p_minus_q) *
                        ArcTangent(std::sqrt(x_minus_p) / std::sqrt(p_minus_q)),
      .branch_cut_factor = branch_cut_factor};
}

// Helper function to compute the indefinite integral of a cubic polynomial with
// only real roots of multiplicity one. It does not compute the constant factor
// of the integral, as it is assumed that it will be used in the context of a
// definite integral, for which that is not required. The coefficient `a` is the
// leading coefficient of the cubic polynomial. The `p`, `q`, `r` are the real
// roots of multiplicity one of the cubic polynomial. `x` is the evaluation
// point for the indefinite integral. It represents the polynomial:
//  a * (x - p) * (x - q) * (x - r), where p < q < r.
inline icon::RealtimeStatusOr<IntegralResult>
IndefiniteIntegralOnlyRealRootsWithMultiplicityOne(const std::complex<double> a,
                                                   const std::complex<double> p,
                                                   const std::complex<double> q,
                                                   const std::complex<double> r,
                                                   const double x) {
  const std::complex<double> jacobi_amplitude =
      std::asin(std::sqrt(q - p) / std::sqrt(x - p));
  const std::complex<double> elliptic_modulus = (p - r) / (p - q);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const std::complex<double> elliptic_integral,
      EllipticIntegralOfTheFirstKind(jacobi_amplitude, elliptic_modulus));

  const std::complex<double> factor = -2.0 / std::sqrt(a * (q - p));
  std::complex<double> branch_cut_factor = 1.0;
  branch_cut_factor *= (x - p).real() >= 0.0 ? 1.0 : -kImaginaryUnit;
  branch_cut_factor *= ((q - x) / (p - x)).real() >= 0.0 ? 1.0 : kImaginaryUnit;
  branch_cut_factor *= ((r - x) / (p - x)).real() >= 0.0 ? 1.0 : kImaginaryUnit;
  branch_cut_factor /=
      ((p - x) * (q - x) * (x - r)).real() >= 0.0 ? 1.0 : kImaginaryUnit;

  return IntegralResult{
      .integral_value = factor * branch_cut_factor * elliptic_integral,
      .branch_cut_factor = branch_cut_factor};
}

// Helper function to compute the indefinite integral of a cubic polynomial with
// a single real root of multiplicity one and two complex conjugate roots. It
// does not compute the constant factor of the integral, as it is assumed that
// it will be used in the context of a definite integral, for which that is not
// required. The coefficient `a` is the leading coefficient of the cubic
// polynomial. The root `p` is the single real root of multiplicity one of
// the cubic polynomial. The roots `q` and `r` are the two complex conjugate
// roots of the cubic polynomial sorted in ascending order based on their
// imaginary part. `x` is the evaluation point for the indefinite integral. It
// represents the polynomial: a * (x - p) * (x - q) * (x - r),
//   where p is real and q, r are complex conjugate roots.
inline icon::RealtimeStatusOr<IntegralResult>
IndefiniteIntegralSingleRealRootAndTwoComplexConjugateRoots(
    const std::complex<double> a, const std::complex<double> p,
    const std::complex<double> q, const std::complex<double> r, const double x,
    const double polynomial_at_x) {
  const std::complex<double> jacobi_amplitude =
      std::asin(kImaginaryUnit * std::sqrt(q - p) / std::sqrt(p - x));
  const std::complex<double> elliptic_modulus = (r - p) / (q - p);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const std::complex<double> elliptic_integral,
      EllipticIntegralOfTheFirstKind(jacobi_amplitude, elliptic_modulus));

  const std::complex<double> factor =
      -2.0 * kImaginaryUnit / std::sqrt(a * (q - p));
  std::complex<double> branch_cut_factor = (x - p).real() >= 0.0 ? 1.0 : -1.0;
  branch_cut_factor *= (p - x).real() >= 0.0 ? 1.0 : -kImaginaryUnit;
  branch_cut_factor /= (x - p).real() >= 0.0 ? 1.0 : kImaginaryUnit;

  return IntegralResult{
      .integral_value = factor * branch_cut_factor * elliptic_integral,
      .branch_cut_factor = branch_cut_factor};
}

// Computes the integral of a cubic polynomial in the range
// [`polynomial.range_start`, `polynomial.range_end`] which will be clamped to
// the range given by `bounds_for_ranges`. This function assumes that the
// polynomial has a single real root with multiplicity three. It computes the
// following integral:
//       range_end               1
// Integral =          ------------------------ dx
//       range_start    sqrt(a * (x - root)^3)
icon::RealtimeStatusOr<double> IntegrateSingleRealRootWithMultiplicityThree(
    const StructuredCubicPolynomial& polynomial,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  const double a = polynomial.scaling_coefficient;
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                polynomial.structured_roots.Get(0));
  const double root = root0.value.real();

  const double clamped_range_start =
      ClampValue(polynomial.range_start, bounds_for_ranges);
  const double clamped_range_end =
      ClampValue(polynomial.range_end, bounds_for_ranges);
  const IntegralResult indefinite_integral_start =
      IndefiniteIntegralSingleRealRootWithMultiplicityThree(
          a, root, clamped_range_start);
  const IntegralResult indefinite_integral_end =
      IndefiniteIntegralSingleRealRootWithMultiplicityThree(a, root,
                                                            clamped_range_end);
  return std::abs(indefinite_integral_end.integral_value -
                  indefinite_integral_start.integral_value);
}

// Computes the integral of a cubic polynomial in the range
// [`polynomial.range_start`, `polynomial.range_end`] which will be clamped to
// the range given by `bounds_for_ranges`. It assumes that the `polynomial` has
// a real root of multiplicity 2 named `root_q` and a root of multiplicity 1
// named `root_p`. Thus, it integrates the polynomial:
//       range_end                         1
// Integral =          ----------------------------------------- dx
//       range_start    sqrt(a * (x - root_p) * (x - root_q)^2)
icon::RealtimeStatusOr<double> IntegrateRealRootWithMultiplicityTwo(
    const StructuredCubicPolynomial& polynomial,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                polynomial.structured_roots.Get(0));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root1,
                                polynomial.structured_roots.Get(1));
  // Converting roots and scaling coefficient to complex numbers to compute the
  // inverse of the tangent function or the square root of a negative number.
  const double a = polynomial.scaling_coefficient;
  const double root_p =
      (root0.multiplicity == 1 ? root0.value.real() : root1.value.real());
  const double root_q =
      (root0.multiplicity == 2 ? root0.value.real() : root1.value.real());

  const double clamped_range_start =
      ClampValue(polynomial.range_start, bounds_for_ranges);
  const double clamped_range_end =
      ClampValue(polynomial.range_end, bounds_for_ranges);
  const IntegralResult indefinite_integral_start =
      IndefiniteIntegralRealRootWithMultiplicityTwo(a, root_p, root_q,
                                                    clamped_range_start);
  const IntegralResult indefinite_integral_end =
      IndefiniteIntegralRealRootWithMultiplicityTwo(a, root_p, root_q,
                                                    clamped_range_end);
  return std::abs(indefinite_integral_end.integral_value -
                  indefinite_integral_start.integral_value);
}

// Computes the integral of a cubic polynomial in the range
// [`polynomial.range_start`, `polynomial.range_end`] which will be clamped to
// the range given by `bounds_for_ranges`. It assumes that the `polynomial` has
// only real roots of multiplicity 1 (`root_p`, `root_q`, `root_r`). Thus, it
// integrates the polynomial:
//       range_end                              1
// Integral =          ---------------------------------------------------- dx
//       range_start   sqrt(a * (x - root_p) * (x - root_q) * (x - root_r))
icon::RealtimeStatusOr<double> IntegrateOnlyRealRootsWithMultiplicityOne(
    const StructuredCubicPolynomial& polynomial,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  // Converting roots and scaling coefficient to complex numbers to more easily
  // compute the square root of their differences which can be negative numbers.
  const std::complex<double> a = polynomial.scaling_coefficient;
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root_p,
                                polynomial.structured_roots.Get(0));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root_q,
                                polynomial.structured_roots.Get(1));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root_r,
                                polynomial.structured_roots.Get(2));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const IntegralResult indefinite_integral_start,
      IndefiniteIntegralOnlyRealRootsWithMultiplicityOne(
          a.real(), root_p.value.real(), root_q.value.real(),
          root_r.value.real(),
          ClampValue(polynomial.range_start, bounds_for_ranges)));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const IntegralResult indefinite_integral_end,
      IndefiniteIntegralOnlyRealRootsWithMultiplicityOne(
          a.real(), root_p.value.real(), root_q.value.real(),
          root_r.value.real(),
          ClampValue(polynomial.range_end, bounds_for_ranges)));
  return std::abs(indefinite_integral_end.integral_value -
                  indefinite_integral_start.integral_value);
}

// Computes the integral of a cubic polynomial in the range
// [`polynomial.range_start`, `polynomial.range_end`] which will be clamped to
// the range given by `bounds_for_ranges`. It assumes that the `polynomial` has
// one real root and two complex conjugate roots. Thus, it integrates:
//       range_end                              1
// Integral =          --------------------------------------- dx
//       range_start   sqrt(a * (x - real_root)
//                            * (x - complex_root)
//                            * (x - complex_conjugate_root))
icon::RealtimeStatusOr<double>
IntegrateSingleRealRootAndTwoComplexConjugateRoots(
    const StructuredCubicPolynomial& polynomial,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  const std::complex<double> a = polynomial.scaling_coefficient;
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                polynomial.structured_roots.Get(0));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root1,
                                polynomial.structured_roots.Get(1));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root2,
                                polynomial.structured_roots.Get(2));

  const bool is_real_root_first = root0.type == RootType::kReal;
  const std::complex<double> real_root =
      (is_real_root_first ? root0.value.real() : root2.value.real());
  const std::complex<double> complex_root_0 =
      (is_real_root_first ? root1.value : root0.value);
  const std::complex<double> complex_root_1 =
      (is_real_root_first ? root2.value : root1.value);

  const double clamped_range_start =
      ClampValue(polynomial.range_start, bounds_for_ranges);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double polynomial_at_range_start,
      polynomial.EvaluatePolynomial(clamped_range_start));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const IntegralResult indefinite_integral_start,
      IndefiniteIntegralSingleRealRootAndTwoComplexConjugateRoots(
          a.real(), real_root.real(), complex_root_0, complex_root_1,
          clamped_range_start, polynomial_at_range_start));

  const double clamped_range_end =
      ClampValue(polynomial.range_end, bounds_for_ranges);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double polynomial_at_range_end,
      polynomial.EvaluatePolynomial(clamped_range_end));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const IntegralResult indefinite_integral_end,
      IndefiniteIntegralSingleRealRootAndTwoComplexConjugateRoots(
          a.real(), real_root.real(), complex_root_0, complex_root_1,
          clamped_range_end, polynomial_at_range_end));

  return std::abs(indefinite_integral_end.integral_value -
                  indefinite_integral_start.integral_value);
}

icon::RealtimeStatusOr<double> UpperLimitForSingleRealRootWithMultiplicityThree(
    const StructuredCubicPolynomial& polynomial, const double integral,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  const double a = polynomial.scaling_coefficient;
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                polynomial.structured_roots.Get(0));
  const double root = root0.value.real();

  const IntegralResult indefinite_integral_start =
      IndefiniteIntegralSingleRealRootWithMultiplicityThree(
          a, root, ClampValue(polynomial.range_start, bounds_for_ranges));
  const std::complex<double> integral_diff =
      (integral + indefinite_integral_start.integral_value.real()) /
      indefinite_integral_start.branch_cut_factor;
  const std::complex<double> upper_limit =
      root + 1.0 / a * ::intrinsic::IPow(-2.0 / integral_diff, 2);
  return upper_limit.real();
}

icon::RealtimeStatusOr<double> UpperLimitForRealRootWithMultiplicityTwo(
    const StructuredCubicPolynomial& polynomial, const double integral,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                polynomial.structured_roots.Get(0));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root1,
                                polynomial.structured_roots.Get(1));
  // Converting roots and scaling coefficient to complex numbers to compute the
  // inverse of the tangent function or the square root of a negative number.
  const double a = polynomial.scaling_coefficient;
  const double root_p =
      (root0.multiplicity == 1 ? root0.value.real() : root1.value.real());
  const double root_q =
      (root0.multiplicity == 2 ? root0.value.real() : root1.value.real());

  const double clamped_range_start =
      ClampValue(polynomial.range_start, bounds_for_ranges);
  const IntegralResult indefinite_integral_start =
      IndefiniteIntegralRealRootWithMultiplicityTwo(a, root_p, root_q,
                                                    clamped_range_start);
  const std::complex<double> integral_diff =
      (integral + indefinite_integral_start.integral_value) /
      indefinite_integral_start.branch_cut_factor;
  const std::complex<double> p_minus_q = root_p - root_q;
  const std::complex<double> coeff = -2.0 / (std::sqrt(a * p_minus_q));
  const std::complex<double> tangent = Tangent(integral_diff / coeff);
  const std::complex<double> upper_limit =
      root_p + (root_p - root_q) * ::intrinsic::IPow(tangent, 2);
  return upper_limit.real();
}

icon::RealtimeStatusOr<double> UpperLimitForOnlyRealRootsWithMultiplicityOne(
    const StructuredCubicPolynomial& polynomial, const double integral,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  // Converting roots and scaling coefficient to complex numbers to more easily
  // compute the square root of their differences which can be negative numbers.
  const double a = polynomial.scaling_coefficient;
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                polynomial.structured_roots.Get(0));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root1,
                                polynomial.structured_roots.Get(1));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root2,
                                polynomial.structured_roots.Get(2));
  const std::complex<double> root_p = root0.value;
  const std::complex<double> root_q = root1.value;
  const std::complex<double> root_r = root2.value;

  const std::complex<double> q_minus_p = root_q - root_p;
  const std::complex<double> sqrt_q_minus_p = std::sqrt(q_minus_p);
  const std::complex<double> coeff = -2.0 / (std::sqrt(a * q_minus_p));
  const std::complex<double> elliptic_modulus =
      (root_p - root_r) / (root_p - root_q);

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const IntegralResult indefinite_integral_start,
      IndefiniteIntegralOnlyRealRootsWithMultiplicityOne(
          a, root_p, root_q, root_r,
          ClampValue(polynomial.range_start, bounds_for_ranges)));

  const std::complex<double> integral_diff =
      (integral + indefinite_integral_start.integral_value) /
      indefinite_integral_start.branch_cut_factor;
  const std::complex<double> elliptic_integral_at_upper_limit =
      integral_diff / coeff;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      std::complex<double> jacobi_amplitude,
      JacobiAmplitude(elliptic_integral_at_upper_limit, elliptic_modulus));
  const std::complex<double> upper_limit =
      root_p +
      ::intrinsic::IPow(sqrt_q_minus_p / std::sin(jacobi_amplitude), 2);
  return upper_limit.real();
}

icon::RealtimeStatusOr<double>
UpperLimitForSingleRealRootAndTwoComplexConjugateRoots(
    const StructuredCubicPolynomial& polynomial, const double integral,
    const PolynomialIntegrationBounds& bounds_for_ranges) {
  const std::complex<double> a = polynomial.scaling_coefficient;
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                polynomial.structured_roots.Get(0));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root1,
                                polynomial.structured_roots.Get(1));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root2,
                                polynomial.structured_roots.Get(2));

  const bool is_real_root_first = root0.type == RootType::kReal;
  const std::complex<double> real_root =
      (is_real_root_first ? root0.value.real() : root2.value.real());
  const std::complex<double> complex_root_0 =
      (is_real_root_first ? root1.value : root0.value);
  const std::complex<double> complex_root_1 =
      (is_real_root_first ? root2.value : root1.value);

  const std::complex<double> complex_0_minus_real = complex_root_0 - real_root;
  const std::complex<double> sqrt_complex_0_minus_real =
      std::sqrt(complex_0_minus_real);

  const std::complex<double> coeff =
      -2.0 * kImaginaryUnit / (std::sqrt(a) * sqrt_complex_0_minus_real);
  const std::complex<double> elliptic_modulus =
      (complex_root_1 - real_root) / (complex_0_minus_real);

  const double clamped_range_start =
      ClampValue(polynomial.range_start, bounds_for_ranges);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double polynomial_at_clamped_range_start,
      polynomial.EvaluatePolynomial(clamped_range_start));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const IntegralResult indefinite_integral_start,
      IndefiniteIntegralSingleRealRootAndTwoComplexConjugateRoots(
          a.real(), real_root.real(), complex_root_0, complex_root_1,
          clamped_range_start, polynomial_at_clamped_range_start));

  const std::complex<double> integral_diff =
      (integral + indefinite_integral_start.integral_value) /
      indefinite_integral_start.branch_cut_factor;
  const std::complex<double> elliptic_integral_at_upper_limit =
      integral_diff / coeff;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const std::complex<double> jacobi_amplitude,
      JacobiAmplitude(elliptic_integral_at_upper_limit, elliptic_modulus));
  std::complex<double> upper_limit =
      real_root -
      ::intrinsic::IPow((kImaginaryUnit * sqrt_complex_0_minus_real) /
                            std::sin(jacobi_amplitude),
                        2);
  return upper_limit.real();
}

}  // namespace one_over_sqrt_of_cubic_polynomial

icon::RealtimeStatusOr<double>
IntegrateOneOverSqrtOfStructuredQuadraticPolynomial(
    const StructuredQuadraticPolynomial& structured_polynomial,
    const bool truncate) {
  INTRINSIC_RT_RETURN_IF_ERROR(structured_polynomial.IsValid());
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const PolynomialIntegrationBounds bounds_for_ranges,
      ComputeBoundsForPolynomialIntegrationRange(structured_polynomial,
                                                 truncate));
  INTRINSIC_RT_RETURN_IF_ERROR(CheckPolynomialAtMiddleRangePointIsPositive(
      structured_polynomial, bounds_for_ranges));

  // Handle the different structured polynomials' integration cases.
  double definite_integral = std::numeric_limits<double>::quiet_NaN();
  const QuadraticPolynomialStructuredRoots& roots =
      structured_polynomial.structured_roots;

  if (roots.HasSingleRealRootWithMultiplicityTwo()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        definite_integral, one_over_sqrt_of_quadratic_polynomial::
                               IntegrateRealRootWithMultiplicityTwo(
                                   structured_polynomial, bounds_for_ranges));
  } else if (roots.HasOnlyRealRootsWithMultiplicityOne()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        definite_integral, one_over_sqrt_of_quadratic_polynomial::
                               IntegrateOnlyRealRootsWithMultiplicityOne(
                                   structured_polynomial, bounds_for_ranges));
  } else if (roots.HasComplexRoots()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        definite_integral,
        one_over_sqrt_of_quadratic_polynomial::IntegrateComplexConjugateRoots(
            structured_polynomial, bounds_for_ranges));
  }

  INTRINSIC_RT_RETURN_IF_ERROR(
      SanityCheckForDefiniteIntegral(definite_integral));
  return definite_integral;
}

icon::RealtimeStatusOr<double>
IntegrateOneOverSqrtOfStructuredQuadraticPolynomial(
    const QuadraticPolynomial& polynomial, const bool truncate) {
  if (AlmostEquals(polynomial.coeffs[0], 0.0)) {
    // Integrate linear polynomial.
    return one_over_sqrt_of_linear_polynomial::
        IntegrateOneOverSqrtOfLinearPolynomial(
            polynomial.range_start, polynomial.range_end, polynomial.coeffs[1],
            polynomial.coeffs[2]);
  }

  // Integrate quadratic polynomial.
  StructuredQuadraticPolynomial structured_polynomial{
      .scaling_coefficient = polynomial.coeffs[0],
      .range_start = polynomial.range_start,
      .range_end = polynomial.range_end,
  };
  INTRINSIC_RT_ASSIGN_OR_RETURN(structured_polynomial.structured_roots,
                                polynomial.ComputeStructuredRoots());

  return IntegrateOneOverSqrtOfStructuredQuadraticPolynomial(
      structured_polynomial, truncate);
}

icon::RealtimeStatusOr<double> IntegrateOneOverSqrtOfCubicPolynomial(
    const CubicPolynomial& polynomial) {
  if (polynomial.range_end <= polynomial.range_start) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Invalid range, `range_start` should be <= `range_end`. Got ",
        polynomial.range_start, " vs. ", polynomial.range_end, "."));
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const CubicPolynomialRoots roots,
                                polynomial.ComputeRoots());
  INTRINSIC_RT_ASSIGN_OR_RETURN(const std::complex<double> root0, roots.Get(0));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const std::complex<double> root1, roots.Get(1));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const std::complex<double> root2, roots.Get(2));

  // Compute elliptic modulus and Jacobi amplitudes.
  const std::complex<double> elliptic_modulus =
      (root1 - root2) / (root0 - root2);
  const std::complex<double> jacobi_amplitude_start =
      std::asin(std::sqrt((root2 - polynomial.range_start) / (root2 - root1)));
  const std::complex<double> jacobi_amplitude_end =
      std::asin(std::sqrt((root2 - polynomial.range_end) / (root2 - root1)));
  // If any of the real components of the elliptic modulus or Jacobi amplitudes
  // becomes not a number (e.g. due to having duplicated roots), then the
  // elliptic integral of the first kind is undefined and an error is returned.
  // Note e.g. that infinity is a valid value for the `elliptic_modulus`.
  if (std::isnan(jacobi_amplitude_start.real()) ||
      std::isnan(jacobi_amplitude_end.real()) ||
      std::isnan(elliptic_modulus.real())) {
    return icon::InternalError(
        "Undefined integral: Arguments failed finiteness check.");
  }

  // Compute elliptic integrals at range start and range end.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const std::complex<double> elliptic_integral_start,
      EllipticIntegralOfTheFirstKind(jacobi_amplitude_start, elliptic_modulus));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const std::complex<double> elliptic_integral_end,
      EllipticIntegralOfTheFirstKind(jacobi_amplitude_end, elliptic_modulus));

  // Compute factors of the elliptic integrals.
  const std::complex<double> factor_start = ComputeEllipticIntegralFactor(
      polynomial.range_start, root0, root1, root2, polynomial.coeffs);
  const std::complex<double> factor_end = ComputeEllipticIntegralFactor(
      polynomial.range_end, root0, root1, root2, polynomial.coeffs);

  // Compose the final complex definite integral.
  const std::complex<double> complex_definite_integral =
      factor_end * elliptic_integral_end -
      factor_start * elliptic_integral_start;
  const double definite_integral = std::abs(complex_definite_integral.real());

  // Sanity check that the `definite_integral` contains a valid number. This
  // case would be reached e.g. when the squared path velocities have become
  // negative within the interval integral. Then the integral becomes undefined.
  if (std::isnan(definite_integral) || std::isinf(definite_integral)) {
    return icon::InternalError(
        "Integration failed: squared path velocity might be negative in "
        "integration range.");
  }
  return definite_integral;
}

icon::RealtimeStatusOr<double> IntegrateOneOverSqrtOfStructuredCubicPolynomial(
    const StructuredCubicPolynomial& structured_polynomial,
    const bool truncate) {
  INTRINSIC_RT_RETURN_IF_ERROR(structured_polynomial.IsValid());
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const PolynomialIntegrationBounds bounds_for_ranges,
      ComputeBoundsForPolynomialIntegrationRange(structured_polynomial,
                                                 truncate));
  INTRINSIC_RT_RETURN_IF_ERROR(CheckPolynomialAtMiddleRangePointIsPositive(
      structured_polynomial, bounds_for_ranges));

  // Handle the different structured polynomials' integration cases.
  double definite_integral = std::numeric_limits<double>::quiet_NaN();
  const CubicPolynomialStructuredRoots& roots =
      structured_polynomial.structured_roots;
  if (roots.HasSingleRealRootWithMultiplicityThree()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        definite_integral, one_over_sqrt_of_cubic_polynomial::
                               IntegrateSingleRealRootWithMultiplicityThree(
                                   structured_polynomial, bounds_for_ranges));
  } else if (roots.HasRealRootWithMultiplicityTwo()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        definite_integral,
        one_over_sqrt_of_cubic_polynomial::IntegrateRealRootWithMultiplicityTwo(
            structured_polynomial, bounds_for_ranges));
  } else if (roots.HasOnlyRealRootsWithMultiplicityOne()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        definite_integral, one_over_sqrt_of_cubic_polynomial::
                               IntegrateOnlyRealRootsWithMultiplicityOne(
                                   structured_polynomial, bounds_for_ranges));
  } else if (roots.HasComplexRoots()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        definite_integral,
        one_over_sqrt_of_cubic_polynomial::
            IntegrateSingleRealRootAndTwoComplexConjugateRoots(
                structured_polynomial, bounds_for_ranges));
  }

  INTRINSIC_RT_RETURN_IF_ERROR(
      SanityCheckForDefiniteIntegral(definite_integral));
  return definite_integral;
}

icon::RealtimeStatusOr<double> IntegrateOneOverSqrtOfStructuredCubicPolynomial(
    const CubicPolynomial& polynomial, const bool truncate) {
  StructuredCubicPolynomial structured_polynomial{
      .scaling_coefficient = polynomial.coeffs[0],
      .range_start = polynomial.range_start,
      .range_end = polynomial.range_end,
  };
  INTRINSIC_RT_ASSIGN_OR_RETURN(structured_polynomial.structured_roots,
                                polynomial.ComputeStructuredRoots());

  return IntegrateOneOverSqrtOfStructuredCubicPolynomial(structured_polynomial,
                                                         truncate);
}

icon::RealtimeStatusOr<double>
UpperLimitOfIntegralOfOneOverSqrtOfStructuredCubicPolynomial(
    const StructuredCubicPolynomial& structured_polynomial,
    const double integral_value) {
  INTRINSIC_RT_RETURN_IF_ERROR(structured_polynomial.IsValid());
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const PolynomialIntegrationBounds bounds_for_ranges,
      ComputeBoundsForPolynomialIntegrationRange(structured_polynomial));
  INTRINSIC_RT_RETURN_IF_ERROR(CheckPolynomialAtMiddleRangePointIsPositive(
      structured_polynomial, bounds_for_ranges));

  // Handle the different structured polynomials' integration cases.
  double upper_limit = std::numeric_limits<double>::quiet_NaN();
  const CubicPolynomialStructuredRoots& roots =
      structured_polynomial.structured_roots;
  if (roots.HasSingleRealRootWithMultiplicityThree()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        upper_limit,
        one_over_sqrt_of_cubic_polynomial::
            UpperLimitForSingleRealRootWithMultiplicityThree(
                structured_polynomial, integral_value, bounds_for_ranges));
  } else if (roots.HasRealRootWithMultiplicityTwo()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        upper_limit,
        one_over_sqrt_of_cubic_polynomial::
            UpperLimitForRealRootWithMultiplicityTwo(
                structured_polynomial, integral_value, bounds_for_ranges));
  } else if (roots.HasOnlyRealRootsWithMultiplicityOne()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        upper_limit,
        one_over_sqrt_of_cubic_polynomial::
            UpperLimitForOnlyRealRootsWithMultiplicityOne(
                structured_polynomial, integral_value, bounds_for_ranges));
  } else if (roots.HasComplexRoots()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        upper_limit,
        one_over_sqrt_of_cubic_polynomial::
            UpperLimitForSingleRealRootAndTwoComplexConjugateRoots(
                structured_polynomial, integral_value, bounds_for_ranges));
  }

  INTRINSIC_RT_RETURN_IF_ERROR(SanityCheckForDefiniteIntegral(upper_limit));
  return upper_limit;
}

icon::RealtimeStatusOr<double>
UpperLimitOfIntegralOfOneOverSqrtOfStructuredCubicPolynomial(
    const CubicPolynomial& polynomial, const double integral_value) {
  StructuredCubicPolynomial structured_polynomial{
      .scaling_coefficient = polynomial.coeffs[0],
      .range_start = polynomial.range_start,
      .range_end = polynomial.range_end,
  };
  INTRINSIC_RT_ASSIGN_OR_RETURN(structured_polynomial.structured_roots,
                                polynomial.ComputeStructuredRoots());

  return UpperLimitOfIntegralOfOneOverSqrtOfStructuredCubicPolynomial(
      structured_polynomial, integral_value);
}

}  // namespace intrinsic::topp
