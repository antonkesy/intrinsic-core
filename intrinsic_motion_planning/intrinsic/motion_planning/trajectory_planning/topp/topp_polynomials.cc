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

#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomials.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <initializer_list>
#include <limits>
#include <optional>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomial_roots.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::topp {

namespace {

// Equality tolerance for floating point numbers.
constexpr double kEqualityTolerance = std::numeric_limits<double>::epsilon();

// Returns `kNumRootsCubicPolynomial` cubic roots for the `input` complex
// number according to Euler's formula:
//   1 = cos(2𝑘𝜋 + θ) + 𝑖sin(2𝑘𝜋 + θ) = 𝑒^{𝑖2𝑘𝜋 + θ}.
// where: 𝑖: denotes the imaginary constant (`sqrt(-1)`).
//        k: 0,1,2 is an index that allows to compute the three cubic roots.
//        θ: angle between imaginary and real component.
std::array<std::complex<double>, kNumRootsCubicPolynomial>
ComputeCubicRootsOfConstantComplexNumber(const std::complex<double>& input) {
  const double radius = std::sqrt(::intrinsic::IPow(input.real(), 2) +
                                  ::intrinsic::IPow(input.imag(), 2));
  const double angle = std::atan2(input.imag(), input.real());
  const double magnitude = std::cbrt(radius);

  std::array<std::complex<double>, kNumRootsCubicPolynomial> roots;
  for (int root_id = 0; root_id < kNumRootsCubicPolynomial; ++root_id) {
    roots[root_id] = std::complex<double>(
        magnitude * std::cos((angle + root_id * 2.0 * M_PI) / 3.0),
        magnitude * std::sin((angle + root_id * 2.0 * M_PI) / 3.0));
  }
  return roots;
}

// Struct that stores the validity of the computed roots (in the sense that they
// have finite coefficients and match the polynomial coefficients to the
// requested tolerance) and the accuracy of the computed roots.
struct ValidityAndAccuracy {
  bool is_valid = false;
  double accuracy = std::numeric_limits<double>::infinity();
};

// Struct that stores the computed roots of a cubic polynomial and the validity
// and accuracy of the computation.
struct RootsAndAccuracy {
  CubicPolynomialRoots roots;
  ValidityAndAccuracy validity_and_accuracy;
};

// Returns a structure with the highest accuracy roots. It only compares the
// accuracy of the `current_best_roots` and `candidate_roots`. If a set of roots
// is invalid, its accuracy is set to infinity. Thus, we only need to compare
// the accuracy.
icon::RealtimeStatusOr<RootsAndAccuracy> SelectRootsWithHighestAccuracy(
    RootsAndAccuracy current_best_roots,
    const CubicPolynomialRoots& candidate_roots,
    const ValidityAndAccuracy& candidate_validity_and_accuracy) {
  if (current_best_roots.validity_and_accuracy.accuracy <
      candidate_validity_and_accuracy.accuracy) {
    return current_best_roots;
  }
  return RootsAndAccuracy{
      .roots = candidate_roots,
      .validity_and_accuracy = candidate_validity_and_accuracy};
}

// Checks that the polynomial coefficients `coeffs` can be reconstructed from
// the computed `roots` and match up to the desired numerical `tolerance`.
icon::RealtimeStatusOr<ValidityAndAccuracy>
ValidateRootsGeneratePolynomialCoefficients(const eigenmath::Vector4d& coeffs,
                                            const CubicPolynomialRoots& roots,
                                            const double tolerance) {
  // Get root values.
  INTRINSIC_RT_ASSIGN_OR_RETURN(const std::complex<double> root0, roots.Get(0));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const std::complex<double> root1, roots.Get(1));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const std::complex<double> root2, roots.Get(2));

  if (std::isnan(root0.real()) || std::isinf(root0.real()) ||
      std::isnan(root0.imag()) || std::isinf(root0.imag()) ||
      std::isnan(root1.real()) || std::isinf(root1.real()) ||
      std::isnan(root1.imag()) || std::isinf(root1.imag()) ||
      std::isnan(root2.real()) || std::isinf(root2.real()) ||
      std::isnan(root2.imag()) || std::isinf(root2.imag())) {
    return ValidityAndAccuracy();
  }

  // Shortcuts for the roots' real and imaginary coefficients.
  const double real0 = root0.real();
  const double imag0 = root0.imag();
  const double real1 = root1.real();
  const double imag1 = root1.imag();
  const double real2 = root2.real();
  const double imag2 = root2.imag();

  // Compute polynomial coefficients from roots.
  const double quad_coeff_real = -real0 - real1 - real2;
  const double quad_coeff_imag = -imag0 - imag1 - imag2;
  const double lin_coeff_real = real0 * real1 + real0 * real2 - imag0 * imag1 -
                                imag0 * imag2 + real1 * real2 - imag1 * imag2;
  const double lin_coeff_imag = real0 * imag1 + real0 * imag2 + imag0 * real1 +
                                imag0 * real2 + real1 * imag2 + imag1 * real2;
  const double const_coeff_real = -real0 * real1 * real2 +
                                  real0 * imag1 * imag2 +
                                  imag0 * real1 * imag2 + imag0 * imag1 * real2;
  const double const_coeff_imag = imag0 * imag1 * imag2 -
                                  real0 * real1 * imag2 -
                                  real0 * imag1 * real2 - imag0 * real1 * real2;

  const double accuracy =
      std::max({std::abs(quad_coeff_real - coeffs[1] / coeffs[0]),
                std::abs(quad_coeff_imag),
                std::abs(lin_coeff_real - coeffs[2] / coeffs[0]),
                std::abs(lin_coeff_imag),
                std::abs(const_coeff_real - coeffs[3] / coeffs[0]),
                std::abs(const_coeff_imag)});
  return ValidityAndAccuracy{.is_valid = accuracy <= tolerance,
                             .accuracy = accuracy};
}

// Optimizes a single root of a cubic polynomial defined by the coefficients
// `coeffs` start from the guess given by `initial_root`.
icon::RealtimeStatusOr<std::complex<double>> ImproveAccuracyOfRoot(
    const eigenmath::Vector4d& coeffs,
    const std::complex<double>& initial_root) {
  std::complex<double> root = initial_root;
  std::optional<std::complex<double>> prev_root = std::nullopt;
  constexpr int kMaxNewtonSteps = 20;
  for (int i = 0; i < kMaxNewtonSteps; ++i) {
    const std::complex<double> f_xn = coeffs[0] * (root * root * root) +
                                      coeffs[1] * (root * root) +
                                      coeffs[2] * (root) + coeffs[3];
    const std::complex<double> fp_xn =
        3.0 * coeffs[0] * (root * root) + 2.0 * coeffs[1] * (root) + coeffs[2];
    // If `f_xn` is zero, then no more improvement is possible. If `fp_xn` is
    // zero, then the procedure has converged. Thus, we break the loop. We use a
    // very strict tolerance for this check to be numerically stable.
    constexpr double kStrictZeroTolerance = 1.0e-16;
    if (AlmostEquals(std::norm(f_xn), 0.0, kStrictZeroTolerance) ||
        AlmostEquals(std::norm(fp_xn), 0.0, kStrictZeroTolerance)) {
      break;
    }
    const std::complex<double> update = f_xn / fp_xn;
    const std::complex<double> new_root = root - update;

    const double kTolerance = 2.0 * std::numeric_limits<double>::epsilon();
    const bool current_root_check =
        (std::abs(new_root.real() - root.real()) < kTolerance &&
         std::abs(new_root.imag() - root.imag()) < kTolerance);
    const bool previous_root_check =
        (!prev_root.has_value()
             ? false
             : (std::abs(new_root.real() - prev_root->real()) < kTolerance &&
                std::abs(new_root.imag() - prev_root->imag()) < kTolerance));
    if (current_root_check || previous_root_check) {
      root = new_root;
      break;
    }
    prev_root = root;
    root = new_root;
  }
  return root;
}

// Runs the optimization of roots for a vector of `roots` of a cubic polynomial
// defined by the coefficients `coeffs`.
icon::RealtimeStatusOr<CubicPolynomialRoots> ImproveAccuracyOfRoots(
    const eigenmath::Vector4d& coeffs, const CubicPolynomialRoots& roots) {
  CubicPolynomialRoots accurate_roots = roots;
  for (int root_id = 0; root_id < accurate_roots.size(); ++root_id) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(std::complex<double> root,
                                  roots.Get(root_id));
    INTRINSIC_RT_ASSIGN_OR_RETURN(root,
                                  ImproveAccuracyOfRoot(coeffs,
                                                        /*initial_root=*/root));
    INTRINSIC_RT_RETURN_IF_ERROR(accurate_roots.Update(root_id, root));
  }
  return accurate_roots;
}

// Runs an optimization routine to find the most accurate roots for the cubic
// polynomial defined by the coefficients `coeffs` starting from the provided
// `guess_roots`. Returns the roots that most accurately reproduce the input
// polynomial coefficients and are thus the most accurate roots. For more
// details on this refer to go/intrinsic-squared-path-velocity-integrals.
icon::RealtimeStatusOr<CubicPolynomialStructuredRoots> OptimizeStructuredRoots(
    const eigenmath::Vector4d& coeffs,
    const CubicPolynomialStructuredRoots& guess_roots) {
  // Optimize the `guess_roots` to get the most accurate roots.
  CubicPolynomialStructuredRoots candidate_roots = guess_roots;
  for (int root_id = 0; root_id < candidate_roots.size(); ++root_id) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(StructuredRoot current_root,
                                  candidate_roots.Get(root_id));

    // Check if the current root is a complex conjugate of the previous root, in
    // which case we set the new root accordingly.
    if (root_id > 0) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot& previous_root,
                                    candidate_roots.Get(root_id - 1));
      if (previous_root.type == RootType::kComplex &&
          current_root.type == RootType::kComplex) {
        current_root.value = std::conj(previous_root.value);
        INTRINSIC_RT_RETURN_IF_ERROR(
            candidate_roots.Update(root_id, current_root));
        continue;
      }
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        current_root.value, ImproveAccuracyOfRoot(coeffs, current_root.value));
    INTRINSIC_RT_RETURN_IF_ERROR(candidate_roots.Update(root_id, current_root));
  }
  return candidate_roots;
}

// Helper method to compose sorted structured roots from a list of roots.
icon::RealtimeStatusOr<CubicPolynomialRoots> ComposeRoots(
    std::initializer_list<std::complex<double>> roots_list) {
  CubicPolynomialRoots roots;
  for (const auto& root : roots_list)
    INTRINSIC_RT_RETURN_IF_ERROR(roots.Add(root));
  INTRINSIC_RT_RETURN_IF_ERROR(roots.Sort());
  return roots;
}

// Helper method to compose sorted structured roots from a list of roots.
icon::RealtimeStatusOr<CubicPolynomialStructuredRoots> ComposeStructuredRoots(
    std::initializer_list<StructuredRoot> roots_list) {
  CubicPolynomialStructuredRoots roots;
  for (const auto& root : roots_list)
    INTRINSIC_RT_RETURN_IF_ERROR(roots.Add(root));
  INTRINSIC_RT_RETURN_IF_ERROR(roots.Sort());
  return roots;
}

// Estimates the structured roots of the cubic polynomial defined by the
// coefficients `coeffs` as in `coeffs[0]*x^3 + coeffs[1]*x^2 + coeffs[2]*x +
// coeffs[3] = 0`. The name of the function comes from the fact that this
// function constructs an initial guess of the roots, which should then be
// numerically optimized using Newton's method to achieve the precision
// required. For more details about the implementation, please refer to
// go/intrinsic-squared-path-velocity-integrals.
icon::RealtimeStatusOr<CubicPolynomialStructuredRoots> EstimateStructuredRoots(
    const eigenmath::Vector4d& coeffs) {
  if (AlmostEquals(coeffs[0], 0.0)) {
    return icon::InvalidArgumentError(
        "The leading coefficient cannot be zero.");
  }

  // Coefficients of the cubic polynomial a*x^3 + b*x^2 + c*x + d = 0.
  const double a = coeffs[0];  // Coefficient for cubic term.
  const double b = coeffs[1];  // Coefficient for quadratic term.
  const double c = coeffs[2];  // Coefficient for linear term.
  const double d = coeffs[3];  // Coefficient for constant term.
  const bool b_is_zero = AlmostEquals(b, 0.0);
  const bool c_is_zero = AlmostEquals(c, 0.0);
  const bool d_is_zero = AlmostEquals(d, 0.0);

  // Edge case where b, c, d are zero. Thus, the polynomial is simply a*x^3.
  if (b_is_zero && c_is_zero && d_is_zero) {
    return ComposeStructuredRoots(
        {{.type = RootType::kReal, .value = 0.0, .multiplicity = 3}});
  }

  // Edge case where c, d are zero. Thus, the polynomial is x^2 * (a*x + b).
  if (c_is_zero && d_is_zero) {
    return ComposeStructuredRoots(
        {{.type = RootType::kReal, .value = 0.0, .multiplicity = 2},
         {.type = RootType::kReal, .value = -b / a}});
  }

  // Edge case where d is zero. Thus, the polynomial is x * (a*x^2 + b*x + c).
  if (d_is_zero) {
    // Construct the roots based on the discriminant of the quadratic term.
    const double discriminant = ::intrinsic::IPow(b, 2) - 4.0 * a * c;
    const double b_over_2a = b / (2.0 * a);
    const double sqrt_over_2a = std::sqrt(std::abs(discriminant)) / (2.0 * a);
    if (AlmostEquals(discriminant, 0.0)) {
      return ComposeStructuredRoots(
          {{.type = RootType::kReal, .value = -b_over_2a, .multiplicity = 2},
           {.type = RootType::kReal, .value = 0.0}});
    } else if (discriminant > 0.0) {
      return ComposeStructuredRoots(
          {{.type = RootType::kReal, .value = -b_over_2a - sqrt_over_2a},
           {.type = RootType::kReal, .value = -b_over_2a + sqrt_over_2a},
           {.type = RootType::kReal, .value = 0.0}});
    } else {
      return ComposeStructuredRoots(
          {{.type = RootType::kComplex, .value = {-b_over_2a, -sqrt_over_2a}},
           {.type = RootType::kComplex, .value = {-b_over_2a, sqrt_over_2a}},
           {.type = RootType::kReal, .value = 0.0}});
    }
  }

  // Normalize the cubic equation to the form: x^3 + p*x^2 + q*x + r = 0.
  const double p = b / a;
  const double q = c / a;
  const double r = d / a;

  // Depress to: y^3 + Ay + B = 0, using the substitution: x = y - p/3.
  const double A = q - ::intrinsic::IPow(p, 2) / 3.0;
  const double B = r - (p * q) / 3.0 + (2.0 * ::intrinsic::IPow(p / 3.0, 3));

  // Calculate the discriminant `delta` of the depressed cubic equation.
  const double delta =
      ::intrinsic::IPow(B, 2) / 4.0 + ::intrinsic::IPow(A, 3) / 27.0;
  const bool delta_is_zero =
      delta >= 0.0 && AlmostEquals(delta, 0.0, kEqualityTolerance);

  CubicPolynomialStructuredRoots roots;
  if (delta_is_zero || delta > 0.0) {
    // It handles the cases:
    //   Case1: Three real roots with 2 or 3 repeated roots (delta = 0).
    //   Case2: One real root and two complex conjugate roots.

    // If `delta_is_zero`, and
    // - the discriminant of the derivative of the normalized cubic polynomial
    //   -> discriminant of (3*x^2 + 2*p*x + q) = 4*p^2 - 12*q is zero, or
    // - either the A or B coefficients are zero
    // then the cubic polynomial has a three-fold root.
    const double discriminant_of_derivative = 4.0 * (p * p) - 12.0 * q;
    if (delta_is_zero &&
        (AlmostEquals(discriminant_of_derivative, 0.0, kEqualityTolerance) ||
         AlmostEquals(A, 0.0, kEqualityTolerance) ||
         AlmostEquals(B, 0.0, kEqualityTolerance))) {
      // The value of the root comes from matching the second coefficient of:
      //   x^3 + p*x^2 + q*x + r = (x - root)^3, which is equivalent to:
      //   (x - root)^3 = x^3 - 3*root*x^2 + 3*root^2*x - root^3
      //   p = -3*root  -> root = -p/3
      return ComposeStructuredRoots(
          {{.type = RootType::kReal, .value = -p / 3.0, .multiplicity = 3}});
    }

    // Considering the depressed cubic polynomial, we use Cardano's formula
    // based on the Vieta's substitution to compute the correct set of roots.
    const double omega = -B / 2.0 + (delta_is_zero ? 0.0 : -std::sqrt(delta));
    const std::array<std::complex<double>, kNumRootsCubicPolynomial>
        omega_roots = ComputeCubicRootsOfConstantComplexNumber(omega);
    for (int i = 0; i < kNumRootsCubicPolynomial; ++i) {
      INTRINSIC_RT_RETURN_IF_ERROR(roots.Add(
          {.type = RootType::kReal,
           .value = (omega_roots[i] - A / (3.0 * omega_roots[i])) - p / 3.0}));
    }

    if (delta_is_zero) {
      // When the discriminant is zero, there are two or three repeated roots.
      // The check for three repeated roots is done above. Thus, here we only
      // need to handle the case of two repeated roots.
      INTRINSIC_RT_RETURN_IF_ERROR(
          roots.MarkRootsWithAlmostZeroButPositiveDiscrimimant());
    } else {
      INTRINSIC_RT_RETURN_IF_ERROR(roots.MarkComplexConjugateRootTypes());
    }
  } else {
    // Case: Three distinct real roots (casus irreducibilis). Note that because
    // delta is negative, it implies that `A` is nonzero. Thus, we can safely
    // perform the following divisions by `A` and `coeff`.
    const double coeff = 2.0 * std::sqrt(-A / 3.0);
    const double t = std::acos(3.0 * (B / A) / coeff);

    for (int i = 0; i < kNumRootsCubicPolynomial; ++i) {
      // The value of - p / 3.0 comes from mapping back to the original
      // coordinates from the substitution x = y - p/3.
      INTRINSIC_RT_RETURN_IF_ERROR(roots.Add(
          {.type = RootType::kReal,
           .value = coeff * std::cos((t + 2.0 * M_PI * i) / 3.0) - p / 3.0}));
    }
  }
  INTRINSIC_RT_RETURN_IF_ERROR(roots.MergeRealRootDuplicates());
  INTRINSIC_RT_RETURN_IF_ERROR(roots.Sort());
  return roots;
}

}  // namespace

double QuadraticPolynomial::EvaluatePolynomial(double x) const {
  return coeffs[0] * ::intrinsic::IPow(x, 2) + coeffs[1] * x + coeffs[2];
}

double QuadraticPolynomial::EvaluatePolynomialFirstDerivative(double x) const {
  return 2.0 * coeffs[0] * x + coeffs[1];
}

double QuadraticPolynomial::EvaluatePolynomialSecondDerivative(double x) const {
  return 2.0 * coeffs[0];
}

icon::RealtimeStatusOr<QuadraticPolynomialRoots>
QuadraticPolynomial::ComputeRoots() const {
  if (AlmostEquals(coeffs[0], 0.0)) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The leading coefficient cannot be zero. Got ", coeffs[0], "."));
  }

  const double a = coeffs[0];
  const double b = coeffs[1];
  const double c = coeffs[2];

  const std::complex<double> sqrt_discriminant =
      std::sqrt(std::complex<double>(::intrinsic::IPow(b, 2) - 4.0 * a * c));

  QuadraticPolynomialRoots roots;
  INTRINSIC_RT_RETURN_IF_ERROR(roots.Add((-b - sqrt_discriminant) / (2.0 * a)));
  INTRINSIC_RT_RETURN_IF_ERROR(roots.Add((-b + sqrt_discriminant) / (2.0 * a)));
  INTRINSIC_RT_RETURN_IF_ERROR(roots.Sort());
  return roots;
}

icon::RealtimeStatusOr<QuadraticPolynomialStructuredRoots>
QuadraticPolynomial::ComputeStructuredRoots() const {
  if (AlmostEquals(coeffs[0], 0.0)) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The leading coefficient cannot be zero. Got ", coeffs[0], "."));
  }

  const double a = coeffs[0];
  const double b = coeffs[1];
  const double c = coeffs[2];

  const double discriminant = ::intrinsic::IPow(b, 2) - 4.0 * a * c;
  const double b_over_2a = b / (2.0 * a);
  const double sqrt_over_2a = std::sqrt(std::abs(discriminant)) / (2.0 * a);

  QuadraticPolynomialStructuredRoots roots;
  if (AlmostEquals(discriminant, 0.0)) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        roots, QuadraticPolynomialStructuredRoots::ComposeSorted(
                   {{.type = RootType::kReal,
                     .value = -b_over_2a,
                     .multiplicity = 2}}));
  } else if (discriminant > 0.0) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        roots,
        QuadraticPolynomialStructuredRoots::ComposeSorted(
            {{.type = RootType::kReal, .value = -b_over_2a - sqrt_over_2a},
             {.type = RootType::kReal, .value = -b_over_2a + sqrt_over_2a}}));
  } else {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        roots,
        QuadraticPolynomialStructuredRoots::ComposeSorted(
            {{.type = RootType::kComplex, .value = {-b_over_2a, -sqrt_over_2a}},
             {.type = RootType::kComplex,
              .value = {-b_over_2a, sqrt_over_2a}}}));
  }
  return roots;
}

icon::RealtimeStatus StructuredQuadraticPolynomial::IsValid() const {
  // Non-zero scaling coefficient.
  if (AlmostEquals(scaling_coefficient, 0.0))
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Scaling coefficient cannot be zero. Got ", scaling_coefficient, "."));

  // Valid integration range.
  if (range_end <= range_start)
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Invalid range, `range_start` should be <= `range_end`. Got ",
        range_start, " vs. ", range_end, "."));

  // Valid roots.
  return structured_roots.IsValid();
}

icon::RealtimeStatusOr<double>
StructuredQuadraticPolynomial::EvaluatePolynomial(double x) const {
  double polynomial_value = scaling_coefficient;

  if (structured_roots.HasComplexRoots()) {
    if (structured_roots.size() != kNumRootsQuadraticPolynomial) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "The polynomial has complex roots, but it does not have exactly ",
          kNumRootsQuadraticPolynomial, " roots."));
    }

    // Evaluate polynomial in the case it has two complex roots.
    // (x - (a+bi))(x - (a-bi)) simplifies to x^2 - 2ax + a^2 + b^2
    INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                  structured_roots.Get(0));
    const std::complex<double>& complex_root = root0.value;

    polynomial_value *=
        (::intrinsic::IPow(x, 2) - 2.0 * complex_root.real() * x +
         ::intrinsic::IPow(complex_root.imag(), 2) +
         ::intrinsic::IPow(complex_root.real(), 2));
  } else {
    // Evaluate polynomial in the case it has only real roots.
    int num_real_roots = 0;
    for (int root_id = 0; root_id < structured_roots.size(); ++root_id) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot& root,
                                    structured_roots.Get(root_id));
      num_real_roots += root.multiplicity;
      polynomial_value *=
          ::intrinsic::IPow(x - root.value.real(), root.multiplicity);
    }
    if (num_real_roots != kNumRootsQuadraticPolynomial) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "The polynomial has only real roots, but it does not have exactly ",
          kNumRootsQuadraticPolynomial, " roots."));
    }
  }
  return polynomial_value;
}

icon::RealtimeStatusOr<double>
StructuredQuadraticPolynomial::EvaluatePolynomialFirstDerivative(
    double x) const {
  double derivative_value = scaling_coefficient;

  if (structured_roots.HasComplexRoots()) {
    if (structured_roots.size() != kNumRootsQuadraticPolynomial) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "The polynomial has complex roots, but it does not have exactly ",
          kNumRootsQuadraticPolynomial, " roots."));
    }
    // P(x) = s * (x^2 - 2ax + a^2 + b^2) => P'(x) = s * (2x - 2a)
    INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                  structured_roots.Get(0));
    derivative_value *= 2.0 * (x - root0.value.real());
  } else {
    if (structured_roots.size() == 1) {
      // Multiplicity 2: P(x) = s * (x - r0)^2 => P'(x) = s * 2(x - r0)
      INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                    structured_roots.Get(0));
      derivative_value *= 2.0 * (x - root0.value.real());
    } else if (structured_roots.size() == 2) {
      // Multiplicity 1, 1: P(x) = s * (x - r0)(x - r1) => P'(x) = s * (2x - r0
      // - r1)
      INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root0,
                                    structured_roots.Get(0));
      INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot root1,
                                    structured_roots.Get(1));
      derivative_value *= (2.0 * x - root0.value.real() - root1.value.real());
    } else {
      return icon::InvalidArgumentError(
          "Invalid number of structured roots for a quadratic polynomial.");
    }
  }
  return derivative_value;
}

double CubicPolynomial::EvaluatePolynomial(double x) const {
  return coeffs[0] * ::intrinsic::IPow(x, 3) +
         coeffs[1] * ::intrinsic::IPow(x, 2) + coeffs[2] * x + coeffs[3];
}

double CubicPolynomial::EvaluatePolynomialFirstDerivative(double x) const {
  return 3.0 * coeffs[0] * ::intrinsic::IPow(x, 2) + 2.0 * coeffs[1] * x +
         coeffs[2];
}

double CubicPolynomial::EvaluatePolynomialSecondDerivative(double x) const {
  return 6.0 * coeffs[0] * x + 2.0 * coeffs[1];
}

// Function to compute the three roots for a cubic polynomial with real
// coefficients in sorted order, from the most negative in the real and
// imaginary components to the most positive in the real and imaginary
// components.
icon::RealtimeStatusOr<CubicPolynomialRoots> CubicPolynomial::ComputeRoots()
    const {
  if (AlmostEquals(coeffs[0], 0.0)) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The leading coefficient cannot be zero. Got ", coeffs[0], "."));
  }

  const double a = coeffs[0];  // Coefficient for cubic term.
  const double b = coeffs[1];  // Coefficient for quadratic term.
  const double c = coeffs[2];  // Coefficient for linear term.
  const double d = coeffs[3];  // Coefficient for constant term.

  // Handle the edge case where the c, d coefficients of the cubic polynomial
  // are zero. From the general equation a*x^3 + b*x^2 + c*x + d = 0, we can
  // refactor it as a*x^3 + b*x^2 = x^2 * (a*x + b) = 0, and we can easily and
  // exactly compute the roots.
  if (AlmostEquals(c, 0.0) && AlmostEquals(d, 0.0)) {
    return ComposeRoots({0.0, 0.0, -b / a});
  }

  // Handle the edge case where the d coefficient of the cubic polynomial is
  // zero. From the general equation a*x^3 + b*x^2 + c*x + d = 0, we can
  // refactor it as a*x^3 + b*x^2 + c*x = x * (a*x^2 + b*x + c) = 0..
  if (AlmostEquals(d, 0.0)) {
    const std::complex<double> sqrt_discriminant =
        std::sqrt(std::complex<double>(::intrinsic::IPow(b, 2) - 4.0 * a * c));
    return ComposeRoots({(-b + sqrt_discriminant) / (2.0 * a),
                         (-b - sqrt_discriminant) / (2.0 * a), 0.0});
  }

  // Some shortcuts used to construct the roots.
  const double a2 = a * a;
  const double b2 = b * b;
  const double b3 = b2 * b;
  const double term1 = -27.0 * a2 * d + 9.0 * a * b * c - 2.0 * b3;
  const double term2 = 3.0 * a * c - b2;
  const double term3 =
      ::intrinsic::IPow(term1, 2) + 4.0 * ::intrinsic::IPow(term2, 3);
  const std::array<std::complex<double>, kNumRootsCubicPolynomial> cubic_roots =
      ComputeCubicRootsOfConstantComplexNumber(
          std::sqrt(std::complex<double>(term3)) + term1);
  const std::complex<double>& term4 = cubic_roots[0];

  const double term5 = 3.0 * std::cbrt(4.0) * a;
  const double term6 = 6.0 * std::cbrt(2.0) * a;
  const double term7 = 3.0 * a;

  // Terms that are only valid when `a` is nonzero.
  const double b_over_3a = b / term7;
  const double term8 = std::cbrt(2.0) / term7;
  const double term9 = 1.0 / (std::cbrt(2.0) * term7);

  std::complex<double> imag_unit(0.0, 1.0);
  std::complex<double> w_plus = 1.0 + std::sqrt(3.0) * imag_unit;
  std::complex<double> w_minus = 1.0 - std::sqrt(3.0) * imag_unit;

  std::complex<double> root1 =
      term9 * term4 - term8 * term2 / term4 - b_over_3a;
  std::complex<double> root2 = -(w_minus * term4) / term6 +
                               (w_plus * term2) / (term5 * term4) - b_over_3a;
  std::complex<double> root3 = -(w_plus * term4) / term6 +
                               (w_minus * term2) / (term5 * term4) - b_over_3a;
  INTRINSIC_RT_ASSIGN_OR_RETURN(CubicPolynomialRoots roots,
                                ComposeRoots({root1, root2, root3}));

  const double kNumericalTolerance = 1e-8;
  INTRINSIC_RT_ASSIGN_OR_RETURN(ValidityAndAccuracy validity_and_accuracy,
                                ValidateRootsGeneratePolynomialCoefficients(
                                    coeffs, roots, kNumericalTolerance));
  // Construct current best guess.
  RootsAndAccuracy highest_accuracy_roots;
  INTRINSIC_RT_ASSIGN_OR_RETURN(highest_accuracy_roots.roots,
                                ComposeRoots({root1, root2, root3}));
  highest_accuracy_roots.validity_and_accuracy = validity_and_accuracy;

  if (!validity_and_accuracy.is_valid &&
      !std::isnan(validity_and_accuracy.accuracy) &&
      !std::isinf(validity_and_accuracy.accuracy)) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(roots, ImproveAccuracyOfRoots(coeffs, roots));
    INTRINSIC_RT_ASSIGN_OR_RETURN(validity_and_accuracy,
                                  ValidateRootsGeneratePolynomialCoefficients(
                                      coeffs, roots, kNumericalTolerance));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        highest_accuracy_roots,
        SelectRootsWithHighestAccuracy(highest_accuracy_roots, roots,
                                       validity_and_accuracy));
  }

  // If the computation above failed, retry with a different approach.
  if (!validity_and_accuracy.is_valid) {
    // Some shortcuts used to construct the roots.
    const double a3 = a2 * a;

    const double term1 = -b3 / (27 * a3) + (b * c) / (6.0 * a2) - d / (2.0 * a);
    const double term2 = -b2 / (9.0 * a2) + c / (3.0 * a);
    const std::complex<double> term3 =
        std::sqrt(::intrinsic::IPow(term1, 2) + ::intrinsic::IPow(term2, 3));

    const std::complex<double> w_plus(-0.5, std::sqrt(3.0) / 2.0);
    const std::complex<double> w_minus(-0.5, -std::sqrt(3.0) / 2.0);

    const std::array<std::complex<double>, kNumRootsCubicPolynomial>
        cubic_roots1 = ComputeCubicRootsOfConstantComplexNumber(term1 + term3);
    const std::array<std::complex<double>, kNumRootsCubicPolynomial>
        cubic_roots2 = ComputeCubicRootsOfConstantComplexNumber(term1 - term3);
    const std::complex<double>& term4 = cubic_roots1[0];
    const std::complex<double>& term5 = cubic_roots2[0];

    const std::complex<double> root1 = term4 + term5 - b_over_3a;
    const std::complex<double> root2 =
        w_plus * term4 + w_minus * term5 - b_over_3a;
    const std::complex<double> root3 =
        w_minus * term4 + w_plus * term5 - b_over_3a;
    INTRINSIC_RT_ASSIGN_OR_RETURN(roots, ComposeRoots({root1, root2, root3}));

    INTRINSIC_RT_ASSIGN_OR_RETURN(validity_and_accuracy,
                                  ValidateRootsGeneratePolynomialCoefficients(
                                      coeffs, roots, kNumericalTolerance));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        highest_accuracy_roots,
        SelectRootsWithHighestAccuracy(highest_accuracy_roots, roots,
                                       validity_and_accuracy));
    if (!validity_and_accuracy.is_valid &&
        !std::isnan(validity_and_accuracy.accuracy) &&
        !std::isinf(validity_and_accuracy.accuracy)) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(roots,
                                    ImproveAccuracyOfRoots(coeffs, roots));
      INTRINSIC_RT_ASSIGN_OR_RETURN(validity_and_accuracy,
                                    ValidateRootsGeneratePolynomialCoefficients(
                                        coeffs, roots, kNumericalTolerance));
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          highest_accuracy_roots,
          SelectRootsWithHighestAccuracy(highest_accuracy_roots, roots,
                                         validity_and_accuracy));
    }
  }

  if (!validity_and_accuracy.is_valid) {
    // This case should never be reached. In case it does, we return the
    // roots with the highest possible accuracy if they satisfy a reduced
    // numerical tolerance. Otherwise, we would return an error.
    const double kReducedNumericalTolerance = 1e-6;
    if (highest_accuracy_roots.validity_and_accuracy.accuracy <
        kReducedNumericalTolerance) {
      INTRINSIC_RT_RETURN_IF_ERROR(highest_accuracy_roots.roots.Sort());
      return highest_accuracy_roots.roots;
    }
    return icon::InternalError(
        "Failed to compute the roots of the cubic polynomial.");
  }

  INTRINSIC_RT_RETURN_IF_ERROR(roots.Sort());
  return roots;
}

// Constructs the roots of a cubic polynomial with real coefficients in
// structured form. It first estimates an initial guess for the roots and then
// improves their accuracy using numerical methods.
icon::RealtimeStatusOr<CubicPolynomialStructuredRoots>
CubicPolynomial::ComputeStructuredRoots() const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const CubicPolynomialStructuredRoots guess_roots,
      EstimateStructuredRoots(coeffs));
  INTRINSIC_RT_ASSIGN_OR_RETURN(CubicPolynomialStructuredRoots optimized_roots,
                                OptimizeStructuredRoots(coeffs, guess_roots));
  INTRINSIC_RT_RETURN_IF_ERROR(optimized_roots.Sort());
  return optimized_roots;
  // return SortRoots(optimized_roots);
}

icon::RealtimeStatus StructuredCubicPolynomial::IsValid() const {
  // Non-zero scaling coefficient.
  if (AlmostEquals(scaling_coefficient, 0.0))
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Scaling coefficient cannot be zero. Got ", scaling_coefficient, "."));

  // Valid integration range.
  if (range_end <= range_start)
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Invalid range, `range_start` should be <= `range_end`. Got ",
        range_start, " vs. ", range_end, "."));

  // Valid roots.
  return structured_roots.IsValid();
}

icon::RealtimeStatusOr<double> StructuredCubicPolynomial::EvaluatePolynomial(
    double x) const {
  double polynomial_value = scaling_coefficient;
  if (structured_roots.HasComplexRoots()) {
    if (structured_roots.size() != kNumRootsCubicPolynomial) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "The polynomial has complex roots, but it does not have exactly ",
          kNumRootsCubicPolynomial, " roots."));
    }
    // Evaluate polynomial in the case it has two complex roots.
    INTRINSIC_RT_ASSIGN_OR_RETURN(StructuredRoot root0,
                                  structured_roots.Get(0));
    INTRINSIC_RT_ASSIGN_OR_RETURN(StructuredRoot root2,
                                  structured_roots.Get(2));
    const bool is_root0_real = root0.type == RootType::kReal;
    const double real_root =
        (is_root0_real ? root0.value.real() : root2.value.real());
    const std::complex<double>& complex_root =
        (is_root0_real ? root2.value : root0.value);

    polynomial_value *= (x - real_root);
    polynomial_value *=
        (::intrinsic::IPow(x, 2) - 2.0 * complex_root.real() * x +
         ::intrinsic::IPow(complex_root.imag(), 2) +
         ::intrinsic::IPow(complex_root.real(), 2));
  } else {
    // Evaluate polynomial in the case it has only real roots.
    int num_real_roots = 0;
    for (int root_id = 0; root_id < structured_roots.size(); ++root_id) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(const StructuredRoot& root,
                                    structured_roots.Get(root_id));
      num_real_roots += root.multiplicity;
      polynomial_value *=
          ::intrinsic::IPow(x - root.value.real(), root.multiplicity);
    }
    if (num_real_roots != kNumRootsCubicPolynomial) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "The polynomial has only real roots, but it does not have exactly ",
          kNumRootsCubicPolynomial, " roots."));
    }
  }
  return polynomial_value;
}

}  // namespace intrinsic::topp
