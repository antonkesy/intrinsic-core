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

#include "intrinsic/motion_planning/trajectory_planning/topp/elliptic_integral.h"

#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <optional>

#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/almost_equals.h"

namespace intrinsic::topp {

namespace {

// Imaginary unit.
constexpr std::complex<double> kImaginaryUnit(0.0, 1.0);

// Computes the inverse of the hyperbolic tangent function for a complex
// number `z`.
std::complex<double> InverseHyperbolicTangent(std::complex<double> z) {
  return 0.5 * std::log((1.0 + z) / (1.0 - z));
}

// Numerical computation of the Jacobi amplitude (`phi` for short) as a way of
// improving its numerical accuracy. It takes a warm-start value for `phi`
// computed with the AGM method. The elliptic integral computes:
// elliptic_integral = u = F(phi|m), where `m` is the elliptic modulus. The
// Jacobi amplitude computes: jacobi_amplitude = phi = am(u|m).
icon::RealtimeStatusOr<std::complex<double>> JacobiAmplitudeNumerical(
    const std::complex<double>& elliptic_integral,
    const std::complex<double>& elliptic_modulus,
    std::optional<std::complex<double>> warm_start_phi = std::nullopt) {
  // Initial guess for the Jacobi amplitude.
  std::complex<double> jacobi_amplitude =
      warm_start_phi.value_or(elliptic_integral);

  // We use Newton method to solve the equation
  //   F(phi|m) = u for phi.
  // In other words, we find a zero of the following error function:
  //   error_function(phi, m, u) = F(phi|m) - u = 0.0.
  constexpr int kMaxNewtonSteps = 30;
  constexpr double kEpsilonMultiplier = 32.0;
  constexpr double kEpsilon =
      kEpsilonMultiplier * std::numeric_limits<double>::epsilon();
  // Above, we use a very strict tolerance for convergence. However, to be more
  // robust to numerical sensitivity, we keep track of the iterate that
  // minimizes the error function.
  double smallest_error = std::numeric_limits<double>::infinity();
  std::complex<double> best_jacobi_amplitude = jacobi_amplitude;
  for (int i = 0; i < kMaxNewtonSteps; ++i) {
    // Compute elliptic integral at the current guess for the Jacobi amplitude.
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        std::complex<double> elliptic_integral_guess,
        EllipticIntegralOfTheFirstKind(jacobi_amplitude, elliptic_modulus));

    const std::complex<double> error_function =
        elliptic_integral_guess - elliptic_integral;
    if (std::abs(error_function) < smallest_error) {
      smallest_error = std::abs(error_function);
      best_jacobi_amplitude = jacobi_amplitude;
    }
    const std::complex<double> derror_function =
        1.0 / std::sqrt(std::complex<double>{1.0, 0.0} -
                        elliptic_modulus * std::sin(jacobi_amplitude) *
                            std::sin(jacobi_amplitude));
    if (std::abs(error_function) < kEpsilon ||
        std::abs(derror_function) < kEpsilon)
      break;
    jacobi_amplitude -= error_function / derror_function;
  }
  return best_jacobi_amplitude;
}

}  // namespace

namespace elliptic_integral_internal {

// Compute the Carlson RF integral (detailed information about it can be found
// in https://en.wikipedia.org/wiki/Carlson_symmetric_form). It constructs an
// integral of the form
//          inf             0.5 * dt
//  integral    -----------------------------------
//      t = 0.0  sqrt((t + x) * (t + y) * (t + z))
// for the given input complex coefficients `x`, `y`, `z`.
icon::RealtimeStatusOr<std::complex<double>> ComputeCarlsonRFIntegral(
    const std::complex<double>& input_x, const std::complex<double>& input_y,
    const std::complex<double>& input_z) {
  int num_zeros = 0;
  if (::intrinsic::AlmostEquals(input_x.real(), 0.0) &&
      ::intrinsic::AlmostEquals(input_x.imag(), 0.0)) {
    num_zeros++;
  }
  if (::intrinsic::AlmostEquals(input_y.real(), 0.0) &&
      ::intrinsic::AlmostEquals(input_y.imag(), 0.0)) {
    num_zeros++;
  }
  if (::intrinsic::AlmostEquals(input_z.real(), 0.0) &&
      ::intrinsic::AlmostEquals(input_z.imag(), 0.0)) {
    num_zeros++;
  }

  if (num_zeros >= 2) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Carlson RF integral input can have one zero. Got ", num_zeros, "."));
  }

  std::complex<double> x = input_x;
  std::complex<double> y = input_y;
  std::complex<double> z = input_z;
  double dx = std::numeric_limits<double>::max();
  double dy = std::numeric_limits<double>::max();
  double dz = std::numeric_limits<double>::max();
  const double kTolerance = 2.0 * std::numeric_limits<double>::epsilon();

  std::complex<double> average;
  int num_iterations = 0;
  const int kMaxNumIterations = 1000;
  while (dx > kTolerance || dy > kTolerance || dz > kTolerance) {
    std::complex<double> lambda = std::sqrt(x) * std::sqrt(y) +
                                  std::sqrt(y) * std::sqrt(z) +
                                  std::sqrt(z) * std::sqrt(x);
    x = (x + lambda) / 4.0;
    y = (y + lambda) / 4.0;
    z = (z + lambda) / 4.0;
    average = (x + y + z) / 3.0;
    dx = std::norm(1.0 - x / average);
    dy = std::norm(1.0 - y / average);
    dz = std::norm(1.0 - z / average);

    // This is just a safety check to avoid infinite loops. But it should never
    // be reached since the loop converges quickly.
    num_iterations++;
    if (num_iterations >= kMaxNumIterations) {
      return icon::InternalError(
          "Carlson RF integral reached max number of iterations.");
    }
  }

  // Compose the integral based on a Taylor series expansion with high order
  // terms to achieve great accuracy.
  const double cubic_prod = dy * dx * dz;
  const double quad_prod = dx * dy + dy * dz + dz * dx;
  return (1.0 - quad_prod / 10.0 + cubic_prod / 14.0 +
          quad_prod * quad_prod / 24.0 - 3.0 * quad_prod * cubic_prod / 44.0 -
          5.0 * quad_prod * quad_prod * quad_prod / 208.0 +
          3.0 * cubic_prod * cubic_prod / 104.0 +
          quad_prod * quad_prod * cubic_prod / 16.0) /
         std::sqrt(average);
}

}  // namespace elliptic_integral_internal

icon::RealtimeStatusOr<std::complex<double>> EllipticIntegralOfTheFirstKind(
    std::complex<double> jacobi_amplitude,
    std::complex<double> elliptic_modulus) {
  // Commonly used constants.
  constexpr double kPi = M_PI;
  constexpr double kHalfPi = M_PI_2;

  if ((::intrinsic::AlmostEquals(jacobi_amplitude.real(), 0.0) &&
       ::intrinsic::AlmostEquals(jacobi_amplitude.imag(), 0.0)) ||
      (std::isinf(elliptic_modulus.real()) ||
       std::isinf(elliptic_modulus.imag()))) {
    return std::complex<double>(0, 0);
  }
  // Degenerate case when the Jacobi amplitude has infinite imaginary part:
  // When the Jacobi amplitude has imaginary part tending to infinity, the
  // corresponding value of the elliptic integral becomes:
  // F( Inf*i | m ) = i * K'(m) = i * K(m') = i * K(1-m)
  // where F denotes the elliptic integral of the first kind, and K is the
  // complete elliptic integral. Note that F(pi/2 | m) = K(m).
  // The effect of the non-zero real part of the Jacobi amplitude is only an
  // oscillation that vanishes as the imaginary part goes to infinity.
  if (std::isinf(jacobi_amplitude.imag()) &&
      (elliptic_modulus.real() >= 0.0 && elliptic_modulus.real() < 1.0)) {
    const double sign_inf = jacobi_amplitude.imag() < 0.0 ? -1.0 : 1.0;
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        std::complex<double> complete_elliptic_integral,
        EllipticIntegralOfTheFirstKind(sign_inf * kHalfPi,
                                       1.0 - elliptic_modulus));
    return kImaginaryUnit * complete_elliptic_integral;
  }
  if (::intrinsic::AlmostEquals(std::abs(jacobi_amplitude.real()), kHalfPi) &&
      jacobi_amplitude.imag() >= 0.0 &&
      ::intrinsic::AlmostEquals(elliptic_modulus.real(), 1.0) &&
      ::intrinsic::AlmostEquals(elliptic_modulus.imag(), 0.0)) {
    return icon::FailedPreconditionError(icon::RealtimeStatus::StrCat(
        "Elliptic integral undefined for angle/modulus (",
        jacobi_amplitude.real(), ",", elliptic_modulus.real(), ")."));
  }
  if (jacobi_amplitude.real() >= -kHalfPi &&
      jacobi_amplitude.real() <= kHalfPi) {
    if (::intrinsic::AlmostEquals(elliptic_modulus.real(), 1.0) &&
        std::abs(jacobi_amplitude.real()) < kHalfPi) {
      return InverseHyperbolicTangent(std::sin(jacobi_amplitude));
    }
    if (::intrinsic::AlmostEquals(elliptic_modulus.real(), 0.0) &&
        ::intrinsic::AlmostEquals(elliptic_modulus.imag(), 0.0)) {
      return jacobi_amplitude;
    }
    std::complex<double> sine = std::sin(jacobi_amplitude);
    if (std::isinf(sine.real()) || std::isinf(sine.imag())) {
      return icon::InternalError(icon::RealtimeStatus::StrCat(
          "Elliptic integral is not finite. Got sine(", jacobi_amplitude.real(),
          " + ", jacobi_amplitude.imag(), "I)."));
    }
    std::complex<double> sine2 = sine * sine;
    std::complex<double> cosine2 = 1.0 - sine2;
    std::complex<double> oneminusmsine2 = 1.0 - elliptic_modulus * sine2;
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        std::complex<double> carlson_rf_integral,
        elliptic_integral_internal::ComputeCarlsonRFIntegral(
            cosine2, oneminusmsine2, std::complex<double>(1.0, 0.0)));
    return sine * carlson_rf_integral;
  }
  double factor;
  if (jacobi_amplitude.real() > kHalfPi) {
    factor = std::ceil((jacobi_amplitude.real() - kHalfPi) / kPi);
    jacobi_amplitude = jacobi_amplitude - factor * kPi;
  } else {
    factor = -std::floor((kHalfPi - jacobi_amplitude.real()) / kPi);
    jacobi_amplitude = jacobi_amplitude - factor * kPi;
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      std::complex<double> part1,
      EllipticIntegralOfTheFirstKind(kHalfPi, elliptic_modulus));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      std::complex<double> part2,
      EllipticIntegralOfTheFirstKind(jacobi_amplitude, elliptic_modulus));
  return 2.0 * factor * part1 + part2;
}

icon::RealtimeStatusOr<std::complex<double>> JacobiAmplitude(
    std::complex<double> elliptic_integral,
    std::complex<double> elliptic_modulus) {
  // Maximum number of iterations and convergence tolerance for the
  // Arithmetic–geometric mean iterative method.
  const int kMaxAGMIterations = 127;
  constexpr double kConvergenceTolerance =
      2.0 * std::numeric_limits<double>::epsilon();

  // Forward pass of the Arithmetic–geometric mean iteration method.
  std::array<std::complex<double>, kMaxAGMIterations + 1> arithmetic_means;
  std::array<std::complex<double>, kMaxAGMIterations + 1> residuals;

  arithmetic_means[0] = 1.0;
  residuals[0] = std::sqrt(elliptic_modulus);
  std::complex<double> geometric_mean = std::sqrt(1.0 - elliptic_modulus);

  // Iterate until the residual is negligible (below `kConvergenceTolerance`).
  int iterations = 1;
  while (std::abs(residuals[iterations - 1]) > kConvergenceTolerance &&
         iterations <= kMaxAGMIterations) {
    arithmetic_means[iterations] =
        0.5 * (arithmetic_means[iterations - 1] + geometric_mean);
    residuals[iterations] =
        0.5 * (arithmetic_means[iterations - 1] - geometric_mean);
    geometric_mean =
        std::sqrt(arithmetic_means[iterations - 1] * geometric_mean);
    iterations++;
  }

  // Check if the algorithm has converged to enough accuracy. If not, attempt to
  // construct the Jacobi amplitude numerically without warm-start. This case
  // should never happen in practice.
  const int kLastIndex = iterations - 1;
  if (iterations >= kMaxAGMIterations + 1 &&
      std::abs(residuals[kLastIndex]) > kConvergenceTolerance) {
    return JacobiAmplitudeNumerical(elliptic_integral, elliptic_modulus);
  }

  // Backward pass of the Arithmetic–geometric mean iterative method that
  // computes the angle phi starting from phi_N = a_N * elliptic_integral.
  std::complex<double> phi = std::pow(2.0, kLastIndex) *
                             arithmetic_means[kLastIndex] * elliptic_integral;

  for (int id = kLastIndex; id > 0; --id) {
    const std::complex<double> ratio =
        (residuals[id] / arithmetic_means[id]) * std::sin(phi);

    // Maybe consider clamping the argument for the arcsin function to the range
    // [-1, 1] to prevent domain errors due to floating-point inaccuracy.
    phi = 0.5 * (phi + std::asin(ratio));
  }

  // Numerically improve the computed Jacobi amplitude form the warm-start.
  return JacobiAmplitudeNumerical(elliptic_integral, elliptic_modulus, phi);
}

}  // namespace intrinsic::topp
