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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_ELLIPTIC_INTEGRAL_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_ELLIPTIC_INTEGRAL_H_

#include <complex>

#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::topp {

namespace elliptic_integral_internal {

// Compute the Carlson RF integral of the following form:
//          inf             0.5 * dt
//  integral    -----------------------------------
//      t = 0.0  sqrt((t + x) * (t + y) * (t + z))
// for the given input complex coefficients `x`, `y`, `z`. For details, refer
// to https://en.wikipedia.org/wiki/Carlson_symmetric_form.
icon::RealtimeStatusOr<std::complex<double>> ComputeCarlsonRFIntegral(
    const std::complex<double>& input_x, const std::complex<double>& input_y,
    const std::complex<double>& input_z);

}  // namespace elliptic_integral_internal

// Computes the elliptic integral of the first kind for the `jacobi_amplitude`
// `phi` and `elliptic_modulus` `m` that takes the following form:
//                   theta = phi            dtheta
//  F(phi, m=k^2) = Integral     --------------------------------
//                   theta = 0    sqrt( 1 - k^2 * sin(theta)^2 )
// Both the Jacobi amplitude and the elliptic modulus can be complex numbers.
// The integral is computed based on Carlson symmetric forms of elliptic
// integrals. Refer to
// https://mathworld.wolfram.com/EllipticIntegraloftheFirstKind.html and
// https://en.wikipedia.org/wiki/Carlson_symmetric_form for more details.
icon::RealtimeStatusOr<std::complex<double>> EllipticIntegralOfTheFirstKind(
    std::complex<double> jacobi_amplitude,
    std::complex<double> elliptic_modulus);

// Computes the Jacobi amplitude for the given `elliptic_integral` value and
// `elliptic_modulus`. It performs the inverse operation of the elliptic
// integral of the first kind using the Arithmetic-Geometric Mean (AGM) method.
// For details, refer to https://mathworld.wolfram.com/JacobiAmplitude.html and
// https://en.wikipedia.org/wiki/Arithmetic%E2%80%93geometric_mean for the
// algorithm used to compute it.
icon::RealtimeStatusOr<std::complex<double>> JacobiAmplitude(
    std::complex<double> elliptic_integral,
    std::complex<double> elliptic_modulus);

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_ELLIPTIC_INTEGRAL_H_
