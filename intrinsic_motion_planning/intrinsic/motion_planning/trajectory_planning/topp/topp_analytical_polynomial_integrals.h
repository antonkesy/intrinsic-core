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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_ANALYTICAL_POLYNOMIAL_INTEGRALS_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_ANALYTICAL_POLYNOMIAL_INTEGRALS_H_

#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomials.h"

namespace intrinsic::topp {

// Computes the following integral for a structured quadratic
// `structured_polynomial`:
//       range_end                1.0
//  integral         ----------------------------- dx
//   x = range_start  sqrt(structured polynomial)
// Note that the structured polynomial must be non-negative in its integration
// range for the integral to be real. If `truncate` is set to true, the
// algorithm attempts to find a valid (strictly non-negative) integration
// interval that maximally overlaps with the desired integration range
// `[structured_polynomial.range_start, structured_polynomial.range_end]` to
// compute the integral within that range.
icon::RealtimeStatusOr<double>
IntegrateOneOverSqrtOfStructuredQuadraticPolynomial(
    const StructuredQuadraticPolynomial& structured_polynomial,
    bool truncate = false);

// Computes the same integral as above, but first converts the quadratic
// `polynomial` to its structured form representation.
icon::RealtimeStatusOr<double>
IntegrateOneOverSqrtOfStructuredQuadraticPolynomial(
    const QuadraticPolynomial& polynomial, bool truncate = false);

// Computes the following integral for a cubic `polynomial`:
//       range_end          1.0
//  integral         ------------------ dx
//   x = range_start  sqrt(polynomial)
// Note that the polynomial must be non-negative in its integration range for
// the integral to be real.
icon::RealtimeStatusOr<double> IntegrateOneOverSqrtOfCubicPolynomial(
    const CubicPolynomial& polynomial);

// Computes the following integral for a cubic `structured_polynomial`:
//       range_end                1.0
//  integral         ----------------------------- dx
//   x = range_start  sqrt(structured polynomial)
// Note that the structured polynomial must be non-negative in its integration
// range for the integral to be real. If `truncate` is set to true, the
// algorithm attempts to find a valid (strictly non-negative) integration
// interval that maximally overlaps with the desired integration range
// `[structured_polynomial.range_start, structured_polynomial.range_end]` to
// compute the integral within that range.
icon::RealtimeStatusOr<double> IntegrateOneOverSqrtOfStructuredCubicPolynomial(
    const StructuredCubicPolynomial& structured_polynomial,
    bool truncate = false);

// Computes the same integral as above, but first converts the cubic
// `polynomial` to its structured form representation.
icon::RealtimeStatusOr<double> IntegrateOneOverSqrtOfStructuredCubicPolynomial(
    const CubicPolynomial& polynomial, bool truncate = false);

// Above we computed an integral for the cubic `structured_polynomial` as in:
//           range_end                1.0
//  I = integral         ----------------------------- dx
//       x = range_start  sqrt(structured polynomial)
// This function, takes a `structured_polynomial`, its `integral_value` `I` at
// some point within the valid range of the polynomial [`range_start`,
// `range_end`] and computes the appropriate upper limit `range_end` such that
// the integral evaluates to the given `integral_value` `I`. Note that the
// structured polynomial must be non-negative in its integration range for the
// integral to be real.
icon::RealtimeStatusOr<double>
UpperLimitOfIntegralOfOneOverSqrtOfStructuredCubicPolynomial(
    const StructuredCubicPolynomial& structured_polynomial,
    double integral_value);

// Computes the same as above, but first converts the cubic `polynomial` to its
// structured form representation.
icon::RealtimeStatusOr<double>
UpperLimitOfIntegralOfOneOverSqrtOfStructuredCubicPolynomial(
    const CubicPolynomial& polynomial, double integral_value);

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_ANALYTICAL_POLYNOMIAL_INTEGRALS_H_
