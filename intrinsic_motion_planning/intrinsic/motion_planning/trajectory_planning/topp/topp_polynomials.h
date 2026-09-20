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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_POLYNOMIALS_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_POLYNOMIALS_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomial_roots.h"

namespace intrinsic::topp {

// Struct that represents a quadratic polynomial used to compose a piecewise
// quadratic polynomial. It stores the coefficients `coeffs` in the form of a 3d
// vector with the following order: quadratic (coeffs[0]), linear (coeffs[1]),
// and constant (coeffs[2]) terms. It also stores the valid integration range
// [`range_start`, `range_end`].
struct QuadraticPolynomial {
  eigenmath::Vector3d coeffs = eigenmath::Vector3d::Zero();
  double range_start = 0.0;
  double range_end = 0.0;

  // Computes the roots of a quadratic polynomial with real coefficients.
  // The roots are returned in a sorted order: starting from the most negative
  // in the real component, and then the most negative imaginary component.
  icon::RealtimeStatusOr<QuadraticPolynomialRoots> ComputeRoots() const;

  // Same as the above, but returns the roots in a structured form.
  icon::RealtimeStatusOr<QuadraticPolynomialStructuredRoots>
  ComputeStructuredRoots() const;

  double EvaluatePolynomial(double x) const;
  double EvaluatePolynomialFirstDerivative(double x) const;
  double EvaluatePolynomialSecondDerivative(double x) const;
};

// Struct that represents a structured quadratic polynomial used to compose a
// piecewise quadratic polynomial. It represents the polynomial in the form of a
// product between a `scaling_coefficient` and all `structured_roots`.
struct StructuredQuadraticPolynomial {
  double scaling_coefficient = 1.0;
  QuadraticPolynomialStructuredRoots structured_roots;
  double range_start = 0.0;
  double range_end = 0.0;

  // Returns an error if the `structured_polynomial` is invalid, i.e.
  // - if the scaling coefficient is close to zero,
  // - if the integration range is invalid.
  // - if the structured roots are invalid.
  icon::RealtimeStatus IsValid() const;

  icon::RealtimeStatusOr<double> EvaluatePolynomial(double x) const;
  icon::RealtimeStatusOr<double> EvaluatePolynomialFirstDerivative(
      double x) const;
};

// Struct that represents a cubic polynomial used to compose a piecewise
// cubic polynomial. It stores the coefficients `coeffs` in the form of a 4d
// vector with the following order: cubic (coeffs[0]), quadratic (coeffs[1]),
// linear (coeffs[2]) and constant (coeffs[3]) terms. It also stores the range
// in which the polynomial is valid or the range in which it should be
// integrated [`range_start`, `range_end`]. No error is incurred if the
// polynomial is evaluated outside the defined range, so it is up to the user to
// query it with an appropriate value.
struct CubicPolynomial {
  eigenmath::Vector4d coeffs = eigenmath::Vector4d::Zero();
  double range_start = 0.0;
  double range_end = 0.0;

  // Computes the roots of a cubic polynomial with real coefficients. This
  // guarantees that there exists at least one real root and either two
  // imaginary ones or two more real roots. The roots are returned in a sorted
  // order: starting from the most negative in the real component, and for those
  // with matching real component starting from the most negative in the
  // imaginary component.
  icon::RealtimeStatusOr<CubicPolynomialRoots> ComputeRoots() const;

  // Same as the above, but returns the roots in a structured form. Details
  // about this can be found in go/intrinsic-squared-path-velocity-integrals.
  icon::RealtimeStatusOr<CubicPolynomialStructuredRoots>
  ComputeStructuredRoots() const;

  double EvaluatePolynomial(double x) const;
  double EvaluatePolynomialFirstDerivative(double x) const;
  double EvaluatePolynomialSecondDerivative(double x) const;
};

// Struct that represents a structured cubic polynomial used to compose a
// piecewise cubic polynomial. It represents the polynomial in the form of a
// product between
// - a `scaling_coefficient` and,
// - all `structured_roots` describing a term of the form
//   (x - structured_roots[id].value) ^ structured_roots[id].multiplicity.
// It also stores the range in which the polynomial is valid or the range in
// which it should be integrated [`range_start`, `range_end`].
// The benefit of this representation is twofold:
// - it is more amenable for a structured approach to computing integrals.
// - it allows to handle edge cases when roots have multiplicity > 1.
struct StructuredCubicPolynomial {
  double scaling_coefficient = 1.0;
  CubicPolynomialStructuredRoots structured_roots;
  double range_start = 0.0;
  double range_end = 0.0;

  // Returns an error if the `structured_polynomial` is invalid, i.e.
  // - if the scaling coefficient is close to zero,
  // - if the integration range is invalid.
  // - if the structured roots are invalid (invalid multiplicity, invalid sum of
  //   multiplicities, invalid combination of root types).
  icon::RealtimeStatus IsValid() const;

  icon::RealtimeStatusOr<double> EvaluatePolynomial(double x) const;
};

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_POLYNOMIALS_H_
