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

#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomial_roots.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <initializer_list>
#include <iterator>
#include <numeric>

#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/almost_equals.h"

namespace intrinsic::topp {

namespace {

// Evaluates to true when complex root `a` is strictly less than `b` (meaning
// `a` should precede `b` in sorted order), and false otherwise. The ordering
// evaluates their real component first (e.g. the most negative real root first)
// and then, if the real components are numerically equal, their imaginary
// components (e.g. the most negative imaginary root first).
bool ComplexPolynomialRootsLessThan(const std::complex<double>& a,
                                    const std::complex<double>& b) {
  if (!::intrinsic::AlmostEquals(a.real(), b.real()))
    return a.real() < b.real();
  return a.imag() < b.imag();
}

// Comparator function to sort structured roots in ascending order.
bool ComparePolynomialStructuredRoots(const StructuredRoot& a,
                                      const StructuredRoot& b) {
  return ComplexPolynomialRootsLessThan(a.value, b.value);
}

// Checks that the structured `roots` of a polynomial of degree 2 or 3 are valid
// and sum up to the `expected_total_multiplicity`.
icon::RealtimeStatus ValidateStructuredRoots(
    absl::Span<const StructuredRoot> roots,
    const int expected_total_multiplicity) {
  int total_multiplicity = 0;
  int num_complex_roots = 0;

  // Combine multiplicity, complex typing, and finiteness checks into a single
  // pass.
  for (const auto& root : roots) {
    if (root.multiplicity <= 0) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "The multiplicity of a root must be positive. Got ",
          root.multiplicity, "."));
    }

    total_multiplicity += root.multiplicity;

    if (root.type == RootType::kComplex) {
      if (root.multiplicity != 1) {
        return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
            "The multiplicity of a complex root should be 1. Got ",
            root.multiplicity, "."));
      }
      num_complex_roots++;
    }

    if (std::isinf(root.value.real()) || std::isinf(root.value.imag()) ||
        std::isnan(root.value.real()) || std::isnan(root.value.imag())) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "The root values must be finite. Got ", root.value.real(), " + ",
          root.value.imag(), "i."));
    }
  }

  // Evaluate the aggregated totals
  if (total_multiplicity != expected_total_multiplicity) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The multiplicities should sum up to ", expected_total_multiplicity,
        ". Got ", total_multiplicity, "."));
  }

  if (num_complex_roots % 2 != 0) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The complex roots should be even. Got ", num_complex_roots, "."));
  }

  // The roots should be sorted in ascending order.
  if (!std::is_sorted(roots.begin(), roots.end(),
                      ComparePolynomialStructuredRoots)) {
    return icon::InvalidArgumentError(
        "The roots should be sorted in ascending order.");
  }

  return icon::OkStatus();
}

// Checks if the array contains any complex `roots`.
bool HasComplexRootsHelper(absl::Span<const StructuredRoot> roots) {
  for (const StructuredRoot& root : roots) {
    if (root.type == RootType::kComplex) return true;
  }
  return false;
}

// Checks if there is exactly one root within `roots`, it is real, and it has
// the `expected_multiplicity`.
bool HasSingleRealRootHelper(absl::Span<const StructuredRoot> roots,
                             const int expected_multiplicity) {
  if (roots.size() != 1) return false;
  if (roots[0].multiplicity != expected_multiplicity) return false;
  if (roots[0].type != RootType::kReal) return false;
  return true;
}

// Checks if all `roots` are purely real and have a multiplicity of exactly 1.
bool HasOnlyRealRootsWithMultiplicityOneHelper(
    absl::Span<const StructuredRoot> roots) {
  for (const StructuredRoot& root : roots) {
    if (root.multiplicity != 1 || root.type != RootType::kReal) return false;
  }
  return true;
}

// Merges adjacent real roots that are numerically equal (up to the given
// `tolerance`). Requires the array of `roots`to be sorted prior to calling.
// Returns the new number of roots.
int MergeRealRootDuplicatesHelper(absl::Span<StructuredRoot> roots,
                                  const double tolerance) {
  int root_id = 0;
  int num_roots = roots.size();
  while (root_id + 1 < num_roots) {
    if (::intrinsic::AlmostEquals(roots[root_id].value.real(),
                                  roots[root_id + 1].value.real(), tolerance)) {
      roots[root_id].multiplicity += roots[root_id + 1].multiplicity;
      for (int next_id = root_id + 1; next_id < num_roots - 1; ++next_id) {
        roots[next_id] = roots[next_id + 1];
      }
      num_roots--;
    } else {
      root_id++;
    }
  }
  return num_roots;
}

// Checks roots with extremely small imaginary components (below `tolerance`),
// zeros them out, and returns the accumulated multiplicity of these numerically
// real roots.
int MarkAlmostZeroImaginaryRootsAsReal(absl::Span<StructuredRoot> roots,
                                       const double tolerance) {
  int numerically_real_roots_count = 0;
  for (StructuredRoot& root : roots) {
    if (::intrinsic::AlmostEquals(root.value.imag(), 0.0, tolerance)) {
      numerically_real_roots_count += root.multiplicity;
      root.value = {root.value.real(), 0.0};
      root.type = RootType::kReal;
    }
  }
  return numerically_real_roots_count;
}

}  // namespace

// Quadratic Polynomial Implementations

icon::RealtimeStatus QuadraticPolynomialRoots::Sort() {
  std::sort(roots_.begin(), std::next(roots_.begin(), num_roots_),
            ComplexPolynomialRootsLessThan);
  return icon::OkStatus();
}

/* static */
icon::RealtimeStatusOr<QuadraticPolynomialStructuredRoots>
QuadraticPolynomialStructuredRoots::Compose(
    std::initializer_list<StructuredRoot> roots_list) {
  QuadraticPolynomialStructuredRoots roots;
  for (const StructuredRoot& root : roots_list)
    INTRINSIC_RT_RETURN_IF_ERROR(roots.Add(root));
  return roots;
}

/* static */
icon::RealtimeStatusOr<QuadraticPolynomialStructuredRoots>
QuadraticPolynomialStructuredRoots::ComposeSorted(
    std::initializer_list<StructuredRoot> roots_list) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(QuadraticPolynomialStructuredRoots roots,
                                Compose(roots_list));
  INTRINSIC_RT_RETURN_IF_ERROR(roots.Sort());
  return roots;
}

icon::RealtimeStatus QuadraticPolynomialStructuredRoots::IsValid() const {
  return ValidateStructuredRoots(absl::MakeConstSpan(roots_.data(), num_roots_),
                                 kNumRootsQuadraticPolynomial);
}

icon::RealtimeStatus QuadraticPolynomialStructuredRoots::Sort() {
  std::sort(roots_.begin(), std::next(roots_.begin(), num_roots_),
            ComparePolynomialStructuredRoots);
  return icon::OkStatus();
}

bool QuadraticPolynomialStructuredRoots::HasSingleRealRootWithMultiplicityTwo()
    const {
  return HasSingleRealRootHelper(absl::MakeConstSpan(roots_.data(), num_roots_),
                                 kNumRootsQuadraticPolynomial);
}

bool QuadraticPolynomialStructuredRoots::HasOnlyRealRootsWithMultiplicityOne()
    const {
  return HasOnlyRealRootsWithMultiplicityOneHelper(
      absl::MakeConstSpan(roots_.data(), num_roots_));
}

bool QuadraticPolynomialStructuredRoots::HasComplexRoots() const {
  return HasComplexRootsHelper(absl::MakeConstSpan(roots_.data(), num_roots_));
}

icon::RealtimeStatus
QuadraticPolynomialStructuredRoots::MarkComplexConjugateRootTypes() {
  if (num_roots_ != kNumRootsQuadraticPolynomial) {
    return icon::OutOfRangeError(icon::RealtimeStatus::StrCat(
        "The number of roots should be ", kNumRootsQuadraticPolynomial,
        ". Got ", num_roots_, "."));
  }

  // Both roots must be complex for a quadratic polynomial.
  INTRINSIC_RT_RETURN_IF_ERROR(Sort());
  roots_[0].type = RootType::kComplex;
  roots_[1].type = RootType::kComplex;

  return icon::OkStatus();
}

icon::RealtimeStatus
QuadraticPolynomialStructuredRoots::MergeRealRootDuplicates(
    const double tolerance) {
  // If a quadratic polynomial has complex roots, they are already unique.
  if (HasComplexRoots()) return icon::OkStatus();

  // If we already have 1 root (multiplicity 2), nothing to do.
  if (num_roots_ < 2) return icon::OkStatus();

  // To merge the numerically equal roots, we first sort them in place in
  // ascending order and then check if the real roots are numerically equal.
  INTRINSIC_RT_RETURN_IF_ERROR(Sort());
  num_roots_ = MergeRealRootDuplicatesHelper(
      absl::MakeSpan(roots_.data(), num_roots_), tolerance);
  return icon::OkStatus();
}

icon::RealtimeStatus QuadraticPolynomialStructuredRoots::
    MarkRootsWithAlmostZeroButPositiveDiscrimimant(const double tolerance) {
  if (num_roots_ != kNumRootsQuadraticPolynomial) return icon::OkStatus();

  // Count the number of roots, whose imaginary part is numerically zero. If at
  // the end, the number of real roots is not 2, then they have small imaginary
  // components and we mark the complex conjugate roots as complex conjugate.
  const int numerically_real_roots_count = MarkAlmostZeroImaginaryRootsAsReal(
      absl::MakeSpan(roots_.data(), num_roots_), tolerance);

  if (numerically_real_roots_count != kNumRootsQuadraticPolynomial)
    INTRINSIC_RT_RETURN_IF_ERROR(MarkComplexConjugateRootTypes());

  return icon::OkStatus();
}

// Cubic Polynomial Implementations

icon::RealtimeStatus CubicPolynomialRoots::Sort() {
  std::sort(roots_.begin(), std::next(roots_.begin(), num_roots_),
            ComplexPolynomialRootsLessThan);
  return icon::OkStatus();
}

/* static */
icon::RealtimeStatusOr<CubicPolynomialStructuredRoots>
CubicPolynomialStructuredRoots::Compose(
    std::initializer_list<StructuredRoot> roots_list) {
  CubicPolynomialStructuredRoots roots;
  for (const auto& root : roots_list)
    INTRINSIC_RT_RETURN_IF_ERROR(roots.Add(root));
  return roots;
}

/* static */
icon::RealtimeStatusOr<CubicPolynomialStructuredRoots>
CubicPolynomialStructuredRoots::ComposeSorted(
    std::initializer_list<StructuredRoot> roots_list) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(CubicPolynomialStructuredRoots roots,
                                Compose(roots_list));
  INTRINSIC_RT_RETURN_IF_ERROR(roots.Sort());
  return roots;
}

icon::RealtimeStatus CubicPolynomialStructuredRoots::IsValid() const {
  return ValidateStructuredRoots(absl::MakeConstSpan(roots_.data(), num_roots_),
                                 kNumRootsCubicPolynomial);
}

icon::RealtimeStatus CubicPolynomialStructuredRoots::Sort() {
  std::sort(roots_.begin(), std::next(roots_.begin(), num_roots_),
            ComparePolynomialStructuredRoots);
  return icon::OkStatus();
}

bool CubicPolynomialStructuredRoots::HasSingleRealRootWithMultiplicityThree()
    const {
  return HasSingleRealRootHelper(absl::MakeConstSpan(roots_.data(), num_roots_),
                                 kNumRootsCubicPolynomial);
}

bool CubicPolynomialStructuredRoots::HasRealRootWithMultiplicityTwo() const {
  for (int i = 0; i < num_roots_; ++i) {
    if (roots_[i].multiplicity == 2 && roots_[i].type == RootType::kReal)
      return true;
  }
  return false;
}

bool CubicPolynomialStructuredRoots::HasOnlyRealRootsWithMultiplicityOne()
    const {
  return HasOnlyRealRootsWithMultiplicityOneHelper(
      absl::MakeConstSpan(roots_.data(), num_roots_));
}

bool CubicPolynomialStructuredRoots::HasComplexRoots() const {
  return HasComplexRootsHelper(absl::MakeConstSpan(roots_.data(), num_roots_));
}

icon::RealtimeStatus
CubicPolynomialStructuredRoots::MarkComplexConjugateRootTypes() {
  if (num_roots_ != kNumRootsCubicPolynomial) {
    return icon::OutOfRangeError(icon::RealtimeStatus::StrCat(
        "The number of roots should be ", kNumRootsCubicPolynomial, ". Got ",
        num_roots_, "."));
  }

  // Check the closest pair of roots.
  INTRINSIC_RT_RETURN_IF_ERROR(Sort());
  const double dist01 = std::norm(roots_[0].value - std::conj(roots_[1].value));
  const double dist12 = std::norm(roots_[1].value - std::conj(roots_[2].value));
  if (dist01 < dist12) {
    roots_[0].type = RootType::kComplex;
    roots_[1].type = RootType::kComplex;
    roots_[2].type = RootType::kReal;
  } else {
    roots_[0].type = RootType::kReal;
    roots_[1].type = RootType::kComplex;
    roots_[2].type = RootType::kComplex;
  }
  return icon::OkStatus();
}

icon::RealtimeStatus CubicPolynomialStructuredRoots::MergeRealRootDuplicates(
    const double tolerance) {
  // If a cubic polynomial has complex roots, they are already unique.
  if (HasComplexRoots()) return icon::OkStatus();

  // To merge the numerically equal roots, we first sort them in place in
  // ascending order and then check if the real roots are numerically equal.
  INTRINSIC_RT_RETURN_IF_ERROR(Sort());
  num_roots_ = MergeRealRootDuplicatesHelper(
      absl::MakeSpan(roots_.data(), num_roots_), tolerance);
  return icon::OkStatus();
}

icon::RealtimeStatus
CubicPolynomialStructuredRoots::MarkRootsWithAlmostZeroButPositiveDiscrimimant(
    const double tolerance) {
  // Check that we have three distinct roots.
  if (num_roots_ != kNumRootsCubicPolynomial) return icon::OkStatus();

  // Count the number of roots, whose imaginary part is numerically zero. If at
  // the end, the number of real roots is not 3, then they have small imaginary
  // components and we mark the complex conjugate roots as complex conjugate.
  const int numerically_real_roots_count = MarkAlmostZeroImaginaryRootsAsReal(
      absl::MakeSpan(roots_.data(), num_roots_), tolerance);
  if (numerically_real_roots_count != kNumRootsCubicPolynomial)
    INTRINSIC_RT_RETURN_IF_ERROR(MarkComplexConjugateRootTypes());
  return icon::OkStatus();
}

}  // namespace intrinsic::topp
