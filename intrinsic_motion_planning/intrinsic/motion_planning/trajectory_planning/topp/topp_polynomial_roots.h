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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_POLYNOMIAL_ROOTS_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_POLYNOMIAL_ROOTS_H_

#include <array>
#include <complex>
#include <initializer_list>
#include <limits>

#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/almost_equals.h"

namespace intrinsic::topp {

// The number of roots of a quadratic polynomial.
constexpr int kNumRootsQuadraticPolynomial = 2;

// The number of roots of a cubic polynomial.
constexpr int kNumRootsCubicPolynomial = 3;

// Marks the type of a root of a polynomial as being either real or complex.
enum class RootType {
  kReal,
  kComplex,
};

// Represents a root of a polynomial with its type, value and multiplicity.
struct StructuredRoot {
  RootType type;
  std::complex<double> value;
  int multiplicity = 1;
};

// Storage for the roots of a polynomial. The number of roots to be stored is
// limited to `kMaxRoots`.
template <typename RootType, int kMaxRoots>
class PolynomialRoots {
 public:
  // Returns the number of roots stored.
  int size() const { return num_roots_; }

  // Adds a root to the storage. Returns an error if the number of roots to be
  // stored exceeds the maximum number.
  icon::RealtimeStatus Add(RootType root) {
    if (num_roots_ >= kMaxRoots)
      return icon::FailedPreconditionError(
          "The number of roots to be stored exceeds the maximum number.");
    roots_[num_roots_] = root;
    ++num_roots_;
    return icon::OkStatus();
  }

  // Returns the root at the given `index`. Returns an error if the `index` is
  // out of bounds.
  icon::RealtimeStatusOr<RootType> Get(int index) const {
    if (index < 0 || index >= num_roots_)
      return icon::OutOfRangeError(icon::RealtimeStatus::StrCat(
          "The `index` must be in [0, ", kMaxRoots, "). Got: ", index, "."));
    return roots_[index];
  }

  // Updates the root at the given `index` with the given `root`. Returns an
  // error if the `index` is out of bounds.
  icon::RealtimeStatus Update(int index, RootType root) {
    if (index < 0 || index >= num_roots_)
      return icon::OutOfRangeError(icon::RealtimeStatus::StrCat(
          "The `index` is out of bounds [0, ", num_roots_, ")."));
    roots_[index] = root;
    return icon::OkStatus();
  }

 protected:
  int num_roots_ = 0;
  std::array<RootType, kMaxRoots> roots_;
};

// Realtime safe storage for the roots of a quadratic polynomial.
class QuadraticPolynomialRoots
    : public PolynomialRoots<std::complex<double>,
                             kNumRootsQuadraticPolynomial> {
 public:
  // Helper method to sort the roots in ascending order.
  icon::RealtimeStatus Sort();
};

// Realtime safe storage for the structured roots of a quadratic polynomial.
class QuadraticPolynomialStructuredRoots
    : public PolynomialRoots<StructuredRoot, kNumRootsQuadraticPolynomial> {
 public:
  // Helper method to compose structured roots from a list of roots.
  static icon::RealtimeStatusOr<QuadraticPolynomialStructuredRoots> Compose(
      std::initializer_list<StructuredRoot> roots_list);

  // Same as above, but the roots are sorted in ascending order.
  static icon::RealtimeStatusOr<QuadraticPolynomialStructuredRoots>
  ComposeSorted(std::initializer_list<StructuredRoot> roots_list);

  // Checks whether the roots are valid.
  icon::RealtimeStatus IsValid() const;

  // Helper method to sort the roots in ascending order.
  icon::RealtimeStatus Sort();

  // Returns true if there is a single real root of multiplicity 2.
  bool HasSingleRealRootWithMultiplicityTwo() const;

  // Returns true if there are only real roots of multiplicity 1.
  bool HasOnlyRealRootsWithMultiplicityOne() const;

  // Returns true if there are complex roots.
  bool HasComplexRoots() const;

  // For a quadratic, if roots are complex, both must be complex conjugates.
  // This marks both roots as complex.
  icon::RealtimeStatus MarkComplexConjugateRootTypes();

  // Compares the real part of the structured roots and merges the duplicates
  // using the desired comparison `tolerance`.
  icon::RealtimeStatus MergeRealRootDuplicates(
      double tolerance = ::intrinsic::kStdError);

  // Checks if the imaginary components of the roots are numerically close to
  // zero. If they are, they are marked as real, otherwise they are marked as
  // complex conjugate.
  icon::RealtimeStatus MarkRootsWithAlmostZeroButPositiveDiscrimimant(
      double tolerance = std::numeric_limits<double>::epsilon());
};

// Realtime safe storage for the roots of a cubic polynomial.
class CubicPolynomialRoots
    : public PolynomialRoots<std::complex<double>, kNumRootsCubicPolynomial> {
 public:
  // Helper method to sort the roots in ascending order.
  icon::RealtimeStatus Sort();
};

// Realtime safe storage for the structured roots of a cubic polynomial.
class CubicPolynomialStructuredRoots
    : public PolynomialRoots<StructuredRoot, kNumRootsCubicPolynomial> {
 public:
  // Helper method to compose structured roots from a list of roots.
  static icon::RealtimeStatusOr<CubicPolynomialStructuredRoots> Compose(
      std::initializer_list<StructuredRoot> roots_list);

  // Same as above, but the roots are sorted in ascending order.
  static icon::RealtimeStatusOr<CubicPolynomialStructuredRoots> ComposeSorted(
      std::initializer_list<StructuredRoot> roots_list);

  // Checks whether the roots are valid, i.e. if the number of roots is correct,
  // if the sum of the multiplicities is correct and they are positive, that
  // there is a pair of complex roots if there are complex roots, that the
  // values are finite and sorted in ascending order.
  icon::RealtimeStatus IsValid() const;

  // Helper method to sort the roots in ascending order.
  icon::RealtimeStatus Sort();

  // Returns true if there is a single real root of multiplicity 3.
  bool HasSingleRealRootWithMultiplicityThree() const;

  // Returns true if the there is a real root of multiplicity 2.
  bool HasRealRootWithMultiplicityTwo() const;

  // Returns true if there are only real roots of multiplicity 1.
  bool HasOnlyRealRootsWithMultiplicityOne() const;

  // Returns true if there are complex roots.
  bool HasComplexRoots() const;

  // This method assumes that a pair of roots are complex conjugate. It
  // identifies this pair (closest roots) and marks the pair as complex
  // conjugate. It also identifies the real root and marks it as real.
  icon::RealtimeStatus MarkComplexConjugateRootTypes();

  // Compares the real part of the structured roots and merges the duplicates
  // using the desired comparison `tolerance`.
  icon::RealtimeStatus MergeRealRootDuplicates(
      double tolerance = ::intrinsic::kStdError);

  // When the discriminant of the polynomial is close to zero (but could also be
  // slightly positive), the polynomial has either real roots with multiplicity
  // or complex conjugate roots. This function checks if the imaginary
  // components of the roots are numerically close to zero. If they are, they
  // are marked as real, otherwise they are marked as complex conjugate.
  icon::RealtimeStatus MarkRootsWithAlmostZeroButPositiveDiscrimimant(
      double tolerance = std::numeric_limits<double>::epsilon());
};

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_POLYNOMIAL_ROOTS_H_
