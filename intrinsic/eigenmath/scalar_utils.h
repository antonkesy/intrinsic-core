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

#ifndef INTRINSIC_EIGENMATH_SCALAR_UTILS_H_
#define INTRINSIC_EIGENMATH_SCALAR_UTILS_H_

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <type_traits>
#include <utility>

namespace intrinsic {
namespace eigenmath {

// Saturate value to -min_max, min_max.
template <typename T>
inline constexpr T Saturate(const T& val, const T& min_max) {
  return std::clamp(val, -min_max, min_max);
}

// Computes the cardinal sine function "sin(x) / x".
// This implementation avoids the singularity at 0.
template <typename T>
inline T Sinc(T x) {
  using std::abs;
  using std::sin;
  if (abs(x) < std::numeric_limits<T>::epsilon()) {
    return T(1.0);
  }
  return sin(x) / x;
}

// Wraps to the open interval [0, upper_bound).
// Wrap for floating point types.
template <typename T>
inline constexpr T Wrap(T x, T upper_bound) {
  static_assert(std::is_floating_point_v<T>,
                "Only floating point values are supported.");
  return x - (upper_bound * std::floor(x / upper_bound));
}

// Wraps to the open interval [lower,upper).
template <typename T>
inline constexpr T Wrap(T x, T lower_bound, T upper_bound) {
  return lower_bound + Wrap(x - lower_bound, upper_bound - lower_bound);
}

// Return the square of the input.
template <class T>
inline constexpr T Square(T a) {
  return a * a;
}

// Compute the roots of a quadratic expression.
// Solves: a*x^2 + b*x + c = 0
// Returns the number of real roots found:
//  - -1 if the quadratic is ill-formed (a == 0 and b == 0)
//  - 0 if roots are complex (root1_re +- root_im * i)
//  - 1 if there are real repeated roots
//  - 2 if there are 2 real roots
// If the imaginary parts are set to null (default), then complex roots are
// not calculated.
template <typename T>
int ComputeQuadraticRoots(T a, T b, T c, T* root1_re, T* root2_re,
                          T* root_im = nullptr) {
  // Some ADL-friendly using clauses (because we are in a function template).
  using std::abs;
  using std::sqrt;
  using std::swap;
  if (root_im != nullptr) {
    *root_im = 0.0;
  }
  if (abs(a) < std::numeric_limits<T>::epsilon()) {
    // Linear equation.
    if (abs(b) < std::numeric_limits<T>::epsilon()) {
      // Infeasible equation (0*x^2 + 0*x + c = 0).
      return -1;
    }
    *root1_re = -c / b;
    *root2_re = *root1_re;
    return 1;
  }
  const T discriminant = b * b - 4.0 * a * c;
  if (discriminant < -std::numeric_limits<T>::epsilon()) {
    // Complex roots, root1_re +- root_im * i.
    if (root_im != nullptr) {
      *root1_re = -b / (2.0 * a);
      *root2_re = *root1_re;
      *root_im = sqrt(-discriminant) / (2.0 * a);
    }
    return 0;
  }
  if (discriminant > std::numeric_limits<T>::epsilon()) {
    // Two real roots.
    const T sqrt_disc = sqrt(discriminant);
    if (b > 0.0) {
      *root1_re = (-b - sqrt_disc) / (2.0 * a);
    } else {
      *root1_re = (-b + sqrt_disc) / (2.0 * a);
    }
    *root2_re = c / (a * (*root1_re));
    if (*root1_re > *root2_re) {
      swap(*root1_re, *root2_re);
    }
    return 2;
  }
  // One real repeated root.
  *root1_re = -b / (2.0 * a);
  *root2_re = *root1_re;
  return 1;
}

//------------------------------------------------------------------------------
/**
 * @brief Return whether dividing two floating point numbers is safe
 *
 * We want to confirm that division doesn't produce a number that is over
 * the maximum - which can be expressed like this:
 *
 * max_float_val > (abs(numer) / abs(denom))
 *
 * The max float value is the same as the inverse of the min float value:
 * (1 / min_float_val) > ((abs(numer) / abs(denom))
 *
 * Since abs(denom) is positive, we can multiply both sides of the inequality
 * by that value:
 * (abs(denom) / min_float_val) > abs(numer)
 *
 * This is the test that we perform to make sure the division is valid.
 * We only bother with this test if the absolute value of the denominator
 * is less than 1.
 */
template <typename T>
constexpr bool isSafeDivide(T numer, T denom) {
  using std::abs;
  return (std::is_integral<T>::value
              ? (denom != T{0})
              : ((abs(denom) >= T{1}) ||
                 ((abs(denom) / std::numeric_limits<T>::min()) > abs(numer))));
}

}  // namespace eigenmath
}  // namespace intrinsic

#endif  // INTRINSIC_EIGENMATH_SCALAR_UTILS_H_
