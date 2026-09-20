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

#include "intrinsic/kinematics/types/is_approx_with_inf.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>

#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// eigenmath::testing::IsApprox doesn't correctly handle infinite values since
// it computes the norm of the difference between the two vectors.
bool IsApproxWithInf(const eigenmath::VectorNd& a, const eigenmath::VectorNd& b,
                     double tolerance,
                     ::testing::MatchResultListener* result_listener) {
  if (a.allFinite() && b.allFinite()) {
    return (a - b).norm() < tolerance;
  }
  if (a.size() != b.size()) {
    *result_listener << "size mismatch: " << a.size() << " vs " << b.size();
    return false;
  }
  for (Eigen::Index i = 0; i < a.size(); ++i) {
    const bool is_approx =
        IsApproxWithInf(a[i], b[i], tolerance, result_listener);
    if (!is_approx) {
      *result_listener << " at position " << i;
      return false;
    }
  }
  return true;
}

// Checks if two numbers that potentially contain infinite values
// are approximately equals. Infinite values signs are compared for equality.
bool IsApproxWithInf(double a, double b, double tolerance,
                     ::testing::MatchResultListener* result_listener) {
  if (isinf(a) && isinf(b)) {
    int sign_a = static_cast<int>(a > 0) - static_cast<int>(a < 0);
    int sign_b = static_cast<int>(b > 0) - static_cast<int>(b < 0);
    if (sign_a != sign_b) {
      *result_listener << "infinity sign differ";
      return false;
    }
  } else {
    double error = std::abs(a - b);
    if (error > tolerance) {
      *result_listener << "tolerance exceeded, error=" << error
                       << ", tolerance=" << tolerance;
      return false;
    }
  }
  return true;
}
}  // namespace intrinsic
