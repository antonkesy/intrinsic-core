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

#ifndef INTRINSIC_KINEMATICS_TYPES_IS_APPROX_WITH_INF_H_
#define INTRINSIC_KINEMATICS_TYPES_IS_APPROX_WITH_INF_H_

#include <gtest/gtest.h>

#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Checks if two eigenmath::VectorNd that potentially contain infinite values
// are approximately equal. Infinite values signs are compared for equality.
bool IsApproxWithInf(const eigenmath::VectorNd& a, const eigenmath::VectorNd& b,
                     double tolerance,
                     ::testing::MatchResultListener* result_listener);

// Checks if two numbers that potentially contain infinite values
// are approximately equal. Infinite values signs are compared for equality.
bool IsApproxWithInf(double a, double b, double tolerance,
                     ::testing::MatchResultListener* result_listener);

}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_TYPES_IS_APPROX_WITH_INF_H_
