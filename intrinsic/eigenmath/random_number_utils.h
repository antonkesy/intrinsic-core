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

#ifndef INTRINSIC_EIGENMATH_RANDOM_NUMBER_UTILS_H_
#define INTRINSIC_EIGENMATH_RANDOM_NUMBER_UTILS_H_

#include <cmath>
#include <cstddef>
#include <limits>

#include "absl/log/absl_check.h"
#include "absl/random/random.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

// Uses MakeSeedSeq to create random generator. See documentation of
// MakeSeedSeq for more details on how to use the seed.
absl::BitGen GetRandomBitGenerator();

namespace eigenmath {

namespace details {
// Verify that the upper limits is greated than lower limits.
absl::Status VerifyLimits(const eigenmath::VectorXd& lower_limits,
                          const eigenmath::VectorXd& upper_limits)
    INTRINSIC_NON_REALTIME_ONLY;

icon::RealtimeStatus VerifyLimitsRT(const eigenmath::VectorNd& lower_limits,
                                    const eigenmath::VectorNd& upper_limits);

}  // namespace details

// Generate a random vector from a uniform distribution within the provided
// limits. Upper limits are not verified to be >= lower limits. The Generator is
// expected to be an absl::BitGen or an util_random::SharedBitGen.
template <typename Vector, typename Generator>
Vector UnverifiedGetUniformRandomVector(const Vector& lower_limits,
                                        const Vector& upper_limits,
                                        Generator& gen) {
  size_t q_length = lower_limits.size();

  // For memory access safety:
  ABSL_CHECK_EQ(q_length, upper_limits.size());

  Vector random_q(q_length);
  for (int i = 0; i < q_length; ++i) {
    double lower = lower_limits[i];
    double upper = upper_limits[i];
    if (!std::isfinite(lower)) {
      lower = std::numeric_limits<double>::lowest();
    }
    if (!std::isfinite(upper)) {
      upper = std::numeric_limits<double>::max();
    }
    random_q[i] =
        absl::Uniform<double>(absl::IntervalClosedClosed, gen, lower, upper);
  }
  return random_q;
}

// Gets a quasirandom VectorXd configuration (using a Halton Sequence) within
// the given limits and seeding the random number generator with the provided
// seed. The dimensionality of the returned vector corresponds to the size of
// the provided limit vectors. The provided seed will be increased by one.
// The algorithm fails if no seed is provided (i.e., nullptr), the lower limit
// is not smaller than the upper limit, or the limits have different
// dimensionality.
// See the following paper for details:
// Deterministic Sampling-Based Motion Planning: Optimality, Complexity, and
// Performance (2016) Lucas Janson et. al. https://arxiv.org/abs/1505.00023
absl::StatusOr<eigenmath::VectorXd> GetQuasiRandomVectorXd(
    const VectorXd& lower_limits, const VectorXd& upper_limits,
    int* seed) INTRINSIC_NON_REALTIME_ONLY;

// Generate a random vector from a uniform distribution within the provided
// limits. The Generator is expected to be an absl::BitGen or an
// util_random::SharedBitGen.
template <typename Generator>
absl::StatusOr<eigenmath::VectorXd> GetUniformRandomVectorXd(
    const VectorXd& lower_limits, const VectorXd& upper_limits,
    Generator& gen) INTRINSIC_NON_REALTIME_ONLY {
  INTR_RETURN_IF_ERROR(details::VerifyLimits(lower_limits, upper_limits));
  return UnverifiedGetUniformRandomVector(lower_limits, upper_limits, gen);
}

// A real-time version of above.
template <typename Generator>
icon::RealtimeStatusOr<eigenmath::VectorNd> GetUniformRandomVectorNd(
    const VectorNd& lower_limits, const VectorNd& upper_limits,
    Generator& gen) {
  INTRINSIC_RT_RETURN_IF_ERROR(
      details::VerifyLimitsRT(lower_limits, upper_limits));
  return UnverifiedGetUniformRandomVector(lower_limits, upper_limits, gen);
}

// Returns a random unit quaternion sampled uniformly from SO(3).
//
// This formula came from Lavalle's Planning Algorithms:
//
// https://lavalle.pl/planning/node198.html
template <typename Generator>
eigenmath::Quaterniond GetUniformRandomQuaterniond(Generator& gen) {
  const double u1 =
      absl::Uniform<double>(absl::IntervalClosedClosed, gen, 0.0, 1.0);
  const double u2 =
      absl::Uniform<double>(absl::IntervalClosedClosed, gen, 0.0, 1.0);
  const double u3 =
      absl::Uniform<double>(absl::IntervalClosedClosed, gen, 0.0, 1.0);
  const double sqrt_1_u1 = std::sqrt(1 - u1);
  const double sqrt_u1 = std::sqrt(u1);
  return eigenmath::Quaterniond(
      sqrt_1_u1 * std::sin(2 * M_PI * u2), sqrt_1_u1 * std::cos(2 * M_PI * u2),
      sqrt_u1 * std::sin(2 * M_PI * u3), sqrt_u1 * std::cos(2 * M_PI * u3));
}

}  // namespace eigenmath
}  // namespace intrinsic

#endif  // INTRINSIC_EIGENMATH_RANDOM_NUMBER_UTILS_H_
