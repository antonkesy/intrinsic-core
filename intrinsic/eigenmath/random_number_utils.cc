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

#include "intrinsic/eigenmath/random_number_utils.h"

#include <cmath>
#include <limits>

#include "absl/log/absl_check.h"
#include "absl/random/random.h"
#include "absl/random/seed_sequences.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/halton_sequence.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

absl::BitGen GetRandomBitGenerator() {
  // Use Error log level because it usually ends up on stdout
  auto seed = absl::MakeSeedSeq();
  return absl::BitGen(seed);
}

namespace eigenmath {

namespace details {

absl::Status VerifyLimits(const eigenmath::VectorXd& lower_limits,
                          const eigenmath::VectorXd& upper_limits) {
  if (lower_limits.size() != upper_limits.size()) {
    return absl::InvalidArgumentError(
        "Lower and upper vector limits must be the same size.");
  }

  if ((upper_limits - lower_limits).minCoeff() <= 0) {
    return absl::InvalidArgumentError(
        "At least one of the lower bounds is greater than the upper bounds.");
  }
  return absl::OkStatus();
}

icon::RealtimeStatus VerifyLimitsRT(const eigenmath::VectorNd& lower_limits,
                                    const eigenmath::VectorNd& upper_limits) {
  if (lower_limits.size() != upper_limits.size()) {
    return icon::InvalidArgumentError(
        "Lower and upper vector limits must be the same size.");
  }

  if ((upper_limits - lower_limits).minCoeff() <= 0) {
    return icon::InvalidArgumentError(
        "At least one of the lower bounds is greater than the upper bounds.");
  }
  return icon::OkStatus();
}

}  // namespace details

absl::StatusOr<eigenmath::VectorXd> GetQuasiRandomVectorXd(
    const eigenmath::VectorXd& lower_limits,
    const eigenmath::VectorXd& upper_limits, int* seed) {
  ABSL_CHECK(seed != nullptr);

  INTR_RETURN_IF_ERROR(details::VerifyLimits(lower_limits, upper_limits));

  // It's possible for us to wrap around while incrementing the seed. In this
  // case the random generator may become degenerate. This check ensures we
  // avoid this case.
  if (*seed < 0) {
    *seed = 0;
  }

  eigenmath::VectorXd random_q(lower_limits.size());
  for (int i = 0; i < lower_limits.size(); ++i) {
    double r = HaltonSequence(*seed, boost::math::prime(i));
    // Check for infinite limits
    double lower = lower_limits[i];
    double upper = upper_limits[i];
    if (!std::isfinite(lower)) {
      lower = std::numeric_limits<double>::lowest();
    }
    if (!std::isfinite(upper)) {
      upper = std::numeric_limits<double>::max();
    }

    random_q[i] = r * upper + (1 - r) * lower;
  }
  (*seed)++;

  return random_q;
}

}  // namespace eigenmath
}  // namespace intrinsic
