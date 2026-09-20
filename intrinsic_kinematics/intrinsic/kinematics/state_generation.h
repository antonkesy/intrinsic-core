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

#ifndef INTRINSIC_KINEMATICS_STATE_GENERATION_H_
#define INTRINSIC_KINEMATICS_STATE_GENERATION_H_

#include <cmath>
#include <random>

#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/util/macros.h"

namespace intrinsic {
namespace kinematics {

// Get a random joint configuration from a uniform distribution within the
// provided position limits. The Generator is expected to be absl::BitGen,
// util_random::SharedBitGen or std::default_random_engine.
template <typename Generator>
eigenmath::VectorNd GetUniformRandomConfiguration(const JointLimits& limits,
                                                  Generator& gen) {
  eigenmath::VectorXd lower_limits = limits.min_position;
  eigenmath::VectorXd upper_limits = limits.max_position;
  // Replace -inf/inf with concrete values.
  for (int i = 0; i < limits.size(); i++) {
    if (std::isinf(lower_limits[i])) {
      lower_limits(i) = -M_PI;
    }
    if (std::isinf(upper_limits[i])) {
      upper_limits(i) = M_PI;
    }
  }

  INTRINSIC_RT_ASSIGN_OR_DIE(
      eigenmath::VectorNd random_q,
      eigenmath::GetUniformRandomVectorNd(lower_limits, upper_limits, gen));
  return random_q;
}

// The random number generator used underneath is deterministic and uses a fixed
// seed. Different threads calling this method will get the same random
// configuration sequence.
inline eigenmath::VectorNd GetDeterministicUniformRandomConfiguration(
    const JointLimits& limits) {
  static thread_local std::default_random_engine generator(0);
  return GetUniformRandomConfiguration(limits, generator);
}

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_STATE_GENERATION_H_
