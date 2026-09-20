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

#include "intrinsic/kinematics/ik/constrained/constrained_ik_util.h"

#include <cmath>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {

absl::StatusOr<eigenmath::VectorNd> GetPseudoRandomConfiguration(
    const Chain& chain, int& halton_sequence_index) {
  JointLimits sampling_joint_limits = chain.GetDofSystemLimits();

  // Replace -inf/inf with concrete values.
  for (int i = 0; i < chain.GetNumberDegreesOfFreedom(); i++) {
    if (std::isinf(sampling_joint_limits.min_position[i])) {
      sampling_joint_limits.min_position(i) = -M_PI;
    }
    if (std::isinf(sampling_joint_limits.max_position[i])) {
      sampling_joint_limits.max_position(i) = M_PI;
    }
  }

  const int kMaxSamplingAttemts = 1000;
  for (int i = 0; i < kMaxSamplingAttemts; ++i) {
    INTR_ASSIGN_OR_RETURN(
        const eigenmath::VectorNd random_joint_config,
        eigenmath::GetQuasiRandomVectorXd(sampling_joint_limits.min_position,
                                          sampling_joint_limits.max_position,
                                          &halton_sequence_index));
    auto limit_check_result_or =
        IsWithinLimits(random_joint_config, sampling_joint_limits);
    if (limit_check_result_or.ok()) {
      if (limit_check_result_or.value().p_ok) {
        return random_joint_config;
      }
    }
  }
  return absl::InternalError(
      "Random sampling of a joint configuration failed after 1000 attempts.");
}

}  // namespace kinematics
}  // namespace intrinsic
