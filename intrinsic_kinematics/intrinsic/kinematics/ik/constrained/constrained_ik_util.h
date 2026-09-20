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

#ifndef INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINED_IK_UTIL_H_
#define INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINED_IK_UTIL_H_

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik.pb.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

// Returns a pseudo-random joint configuration drawn from a uniform distribution
// between the `chain`'s joint position limits. The `halton_sequence_index` can
// be provided to seed the random number generation. `halton_sequence_index`
// will be incremented by one before returning an Ok-status. Infinite degrees of
// freedom are wrapped to the range [-M_PI, M_PI].
absl::StatusOr<eigenmath::VectorNd> GetPseudoRandomConfiguration(
    const Chain& chain, int& halton_sequence_index);

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINED_IK_UTIL_H_
