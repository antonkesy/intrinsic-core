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

#ifndef INTRINSIC_KINEMATICS_IK_SANITIZE_IK_H_
#define INTRINSIC_KINEMATICS_IK_SANITIZE_IK_H_

#include <vector>

#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

// This header provides various utilities for tidying a list of candidate IK
// solutions.

namespace intrinsic {
namespace kinematics {
namespace sanitize_ik {

// Wraps each IK solution in `solutions` to within joint limits, by adding a
// (positive or negative) multiple of TWO_PI to each joint value.  Solutions
// that cannot be wrapped within the limits are dropped.
//
// The returned solutions will have joint values within the range
// `[min_limit[i], min_limit[i] + 2_PI)` (for each joint `i`). This wrapping
// applies to all joint values, including those that are already within the
// joint limits.
//
// This takes `O(N)` time (where `N == solutions.size()`).
//
// This function returns the number of valid solutions. Valid solutions will be
// placed at the beginning of the solutions Span. The rest of the values in the
// Span are not valid solutions.
//
// The ordering of elements in `solutions` is not preserved.
// This is realtime safe.
int WrapToLimits(absl::Span<JointStateP> solutions, const JointLimits& limits);

// Grows a list of IK solutions by expanding all possible windings. For each
// joint value, multiples of TWO_PI are added (and subtracted) to produce new IK
// solutions within the joint limits. If multiple joints have multiple valid
// windings then all combinations are produced.  Note that a full expansion may
// have a very large (exponential) number of solutions. However, the actual
// number of returned solutions is limited to the size of the `solutions` span.
//
// The first `num_provided_solutions` elements in `solutions` remain unchanged.
// Additional solutions are placed after the initial `num_provided_solutions`
// elements in `solutions`, up to at most the size of the span.
//
// If a provided solution is outside the joint limits by more than TWO_PI, no
// additional expansions will be produced from that solution. For this reason,
// we recommend calling `WrapToLimits()` prior to calling this.
// This is realtime safe.
icon::RealtimeStatusOr<int> ExpandWindings(absl::Span<JointStateP> solutions,
                                           const JointLimits& limits,
                                           int num_provided_solutions);

}  // namespace sanitize_ik
}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SANITIZE_IK_H_
