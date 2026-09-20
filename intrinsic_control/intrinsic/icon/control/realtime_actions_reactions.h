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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_ACTIONS_REACTIONS_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_ACTIONS_REACTIONS_H_

#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_condition.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// While a double can be larger than int64, it cannot represent all values
// to the last digit. This value is 2^53, due to the mantissa of 52 of a
// IEEE-754 double.
constexpr double kMaxValidInt64AsDouble = 9007199254740992;

// Uses the given data in `comparison` to compare the `comparison.operand` with
// `comparison.value` using `comparison.operation`.
//
// If `comparison.operand` contains a std::function, uses `robot_status` to
// extract the requested part status field.

// If `comparison.operand` contains a string, queries `action` for the
// corresponding state variable value.

// In both cases, returns the final boolean output of `comparison`.
//
// Returns various error messages when encountering problems with part status
// field conditions.
// Returns kNotFound if the action does not know either of the involved state
// variables.
RealtimeStatusOr<bool> LookupAndEvaluate(
    const RtclActionInterface& action, const RealtimeComparison& comparison,
    const AggregatedRobotStatus& robot_status);

// Traverses the passed conditions and evaluates the embedded Comparisons.
// Calls the `Comparison` overload of `LookupAndEvaluate` for each leaf
// comparison in `condition`. See the comment on that overload for details.
RealtimeStatusOr<bool> LookupAndEvaluate(
    const RtclActionInterface& action, const RealtimeCondition& condition,
    const AggregatedRobotStatus& robot_status);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_ACTIONS_REACTIONS_H_
