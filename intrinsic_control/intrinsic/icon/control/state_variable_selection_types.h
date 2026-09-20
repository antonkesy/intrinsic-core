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

#ifndef INTRINSIC_ICON_CONTROL_STATE_VARIABLE_SELECTION_TYPES_H_
#define INTRINSIC_ICON_CONTROL_STATE_VARIABLE_SELECTION_TYPES_H_

#include <cstdint>
#include <functional>
#include <variant>

#include "intrinsic/icon/control/realtime_bridge_types.h"

namespace intrinsic::icon {

// A variant that can take any value that a state variable selection function
// can return.
using PartStatusVariant = std::variant<bool, double, int64_t>;

struct StateVariableFieldSelectionData {
  const AggregatedRobotStatus& robot_status;
};

// A function object used to select a part status value from the PublishOutput.
using StateVariableFieldSelectionFunction =
    std::function<RealtimeStatusOr<PartStatusVariant>(
        const StateVariableFieldSelectionData&)>;

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_STATE_VARIABLE_SELECTION_TYPES_H_
