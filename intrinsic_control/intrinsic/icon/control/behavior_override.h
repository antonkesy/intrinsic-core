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

#ifndef INTRINSIC_ICON_CONTROL_BEHAVIOR_OVERRIDE_H_
#define INTRINSIC_ICON_CONTROL_BEHAVIOR_OVERRIDE_H_

#include <string_view>

#include "absl/container/fixed_array.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"

namespace intrinsic::icon {

// The list of `BehaviorOverrideRequest`s that a safety_action needs to support.
// ICON won't start otherwise.
// Behavior support is checked for all actions.
// The override is set for safety actions in
// intrinsic_control/intrinsic/icon/control/realtime_part_manager.cc.
std::vector<intrinsic_proto::icon::v1::BehaviorOverrideRequest>
GetRequiredSafetyActionBehaviorOverrides() INTRINSIC_NON_REALTIME_ONLY;

// Creates a dense array of boolean values to check supported
// BEHAVIOR_OVERRIDE_REQUESTs using the provided `action_signature`.
//
//
// True at the index of a BehaviorOverrideRequest means the action supports this
// override.
// The convenience method `ActionSupportsRequestedOverrideBehavior` includes
// bounds checks.
absl::StatusOr<absl::FixedArray<bool>> MakeBehaviorOverrideSupportArray(
    const intrinsic_proto::icon::v1::ActionSignature& action_signature)
    INTRINSIC_NON_REALTIME_ONLY;

// Returns `true` when `behavior_override_request` is marked as supported in
// the dense array `behavior_override_support_list`.
// Returns `false` if the behavior is not supported.
// Returns false if the enum value of `behavior_override_request` is not within
// the bounds of `behavior_override_support_list`. This indicates a bug.
bool ActionSupportsRequestedOverrideBehavior(
    absl::Span<const bool> behavior_override_support_list,
    intrinsic_proto::icon::v1::BehaviorOverrideRequest
        behavior_override_request) INTRINSIC_CHECK_REALTIME_SAFE;

// Returns a realtime-safe string representation of the given
// `behavior_override_request`.
std::string_view BehaviorOverrideRequestToRealtimeSafeString(
    intrinsic_proto::icon::v1::BehaviorOverrideRequest
        behavior_override_request) INTRINSIC_CHECK_REALTIME_SAFE;

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_BEHAVIOR_OVERRIDE_H_
