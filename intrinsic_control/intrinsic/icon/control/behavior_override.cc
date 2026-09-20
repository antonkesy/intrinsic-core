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

#include "intrinsic/icon/control/behavior_override.h"

#include "absl/container/fixed_array.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/log.h"

namespace intrinsic::icon {

using ::intrinsic_proto::icon::v1::BehaviorOverrideRequest;

absl::StatusOr<absl::FixedArray<bool>> MakeBehaviorOverrideSupportArray(
    const intrinsic_proto::icon::v1::ActionSignature& action_signature) {
  using BehaviorOverrideType = std::underlying_type_t<BehaviorOverrideRequest>;

  // Needs to be adjusted when a new BehaviorOverrideRequest is added to the
  // enum.
  const auto kMaxEnumValue = static_cast<BehaviorOverrideType>(
      BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_PAUSE);

  // Zero-initialize vector of correct size.
  // kMaxEnumValue + 1 so that the enum value can be used as index.
  std::vector<bool> support_list(kMaxEnumValue + 1, 0);
  for (const auto& info : action_signature.behavior_override_infos()) {
    const auto cap = info.override_request();
    const auto enum_value = static_cast<BehaviorOverrideType>(cap);
    if (enum_value > kMaxEnumValue) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Unsupported behavior override '",
          intrinsic_proto::icon::v1::BehaviorOverrideRequest_Name(cap),
          "' with value '", enum_value, "'."));
    }
    support_list[enum_value] = true;
  }
  // Explicitly mark BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN as supported.
  // Every action "supports" no behavior override.
  support_list[static_cast<BehaviorOverrideType>(
      BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN)] = true;

  return absl::FixedArray<bool>{support_list.begin(), support_list.end()};
}

bool ActionSupportsRequestedOverrideBehavior(
    absl::Span<const bool> behavior_override_support_list,
    BehaviorOverrideRequest behavior_override_request) {
  if (behavior_override_request ==
      BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN) {
    // BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN is by definition supported by all
    // actions.
    return true;
  }

  const auto enum_value =
      static_cast<std::underlying_type_t<BehaviorOverrideRequest>>(
          behavior_override_request);

  // Return false if the requested override is out of bounds.
  // Indicates a serious issue.
  if (enum_value < 0 || enum_value >= behavior_override_support_list.size())
      [[unlikely]] {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "'"
        << BehaviorOverrideRequestToRealtimeSafeString(
               behavior_override_request)
        << "' is out of bounds. Please file a bug.";
    return false;
  }

  return behavior_override_support_list[enum_value];
}

std::vector<intrinsic_proto::icon::v1::BehaviorOverrideRequest>
GetRequiredSafetyActionBehaviorOverrides() {
  // BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN is included for completeness, even though
  // it's by definition supported by all actions.
  return {BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN,
          BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_PAUSE};
}

std::string_view BehaviorOverrideRequestToRealtimeSafeString(
    BehaviorOverrideRequest behavior_override_request) {
  switch (behavior_override_request) {
    case BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN:
      return "BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN";
    case BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_PAUSE:
      return "BEHAVIOR_OVERRIDE_REQUEST_PAUSE";
    default:
      return "INVALID_BEHAVIOR_OVERRIDE_REQUEST";
  }
}

}  // namespace intrinsic::icon
