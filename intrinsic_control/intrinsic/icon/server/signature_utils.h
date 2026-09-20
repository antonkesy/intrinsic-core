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

#ifndef INTRINSIC_ICON_SERVER_SIGNATURE_UTILS_H_
#define INTRINSIC_ICON_SERVER_SIGNATURE_UTILS_H_

#include <string>

#include "absl/container/btree_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"

namespace intrinsic::icon {

struct SlotPartCompatibility {
  // True if the part is compatible. See `PartCompatibleWithSlot` for details.
  bool Compatible() const {
    return missing_required_interfaces.empty() &&
           missing_optional_interfaces.empty();
  }
  // String explanation why the part is not compatible to the `PartSlotInfo`.
  // Explains which required or optional interfaces are missing from part.
  std::string Explain() const {
    if (missing_required_interfaces.empty() &&
        !missing_optional_interfaces.empty()) {
      return absl::StrCat(
          "Part '", part_name,
          "' must support at least one of these optional interfaces [",
          absl::StrJoin(missing_optional_interfaces, ", "), "].");
    }
    if (!missing_required_interfaces.empty()) {
      return absl::StrCat(
          "Part '", part_name, "' is missing these required interfaces [",
          absl::StrJoin(missing_required_interfaces, ", "), "].");
    }
    return absl::StrCat("Part '", part_name, "' is compatible.");
  }
  std::string part_name = "";
  // Part needs to support all of the required interfaces.
  absl::flat_hash_set<std::string> missing_required_interfaces = {};
  // Part needs to support at least one of the optional interfaces.
  absl::flat_hash_set<std::string> missing_optional_interfaces = {};
};

// Returns SlotPartCompatibility
// SlotPartCompatibility::Compatible is `true` if:
// * `slot_info` lists any required Feature Interfaces, and `part_config` has
//   *all* of them
// * `slot_info` lists only optional Feature Interfaces, and `part_config` has
//   *any* of them
// * `slot_info` lists neither required not optional Feature Interfaces
//
// SlotPartCompatibility::Explain() contains an explanation if the part is
// not compatible to a slot.
//
// Not realtime safe.
SlotPartCompatibility PartCompatibleWithSlot(
    const intrinsic_proto::icon::v1::PartConfig& part_config,
    const intrinsic_proto::icon::v1::ActionSignature::PartSlotInfo& slot_info)
    INTRINSIC_NON_REALTIME_ONLY;

// For each mapping in `slot_part_map`:
// * Checks if the slot name is one that `action_signature` supports
// * Looks up the configuration proto for the corresponding part in
//   `part_configs``
// * Checks if the part is compatible with the slot
//
// It's legal to map a part to multiple slots, provided that the feature
// interfaces for those slots do not overlap.
// Example:
// part_slot_infos {
//   key: "slot_1"
//   value {
//     required_feature_interfaces: FEATURE_INTERFACE_JOINT_POSITION
//   }
// }
// part_slot_infos {
//   key: "slot_2"
//   value {
//     optional_feature_interfaces: FEATURE_INTERFACE_JOINT_POSITION
//   }
// }
// part_slot_infos {
//   key: "slot_3"
//   value {
//     required_feature_interfaces: FEATURE_INTERFACE_SIMPLE_GRIPPER
//   }
// }
//
// Given the above signature, and a Part `arm_with_gripper` that supports both
// FEATURE_INTERFACE_JOINT_POSITION and FEATURE_INTERFACE_SIMPLE_GRIPPER:
//
// * {slot_1: arm_with_gripper, slot_2: arm_with_gripper, slot_3: other_part}
//   Is *invalid*. both slot_1 and slot_2 use FEATURE_INTERFACE_JOINT_POSITION.
// * {slot_1: arm_with_gripper, slot_2: other_part, slot_3: arm_with_gripper}
//   Is *valid*. None of `arm_with_gripper`'s interfaces are used more than
//   once.
absl::Status ActionCompatibleWithSlotPartMap(
    const absl::btree_map<std::string, std::string>& slot_part_map,
    const intrinsic_proto::icon::v1::ActionSignature& action_signature,
    absl::Span<const intrinsic_proto::icon::v1::PartConfig> part_configs)
    INTRINSIC_NON_REALTIME_ONLY;

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_SERVER_SIGNATURE_UTILS_H_
