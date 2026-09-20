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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_PAYLOAD_PROPERTY_H_
#define INTRINSIC_ICON_CONTROL_PARTS_PAYLOAD_PROPERTY_H_

#include <array>
#include <optional>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/control/parts/part_property_registry.h"
#include "intrinsic/icon/control/parts/realtime_part_property_access.h"
#include "intrinsic/icon/control/parts/realtime_robot_payload.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/world/robot_payload/robot_payload.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::icon {

constexpr int kNumPayloadProperties = 14;
constexpr std::array<absl::string_view, kNumPayloadProperties> kNameSuffixes = {
    // The payload mass in kg.
    "mass",
    // The transform between mass and payload center of gravity.
    "tip_t_cog_x",
    "tip_t_cog_y",
    "tip_t_cog_z",
    "tip_t_cog_qx",
    "tip_t_cog_qy",
    "tip_t_cog_qz",
    "tip_t_cog_qw",
    // The inertia matrix of the payload. Only the upper triangle is
    // stored because the matrix is symmetric.
    "inertia_xx",
    "inertia_yy",
    "inertia_zz",
    "inertia_xy",
    "inertia_xz",
    "inertia_yz",
};
constexpr absl::string_view kHasValueSuffix = "has_value";

// Register and access the payload properties in a part in real-time.
class PayloadProperty {
 public:
  PayloadProperty() = default;

  // Creates a new payload property and registers the fields as double part
  // properties. Stores the part property IDs which are used in Read and Write.
  static absl::StatusOr<PayloadProperty> Create(
      absl::string_view payload_name,
      const std::optional<RobotPayloadBase>& initial_payload,
      PartPropertyRegistry& registry);

  // Reads the payload from the part property access.
  RealtimeStatusOr<std::optional<RealtimeRobotPayload>> Read(
      const RealtimePartPropertyAccess& access) const;

  // Writes the payload to the part property access.
  RealtimeStatus Write(RealtimePartPropertyAccess& access,
                       const std::optional<RobotPayloadBase>& payload) const;

 private:
  using Ids = std::array<PartPropertyId, kNumPayloadProperties>;
  // Creates a new payload property using the given IDs. The IDs are ordered as
  // in kNameSuffixes.
  explicit PayloadProperty(const Ids& ids, PartPropertyId has_value_id);

  // Ids are ordered as in kNameSuffixes.
  Ids ids_;
  PartPropertyId has_value_id_;
};

absl::StatusOr<absl::flat_hash_map<std::string, PartPropertyValue>>
ToPartPropertyMap(const std::optional<RobotPayload>& payload,
                  absl::string_view name);

absl::StatusOr<std::optional<RobotPayload>> FromPartPropertyMap(
    const absl::flat_hash_map<std::string, PartPropertyValue>& property_map,
    absl::string_view name);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_PAYLOAD_PROPERTY_H_
