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

#ifndef INTRINSIC_ICON_EQUIPMENT_TESTING_EQUIPMENT_TEST_UTILS_H_
#define INTRINSIC_ICON_EQUIPMENT_TESTING_EQUIPMENT_TEST_UTILS_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/equipment/force_control_settings.pb.h"
#include "intrinsic/icon/equipment/icon_equipment.pb.h"
#include "intrinsic/resources/proto/resource_handle.pb.h"
#include "intrinsic/skills/proto/equipment.pb.h"
#include "intrinsic/skills/proto/skills.pb.h"

namespace intrinsic {
namespace icon {

class Icon2EquipmentHandleBuilder {
  struct PositionPart {
    std::string part_name;
    std::string hardware_object_name;
  };

 public:
  // Creates a resource handle builder for an intrinsic_proto.icon.v1.IconApi
  // equipment with handle name `robot_id` and with target gRPC address at
  // `target`.
  Icon2EquipmentHandleBuilder(absl::string_view robot_id,
                              absl::string_view target);

  // Creates the resource handle.
  intrinsic_proto::resources::ResourceHandle Build() const;

  // Returns an equipment map containing a single entry with the given slot name
  // 'equipment_slot' and the resource handle represented by this builder.
  absl::flat_hash_map<std::string, intrinsic_proto::resources::ResourceHandle>
  BuildMapForEquipmentSlot(absl::string_view equipment_slot) const;

  // Adds a Icon2PositionPart with `part_name`.
  Icon2EquipmentHandleBuilder& WithPositionControlledPart(
      absl::string_view part_name, absl::string_view hardware_object_name = "");

  // Adds a Icon2GripperPart with `part_name`.
  Icon2EquipmentHandleBuilder& WithGripperPart(absl::string_view part_name);

  // Adds a Icon2AdioPart with `part_name`.
  Icon2EquipmentHandleBuilder& WithAdioPart(absl::string_view part_name);

  // Adds a Icon2ForceTorqueSensorPart with `part_name`.
  Icon2EquipmentHandleBuilder& WithForceTorqueSensorPart(
      absl::string_view part_name,
      const std::optional<intrinsic_proto::icon::ForceControlSettings>&
          force_control_settings = std::nullopt);

  // Adds a Icon2RangefinderPart with `part_name`.
  Icon2EquipmentHandleBuilder& WithRangefinderPart(absl::string_view part_name);

 private:
  const std::string robot_id_;
  const std::string target_;
  // Position and ADIO support multiple parts per instance.
  std::vector<PositionPart> position_parts_;
  std::string gripper_part_;
  std::vector<std::string> adio_parts_;
  std::optional<intrinsic_proto::icon::Icon2ForceTorqueSensorPart>
      force_torque_sensor_part_;
  std::string rangefinder_part_;
};

class Icon2SkillInstanceBuilder {
 public:
  // Creates a skill instance builder for `skill_id` containing a single
  // intrinsic_proto.icon.v1.IconApi equipment with a target gRPC address at
  // `target` in the specified `equipment_slot` with handle name `robot_id`.
  Icon2SkillInstanceBuilder(absl::string_view skill_id,
                            absl::string_view target,
                            absl::string_view equipment_slot,
                            absl::string_view robot_id);
  // Creates the skill instance.
  intrinsic_proto::skills::SkillInstance Build() const;

  // Adds a Icon2PositionPart with `part_name`.
  Icon2SkillInstanceBuilder& WithPositionControlledPart(
      absl::string_view part_name, absl::string_view hardware_object_name = "");
  // Adds a Icon2GripperPart with `part_name`.
  Icon2SkillInstanceBuilder& WithGripperPart(absl::string_view part_name);

  // Adds a Icon2AdioPart with `part_name`.
  Icon2SkillInstanceBuilder& WithAdioPart(absl::string_view part_name);
  // Adds a Icon2ForceTorqueSensorPart with `part_name`.
  Icon2SkillInstanceBuilder& WithForceTorqueSensorPart(
      absl::string_view part_name,
      const std::optional<intrinsic_proto::icon::ForceControlSettings>&
          force_control_settings = std::nullopt);

  // Adds a Icon2RangefinderPart with `part_name`.
  Icon2SkillInstanceBuilder& WithRangefinderPart(absl::string_view part_name);

  Icon2SkillInstanceBuilder() = delete;

 private:
  const std::string skill_id_;
  const std::string equipment_slot_;
  Icon2EquipmentHandleBuilder handle_builder_;
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_EQUIPMENT_TESTING_EQUIPMENT_TEST_UTILS_H_
