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

#include "intrinsic/world/util/ros_joint_state_util.h"

#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/util/proto_time.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"

namespace intrinsic {
namespace object_world {

absl::Status SetJointPositionsFromRos(
    KinematicObject& object,
    const ::sensor_msgs::msg::pb::jazzy::JointState& joint_state) {
  if (joint_state.name_size() == 0) {
    return absl::InvalidArgumentError("ROS JointState message has no names.");
  }

  if (joint_state.name_size() != joint_state.position_size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "ROS JointState name (", joint_state.name_size(), ") and position (",
        joint_state.position_size(), ") sizes mismatch."));
  }

  const auto& entity_world = object.GetEntityWorld();
  INTR_ASSIGN_OR_RETURN(const std::vector<JointEntityId> joint_ids,
                        object.GetJointEntityIds());

  absl::flat_hash_map<std::string, double> name_to_pos;
  name_to_pos.reserve(joint_state.name_size());
  for (int i = 0; i < joint_state.name_size(); ++i) {
    name_to_pos[joint_state.name(i)] = joint_state.position(i);
  }

  eigenmath::VectorXd joint_positions(joint_ids.size());
  for (size_t i = 0; i < joint_ids.size(); ++i) {
    const std::string joint_name =
        entity_world.GetLocalNameForEntityById(joint_ids[i]);
    auto it = name_to_pos.find(joint_name);

    if (it == name_to_pos.end()) {
      return absl::NotFoundError(absl::StrCat(
          "Joint '", joint_name, "' for object '", object.GetName().value(),
          "' not found in ROS JointState."));
    }
    joint_positions[i] = it->second;
  }

  INTR_ASSIGN_OR_RETURN(absl::Time stamp,
                        ToAbslTime(joint_state.header().stamp()));
  return object.SetJointPositions(joint_positions, stamp,
                                  /*enforce_monotonic_time=*/true);
}

}  // namespace object_world
}  // namespace intrinsic
