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

#include "intrinsic/world/util/remove_redundant_updates.h"

#include <utility>

#include "absl/status/statusor.h"
#include "intrinsic/kinematics/types/joint_limits.pb.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"

namespace intrinsic {

absl::StatusOr<intrinsic_proto::world::ObjectWorldUpdates>
RemoveRedundantUpdates(
    intrinsic_proto::world::ObjectWorldUpdates input_updates) {
  intrinsic_proto::world::ObjectWorldUpdates result;
  for (auto& update : *input_updates.mutable_updates()) {
    if (result.updates().empty()) {
      *result.add_updates() = std::move(update);
      continue;
    }

    auto& last_update = *result.updates().rbegin();
    if (!last_update.has_update_object_joints()) {
      *result.add_updates() = std::move(update);
      continue;
    }

    if (object_world::AreTheSame(last_update.update_object_joints().object(),
                                 update.update_object_joints().object())) {
      // If the old update had joint system limit updates but the new one
      // doesn't then we can carry those forward.
      if (last_update.update_object_joints().has_joint_system_limits() &&
          !update.update_object_joints().has_joint_system_limits()) {
        *update.mutable_update_object_joints()->mutable_joint_system_limits() =
            last_update.update_object_joints().joint_system_limits();
      }

      // If the old update had joint application limit updates but the new one
      // doesn't then we can carry those forward.
      if (last_update.update_object_joints().has_joint_application_limits() &&
          !update.update_object_joints().has_joint_application_limits()) {
        *update.mutable_update_object_joints()
             ->mutable_joint_application_limits() =
            last_update.update_object_joints().joint_application_limits();
      }

      // If the old update had joint positions but the new one doesn't then
      // we can carry those forward.
      if (last_update.update_object_joints().joint_positions_size() != 0 &&
          update.update_object_joints().joint_positions_size() == 0) {
        *update.mutable_update_object_joints()->mutable_joint_positions() =
            last_update.update_object_joints().joint_positions();
      }

      // Remove the previous update since it is equivalent to the new one.
      result.mutable_updates()->RemoveLast();
    }

    *result.add_updates() = std::move(update);
  }

  return result;
}

}  // namespace intrinsic
