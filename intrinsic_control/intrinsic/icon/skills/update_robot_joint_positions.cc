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

#include "intrinsic/icon/skills/update_robot_joint_positions.h"

#include <memory>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "google/protobuf/message.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/cc_client/client.h"
#include "intrinsic/icon/equipment/channel_factory.h"
#include "intrinsic/icon/equipment/equipment_utils.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/skills/update_robot_joint_positions.pb.h"
#include "intrinsic/icon/utils/arm_utils.h"
#include "intrinsic/skills/cc/equipment_pack.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/equipment.pb.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/skills/proto/skills.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/kinematic_object.h"
#include "intrinsic/world/objects/object_world_client.h"

namespace intrinsic {
namespace skills {

std::unique_ptr<SkillInterface> UpdateRobotJointPositions::CreateSkill() {
  return std::make_unique<UpdateRobotJointPositions>(
      std::make_unique<icon::DefaultChannelFactory>());
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
UpdateRobotJointPositions::Execute(const ExecuteRequest& request,
                                   ExecuteContext& context) {
  INTR_ASSIGN_OR_RETURN(
      auto params,
      request.params<
          intrinsic_proto::skills::UpdateRobotJointPositionsParameters>());

  world::ObjectWorldClient& world = context.object_world();

  EquipmentPack equipment_pack = context.equipment();

  INTR_ASSIGN_OR_RETURN(auto equipment,
                        equipment_pack.GetHandle(kEquipmentSlot));
  LOG(INFO) << "Using equipment: " << equipment.name() << ".";

  INTR_ASSIGN_OR_RETURN(world::KinematicObject kinematic_object,
                        world.GetKinematicObject(equipment));

  INTR_ASSIGN_OR_RETURN(
      icon::IconEquipment icon_equipment,
      icon::ConnectToIconEquipment(equipment_pack, kEquipmentSlot,
                                   *icon_channel_factory_));

  // Read the joint configuration.
  icon::Client icon_client(icon_equipment.channel);
  INTR_ASSIGN_OR_RETURN(intrinsic_proto::icon::PartStatus part_status,
                        icon_client.GetSinglePartStatus(params.part()));
  const eigenmath::VectorXd joint_values =
      icon::GetSensedJointPosition(part_status);

  eigenmath::VectorXd world_joint_values = kinematic_object.JointPositions();
  if (!world_joint_values.isApprox(joint_values)) {
    LOG(WARNING) << "Joint values for '" << kinematic_object.Name()
                 << "' in world " << world_joint_values
                 << " is different from actual " << joint_values;
  }

  INTR_RETURN_IF_ERROR(
      world.UpdateJointPositions(kinematic_object, joint_values));

  return nullptr;
}

}  // namespace skills
}  // namespace intrinsic
