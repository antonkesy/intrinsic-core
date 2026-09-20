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

#include "intrinsic/icon/skills/enable_realtime_control.h"

#include <memory>

#include "absl/base/attributes.h"
#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/message.h"
#include "intrinsic/icon/cc_client/client.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/equipment/channel_factory.h"
#include "intrinsic/icon/equipment/equipment_utils.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/skills/enable_realtime_control.pb.h"
#include "intrinsic/icon/utils/arm_utils.h"
#include "intrinsic/resources/proto/resource_handle.pb.h"
#include "intrinsic/skills/cc/equipment_pack.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/internal/kinematic_object_for_position_part.h"
#include "intrinsic/skills/proto/equipment.pb.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/skills/proto/skills.pb.h"
#include "intrinsic/stats/scoped_span.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/kinematic_object.h"
#include "intrinsic/world/objects/object_world_client.h"

namespace intrinsic {
namespace skills {

// static
std::unique_ptr<SkillInterface> EnableRealtimeControl::CreateSkill() {
  return std::make_unique<EnableRealtimeControl>(
      std::make_unique<icon::DefaultChannelFactory>());
}

absl::StatusOr<intrinsic_proto::skills::Footprint>
EnableRealtimeControl::GetFootprint(const GetFootprintRequest& request,
                                    GetFootprintContext& context) const {
  const stats::ScopedSpan span("skills.EnableRealtimeControl/GetFootprint");
  // Enabling motion has no outward-facing behaviors, so the footprint stays
  // empty..
  return intrinsic_proto::skills::Footprint();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
EnableRealtimeControl::Execute(const ExecuteRequest& request,
                               ExecuteContext& context) {
  const stats::ScopedSpan span("skills.EnableRealtimeControl/Execute");
  INTR_ASSIGN_OR_RETURN(
      auto params,
      request.params<intrinsic_proto::skills::EnableRealtimeControlParams>());

  // const google::protobuf::Map<std::string,
  // intrinsic_proto::resources::ResourceHandle>& resource_handles,
  const EquipmentPack equipment_pack = context.equipment();

  INTR_ASSIGN_OR_RETURN(
      const intrinsic_proto::resources::ResourceHandle equipment,
      equipment_pack.GetHandle(kEquipmentSlot), _.LogError());

  LOG(INFO) << "Executing EnableRealtimeControl with [ clear_faults = "
            << params.clear_faults() << "] on " << equipment.name() << ".";

  INTR_ASSIGN_OR_RETURN(
      icon::IconEquipment icon_equipment,
      icon::ConnectToIconEquipment(equipment_pack, kEquipmentSlot,
                                   *icon_channel_factory_),
      _.LogError());

  intrinsic::icon::Client icon_client(icon_equipment.channel);
  INTR_ASSIGN_OR_RETURN(auto icon_status, icon_client.GetOperationalStatus(),
                        _.LogError());
  LOG(INFO) << "ICON server is " << icon_status << ".";
  switch (icon_status.state()) {
    case icon::OperationalState::kEnabled:
      break;
    case icon::OperationalState::kFaulted:
      if (params.clear_faults()) {
        LOG(INFO) << "Clearing faults.";
        INTR_RETURN_IF_ERROR(icon_client.ClearFaults()).LogError();
        LOG(INFO) << "Faults cleared.";
      }
      ABSL_FALLTHROUGH_INTENDED;
    case icon::OperationalState::kDisabled:
      LOG(INFO) << "Enabling " << equipment.name();
      INTR_RETURN_IF_ERROR(icon_client.Enable()).LogError();
      LOG(INFO) << "Enabling done";
      break;
    default:
      return intrinsic::InternalErrorBuilder().LogError()
             << equipment.name()
             << "is in state: " << absl::StrCat(icon_status.state());
  }

  if (icon_equipment.position_part_name.has_value()) {
    INTR_ASSIGN_OR_RETURN(
        intrinsic_proto::icon::PartStatus part_status,
        icon_client.GetSinglePartStatus(*icon_equipment.position_part_name));
    world::ObjectWorldClient& world = context.object_world();
    INTR_ASSIGN_OR_RETURN(
        world::KinematicObject robot_object,
        KinematicObjectForPositionPart(kEquipmentSlot, equipment_pack, world));
    INTR_RETURN_IF_ERROR(world.UpdateJointPositions(
        robot_object, icon::GetSensedJointPosition(part_status)));
  }

  return nullptr;
}

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>>
EnableRealtimeControl::Preview(const PreviewRequest& request,
                               PreviewContext& context) {
  return nullptr;
}

}  // namespace skills
}  // namespace intrinsic
