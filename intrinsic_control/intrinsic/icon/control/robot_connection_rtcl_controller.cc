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

#include "intrinsic/icon/control/robot_connection_rtcl_controller.h"

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/common/slot_part_map.h"
#include "intrinsic/icon/control/initialized_async_buffer.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/parts/realtime_part_status.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/rtcl_action_factory_registry.h"
#include "intrinsic/icon/control/rtcl_controller_bridge_interface.h"
#include "intrinsic/icon/control/rtcl_controller_session.h"
#include "intrinsic/icon/control/rtcl_session_bridge_interface.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/proto/safety_status.pb.h"
#include "intrinsic/icon/proto/safety_status_conversion.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/operational_state_interface.h"
#include "intrinsic/icon/server/part_collection.h"
#include "intrinsic/icon/server/session_interface.h"
#include "intrinsic/icon/server/signature_utils.h"
#include "intrinsic/logging/proto/context.pb.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {
namespace {

class RtclStatusGetter final : public PartCollection::StatusGetter {
 public:
  explicit RtclStatusGetter(
      RtclControllerBridgeInterface& rtcl_controller_bridge)
      : rtcl_controller_bridge_(rtcl_controller_bridge) {}

  absl::StatusOr<
      absl::flat_hash_map<std::string, intrinsic_proto::icon::PartStatus>>
  GetPartStatuses(absl::Time deadline) override {
    INTR_ASSIGN_OR_RETURN(RealtimePartStatuses realtime_statuses,
                          rtcl_controller_bridge_.GetPartStatuses(deadline));
    absl::flat_hash_map<std::string, intrinsic_proto::icon::PartStatus>
        proto_part_status_map;
    for (const auto& [part_name, realtime_status] :
         realtime_statuses.status_by_part_name) {
      proto_part_status_map.insert({part_name, ToProto(realtime_status)});
    }
    return proto_part_status_map;
  }

  absl::StatusOr<intrinsic_proto::icon::SafetyStatus> GetSafetyStatus(
      absl::Time deadline) override {
    INTR_ASSIGN_OR_RETURN(const RealtimeSafetyStatus rt_safety_status,
                          rtcl_controller_bridge_.GetSafetyStatus(deadline));
    return ToProto(rt_safety_status.safety_status);
  }

  absl::StatusOr<TimestampedPartProperties> GetPartProperties() override {
    return rtcl_controller_bridge_.GetPartProperties();
  }

  absl::Status SetPartProperties(const PartPropertyMap& properties) override {
    return rtcl_controller_bridge_.SetPartProperties(properties);
  }

 private:
  RtclControllerBridgeInterface& rtcl_controller_bridge_;
};

}  // namespace

RobotConnectionRtclController::RobotConnectionRtclController(
    const intrinsic_proto::icon::v1::ServerConfig& server_config,
    const absl::flat_hash_map<std::string, size_t>& part_name_to_realtime_index,
    PartCollection part_collection,
    RtclControllerBridgeInterface& rtcl_controller_bridge,
    OperationalStateInterface& operational_state,
    InitializedAsyncBuffer<PublishOutput>& robot_status_buffer,
    InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer)
    : server_config_(server_config),
      action_signature_map_(
          GetGlobalRtclActionFactoryRegistry().GetAllSignatures()),
      part_name_to_realtime_index_(part_name_to_realtime_index),
      part_collection_(std::move(part_collection)),
      rtcl_controller_bridge_(rtcl_controller_bridge),
      operational_state_(operational_state),
      robot_status_buffer_(robot_status_buffer),
      logging_mode_buffer_(logging_mode_buffer) {}

// static
absl::StatusOr<std::unique_ptr<RobotConnectionRtclController>>
RobotConnectionRtclController::Create(
    const intrinsic_proto::icon::v1::ServerConfig& server_config,
    std::vector<intrinsic_proto::icon::v1::PartConfig>
        part_configs_in_index_order,
    RtclControllerBridgeInterface& rtcl_controller_bridge,
    OperationalStateInterface& operational_state,
    InitializedAsyncBuffer<PublishOutput>& robot_status_buffer,
    InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer) {
  absl::flat_hash_map<std::string, size_t> part_name_to_realtime_index;
  for (size_t i = 0; i < part_configs_in_index_order.size(); ++i) {
    part_name_to_realtime_index[part_configs_in_index_order[i].name()] = i;
  }
  INTR_ASSIGN_OR_RETURN(
      auto part_collection,
      PartCollection::Create(
          std::move(part_configs_in_index_order),
          std::make_unique<RtclStatusGetter>(rtcl_controller_bridge)));
  return absl::WrapUnique(new RobotConnectionRtclController(
      server_config, part_name_to_realtime_index, std::move(part_collection),
      rtcl_controller_bridge, operational_state, robot_status_buffer,
      logging_mode_buffer));
}

absl::StatusOr<std::unique_ptr<SessionInterface>>
RobotConnectionRtclController::CreateSession(
    const absl::flat_hash_set<std::string>& part_names, SessionId session_id,
    const RealtimeLogContext& log_context) {
  std::vector<intrinsic_proto::icon::v1::PartConfig> part_config_vector =
      part_collection_.GetPartConfigs();
  absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::PartConfig>
      part_config_map;
  absl::flat_hash_map<std::string, intrinsic_proto::data_logger::Context>
      part_log_context_map;
  for (const auto& part_config : part_config_vector) {
    if (!part_names.contains(part_config.name())) {
      continue;
    }
    if (bool inserted =
            part_config_map.try_emplace(part_config.name(), part_config).second;
        !inserted) {
      return absl::InternalError(
          absl::StrCat("Duplicate part name '", part_config.name(), "'"));
    }
  }

  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<RtclSessionBridgeInterface> session_bridge,
      rtcl_controller_bridge_.StartSession(session_id, part_names,
                                           log_context));

  return std::make_unique<RtclControllerSession>(
      server_config_, part_name_to_realtime_index_, session_id,
      std::move(part_config_map), std::move(session_bridge), log_context,
      robot_status_buffer_, logging_mode_buffer_);
}

OperationalStateInterface&
RobotConnectionRtclController::MutableOperationalStateInterface() {
  return operational_state_;
}

absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::ActionSignature>
RobotConnectionRtclController::ActionTypeToSignature() const {
  return GetGlobalRtclActionFactoryRegistry().GetAllSignatures();
}

PartCollection& RobotConnectionRtclController::MutablePartCollection() {
  return part_collection_;
}

const PartCollection& RobotConnectionRtclController::GetPartCollection() const {
  return part_collection_;
}

const intrinsic_proto::icon::v1::ServerConfig&
RobotConnectionRtclController::config() const {
  return server_config_;
}

absl::Status RobotConnectionRtclController::ActionCompatibleWithSlotPartMap(
    absl::string_view action_type_name,
    const SlotPartMap& slot_part_map) const {
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::icon::v1::ActionSignature action_signature,
      GetGlobalRtclActionFactoryRegistry().GetSignature(action_type_name));
  return ::intrinsic::icon::ActionCompatibleWithSlotPartMap(
      slot_part_map, action_signature, part_collection_.GetPartConfigs());
}

absl::Status RobotConnectionRtclController::ActionCompatibleWithPart(
    absl::string_view action_type_name, absl::string_view part_name) const {
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::icon::v1::ActionSignature action_signature,
      GetGlobalRtclActionFactoryRegistry().GetSignature(action_type_name));
  const auto part_configs = part_collection_.GetPartConfigs();
  auto part_config = absl::c_find_if(
      part_configs,
      [part_name](const intrinsic_proto::icon::v1::PartConfig& part_config) {
        return part_config.name() == part_name;
      });
  if (part_config == part_configs.end()) {
    return absl::NotFoundError(
        absl::StrCat("Could not find config for Part '", part_name, "'"));
  }

  // Try to find (at least) one Slot for which the Part in question has *all*
  // required FeatureInterfaces.
  auto first_compatible_slot =
      absl::c_find_if(action_signature.part_slot_infos(),
                      [&part_config](const auto& slot_name_and_slot_info) {
                        return PartCompatibleWithSlot(
                                   *part_config, slot_name_and_slot_info.second)
                            .Compatible();
                      });

  if (first_compatible_slot == action_signature.part_slot_infos().end()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Part '", part_name,
        "' is not compatible with any of the Slots for Action type '",
        action_type_name, "'. Available Slots: [",
        absl::StrJoin(action_signature.part_slot_infos(), ", ",
                      [](std::string* str, const auto& slot_name_and_info) {
                        absl::StrAppend(str, slot_name_and_info.first);
                      }),
        "]"));
  }
  return absl::OkStatus();
}

void RobotConnectionRtclController::SetSpeedOverride(
    double new_speed_override) {
  rtcl_controller_bridge_.SetSpeedOverride(new_speed_override);
}

double RobotConnectionRtclController::GetSpeedOverride() const {
  return rtcl_controller_bridge_.GetSpeedOverride();
}

void RobotConnectionRtclController::SetLoggingMode(LoggingMode logging_mode) {
  return rtcl_controller_bridge_.SetLoggingMode(logging_mode);
}

LoggingMode RobotConnectionRtclController::GetLoggingMode() const {
  return rtcl_controller_bridge_.GetLoggingMode();
}

absl::StatusOr<absl::flat_hash_map<std::string, OperationalState>>
RobotConnectionRtclController::GetPartStates() {
  return rtcl_controller_bridge_.GetPartStates();
}

}  // namespace intrinsic::icon
