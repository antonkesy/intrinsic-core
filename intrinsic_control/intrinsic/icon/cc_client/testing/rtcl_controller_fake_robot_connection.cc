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

#include "intrinsic/icon/cc_client/testing/rtcl_controller_fake_robot_connection.h"

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/functional/bind_front.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/cc_client/testing/channel_fake.h"
#include "intrinsic/icon/cc_client/testing/rtcl_controller_fake_session.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/common/slot_part_map.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/rtcl_action_factory_registry.h"
#include "intrinsic/icon/proto/safety_status.pb.h"
#include "intrinsic/icon/proto/v1/service.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/part_collection.h"
#include "intrinsic/icon/server/session_interface.h"
#include "intrinsic/icon/server/signature_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {
namespace {

class ChannelFakeStatusGetter : public PartCollection::StatusGetter {
 public:
  // `mutex`, `current_robot_status` and `property_map` must all outlive the
  // StatusGetter! Assumes that access to `current_robot_status` is guarded by
  // `mutex`.
  ChannelFakeStatusGetter(
      absl::Mutex& mutex, PartPropertyMap& property_map,
      const intrinsic_proto::icon::v1::GetStatusResponse* current_robot_status,
      const int* current_time_ns,
      absl::flat_hash_set<std::string> config_part_names)
      : mutex_(mutex),
        property_map_(property_map),
        current_robot_status_(*current_robot_status),
        current_time_ns_(*current_time_ns),
        config_part_names_(std::move(config_part_names)) {}

  absl::StatusOr<
      absl::flat_hash_map<std::string, intrinsic_proto::icon::PartStatus>>
  GetPartStatuses(absl::Time deadline) override {
    absl::MutexLock l(mutex_);

    absl::flat_hash_map<std::string, intrinsic_proto::icon::PartStatus>
        status_map;
    for (const auto& [part_status_name, part_status] :
         current_robot_status_.part_status()) {
      status_map.insert({part_status_name, part_status});
    }
    // Add empty status messages for parts that aren't in the current
    // status.
    for (const auto& part_name : config_part_names_) {
      if (!status_map.contains(part_name)) {
        intrinsic_proto::icon::PartStatus dummy_status;
        dummy_status.set_timestamp_ns(current_time_ns_);
        status_map[part_name] = dummy_status;
      }
    }

    return status_map;
  }

  absl::StatusOr<intrinsic_proto::icon::SafetyStatus> GetSafetyStatus(
      absl::Time deadline) override {
    absl::MutexLock l(mutex_);
    if (current_robot_status_.has_safety_status()) {
      return current_robot_status_.safety_status();
    }
    // Returns an empty SafetyStatus (Values default to UNKNOWN) if none is
    // present in current_robot_status_.
    // This matches the output of a real robot with no safety device configured.
    return intrinsic_proto::icon::SafetyStatus();
  }

  absl::StatusOr<TimestampedPartProperties> GetPartProperties() override {
    absl::MutexLock l(mutex_);
    return TimestampedPartProperties{
        .timestamp_control = absl::Nanoseconds(current_time_ns_),
        .timestamp_wall = absl::FromUnixNanos(current_time_ns_),
        .properties = property_map_.properties,
    };
  }

  absl::Status SetPartProperties(const PartPropertyMap& properties) override {
    absl::MutexLock l(mutex_);
    for (const auto& [part_name, part_properties] : properties.properties) {
      auto property_indices_for_part = property_map_.properties.find(part_name);
      if (property_indices_for_part == property_map_.properties.end()) {
        return absl::NotFoundError(absl::StrCat(
            "Cannot set properties for unknown part '", part_name, "'"));
      }
      for (const auto& [property_name, new_value] : part_properties) {
        auto old_value_it =
            property_indices_for_part->second.find(property_name);
        if (old_value_it == property_indices_for_part->second.end()) {
          return absl::NotFoundError(
              absl::StrCat("Cannot set unknown property '", property_name,
                           "' for part '", part_name, "'"));
        }
        INTR_RETURN_IF_ERROR(
            std::visit(AssignPropertyValue{.property_name = property_name},
                       /*src=*/new_value, /*dst=*/old_value_it->second));
      }
    }
    return absl::OkStatus();
  }

 private:
  absl::Mutex& mutex_;
  PartPropertyMap& property_map_ ABSL_GUARDED_BY(mutex_);
  const intrinsic_proto::icon::v1::GetStatusResponse& current_robot_status_
      ABSL_GUARDED_BY(mutex_);
  const int& current_time_ns_ ABSL_GUARDED_BY(mutex_);
  const absl::flat_hash_set<std::string> config_part_names_;
};

intrinsic_proto::icon::v1::ServerConfig MakeServerConfig(
    absl::string_view server_name, double control_frequency_hz) {
  intrinsic_proto::icon::v1::ServerConfig config;
  config.set_name(server_name);
  config.set_frequency_hz(control_frequency_hz);
  return config;
}

absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::PartConfig>
MakePartConfigsByName(
    absl::Span<const intrinsic_proto::icon::v1::PartConfig> part_configs) {
  absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::PartConfig>
      part_configs_by_name;
  for (const auto& part_config : part_configs) {
    part_configs_by_name[part_config.name()] = part_config;
  }
  return part_configs_by_name;
}

absl::flat_hash_set<std::string> PartNamesFromConfigs(
    absl::Span<const intrinsic_proto::icon::v1::PartConfig> part_configs) {
  absl::flat_hash_set<std::string> part_names;
  for (const auto& part_config : part_configs) {
    part_names.insert(part_config.name());
  }
  return part_names;
}

}  // namespace

// static
absl::StatusOr<std::unique_ptr<RtclControllerFakeRobotConnection>>
RtclControllerFakeRobotConnection::Create(
    absl::string_view server_name,
    RtclControllerFakeSession::ValidationMode validation_mode,
    const intrinsic_proto::icon::v1::GetStatusResponse& initial_robot_status,
    absl::Span<const intrinsic_proto::icon::v1::PartConfig>
        part_configs_in_index_order,
    absl::Span<const SessionWill> session_behaviors,
    double control_frequency_hz) {
  auto status_mutex = std::make_unique<absl::Mutex>();
  auto part_property_map = std::make_unique<PartPropertyMap>();
  auto robot_status_proto =
      std::make_unique<intrinsic_proto::icon::v1::GetStatusResponse>(
          initial_robot_status);
  auto current_time_ns = std::make_unique<int>();
  absl::flat_hash_map<std::string, size_t> part_name_to_realtime_index;
  for (size_t i = 0; i < part_configs_in_index_order.size(); ++i) {
    part_name_to_realtime_index[part_configs_in_index_order[i].name()] = i;
    LOG(INFO) << "Assigning " << part_configs_in_index_order[i].name()
              << " to index " << i;
  }
  INTR_ASSIGN_OR_RETURN(
      PartCollection part_collection,
      PartCollection::Create(
          part_configs_in_index_order,
          std::make_unique<ChannelFakeStatusGetter>(
              *status_mutex, *part_property_map, robot_status_proto.get(),
              current_time_ns.get(),
              PartNamesFromConfigs(part_configs_in_index_order))));
  auto connection = absl::WrapUnique(new RtclControllerFakeRobotConnection(
      server_name, validation_mode, session_behaviors, std::move(status_mutex),
      std::move(part_property_map), std::move(robot_status_proto),
      std::move(current_time_ns), std::move(part_collection),
      part_name_to_realtime_index, control_frequency_hz));
  INTR_RETURN_IF_ERROR(connection->MutableOperationalStateInterface().Enable());
  return connection;
}

RtclControllerFakeRobotConnection::RtclControllerFakeRobotConnection(
    absl::string_view server_name,
    RtclControllerFakeSession::ValidationMode validation_mode,
    absl::Span<const SessionWill> session_behaviors,
    std::unique_ptr<absl::Mutex> status_mutex,
    std::unique_ptr<PartPropertyMap> part_property_map,
    std::unique_ptr<intrinsic_proto::icon::v1::GetStatusResponse>
        initial_robot_status,
    std::unique_ptr<int> current_time_ns, PartCollection part_collection,
    const absl::flat_hash_map<std::string, size_t>& part_name_to_index_map,
    double control_frequency_hz)
    : config_(MakeServerConfig(server_name, control_frequency_hz)),
      part_name_to_index_map_(part_name_to_index_map),
      status_mutex_(std::move(status_mutex)),
      part_property_map_(std::move(part_property_map)),
      current_robot_status_(std::move(initial_robot_status)),
      current_time_ns_(std::move(current_time_ns)),
      part_collection_(std::move(part_collection)),
      validation_mode_(validation_mode),
      session_behaviors_(session_behaviors.begin(), session_behaviors.end()),
      part_configs_by_name_(
          MakePartConfigsByName(part_collection_.GetPartConfigs())) {}

absl::StatusOr<std::unique_ptr<SessionInterface>>
RtclControllerFakeRobotConnection::CreateSession(
    const absl::flat_hash_set<std::string>& part_names, SessionId session_id,
    const RealtimeLogContext& context) {
  absl::MutexLock l(session_mutex_);
  SessionWill session_behavior;
  if (next_session_expectation_index_ < session_behaviors_.size()) {
    session_behavior = session_behaviors_.at(next_session_expectation_index_++);
  } else {
    if (validation_mode_ ==
        RtclControllerFakeSession::ValidationMode::kStrict) {
      return absl::FailedPreconditionError(
          absl::StrCat("ChannelFake only expected ", session_behaviors_.size(),
                       " sessions and is in strict mode. If you want to allow "
                       "extra sessions with no expectations, use nice mode."));
    }
  }
  absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::PartConfig>
      filtered_part_configs_by_name;
  for (const auto& part_name : part_names) {
    auto it = part_configs_by_name_.find(part_name);
    if (it == part_configs_by_name_.end()) {
      return absl::NotFoundError(
          absl::StrCat("Part '", part_name, "' not found."));
    }
    filtered_part_configs_by_name[part_name] = it->second;
  }

  return std::make_unique<RtclControllerFakeSession>(
      session_behavior, filtered_part_configs_by_name, validation_mode_,
      operational_state_,
      absl::bind_front(&RtclControllerFakeRobotConnection::UpdateCurrentStatus,
                       this),
      absl::bind_front(&RtclControllerFakeRobotConnection::GetCurrentStatus,
                       this),
      &part_name_to_index_map_, config_.frequency_hz());
}

void RtclControllerFakeRobotConnection::UpdateCurrentStatus(
    const intrinsic_proto::icon::v1::GetStatusResponse& new_status) {
  absl::MutexLock l(*status_mutex_);
  *current_time_ns_ +=
      absl::ToInt64Nanoseconds(absl::Seconds(1.0 / config_.frequency_hz()));
  *current_robot_status_ = new_status;
}

PartPropertyMap RtclControllerFakeRobotConnection::GetPartPropertiesTestOnly()
    const {
  absl::MutexLock l(*status_mutex_);
  return *part_property_map_;
}

void RtclControllerFakeRobotConnection::SetPartPropertiesTestOnly(
    const PartPropertyMap& new_part_properties) {
  absl::MutexLock l(*status_mutex_);
  *part_property_map_ = new_part_properties;
}

absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::ActionSignature>
RtclControllerFakeRobotConnection::ActionTypeToSignature() const {
  return GetGlobalRtclActionFactoryRegistry().GetAllSignatures();
}

absl::Status RtclControllerFakeRobotConnection::ActionCompatibleWithSlotPartMap(
    absl::string_view action_type_name,
    const SlotPartMap& slot_part_map) const {
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::icon::v1::ActionSignature action_signature,
      GetGlobalRtclActionFactoryRegistry().GetSignature(action_type_name));
  return ::intrinsic::icon::ActionCompatibleWithSlotPartMap(
      slot_part_map, action_signature, part_collection_.GetPartConfigs());
}

absl::Status RtclControllerFakeRobotConnection::ActionCompatibleWithPart(
    absl::string_view action_type_name, absl::string_view part_name) const {
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::icon::v1::ActionSignature action_signature,
      GetGlobalRtclActionFactoryRegistry().GetSignature(action_type_name));
  auto part_config = part_configs_by_name_.find(part_name);
  if (part_config == part_configs_by_name_.end()) {
    return absl::NotFoundError(
        absl::StrCat("No part named '", part_name,
                     "', did you register a PartConfig proto for that part?"));
  }
  // Try to find (at least) one Slot for which the Part in question has *all*
  // required FeatureInterfaces.
  absl::flat_hash_map<std::string, SlotPartCompatibility>
      slot_compatibility_map;
  for (const auto& [slot_name, slot_info] :
       action_signature.part_slot_infos()) {
    if (const SlotPartCompatibility compatibility =
            PartCompatibleWithSlot(part_config->second, slot_info);
        compatibility.Compatible()) {
      return absl::OkStatus();
    } else {
      slot_compatibility_map[slot_name] = compatibility;
    }
  }

  return absl::InvalidArgumentError(absl::StrCat(
      "Part '", part_name,
      "' is not compatible with any of the Slots for Action type '",
      action_type_name, "'. Available Slots: [",
      absl::StrJoin(action_signature.part_slot_infos(), ", ",
                    [](std::string* str, const auto& slot_name_and_info) {
                      absl::StrAppend(str, slot_name_and_info.first);
                    }),
      // Appends the explanations why the part is not compatible to the
      // respective slots.
      "] Detailed Explanation: ",
      absl::StrJoin(
          slot_compatibility_map, ", ",
          [](std::string* str, const auto& slot_name_and_compatibility) {
            absl::string_view slot_name = slot_name_and_compatibility.first;
            const SlotPartCompatibility& compatibility =
                slot_name_and_compatibility.second;
            if (compatibility.missing_required_interfaces.empty() &&
                !compatibility.missing_optional_interfaces.empty()) {
              absl::StrAppend(
                  str, "Slot '", slot_name,
                  "' requires at least one of these optional interfaces "
                  "[",
                  absl::StrJoin(compatibility.missing_optional_interfaces,
                                ", "),
                  "]. ");
            }
            if (!compatibility.missing_required_interfaces.empty()) {
              absl::StrAppend(
                  str,
                  "The part is missing the following required interfaces for "
                  "slot '",
                  slot_name, "' [",
                  absl::StrJoin(compatibility.missing_required_interfaces,
                                ", "),
                  "]. ");
            }
          }),
      "]"));
}

void RtclControllerFakeRobotConnection::SetSpeedOverride(
    double new_speed_override) {
  absl::MutexLock l(speed_override_mutex_);
  speed_override_ = new_speed_override;
}

double RtclControllerFakeRobotConnection::GetSpeedOverride() const {
  absl::MutexLock l(speed_override_mutex_);
  return speed_override_;
}

intrinsic_proto::icon::v1::GetStatusResponse
RtclControllerFakeRobotConnection::GetCurrentStatus() const {
  absl::MutexLock l(*status_mutex_);
  return *current_robot_status_;
}

void RtclControllerFakeRobotConnection::SetLoggingMode(
    LoggingMode logging_mode) {
  absl::MutexLock l(logging_mode_mutex_);
  logging_mode_ = logging_mode;
}

LoggingMode RtclControllerFakeRobotConnection::GetLoggingMode() const {
  absl::MutexLock l(logging_mode_mutex_);
  return logging_mode_;
}

absl::StatusOr<absl::flat_hash_map<std::string, OperationalState>>
RtclControllerFakeRobotConnection::GetPartStates() {
  INTR_ASSIGN_OR_RETURN(OperationalStatus status,
                        operational_state_.GetStatus());
  absl::flat_hash_map<std::string, OperationalState> result;
  for (const auto& [part_name, unused] : part_configs_by_name_) {
    result[part_name] = status.state();
  }
  return result;
}

}  // namespace intrinsic::icon
