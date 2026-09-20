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

#include "intrinsic/icon/control/rtcl_controller_bridge.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/control/initialized_async_buffer.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/parts/realtime_part_status.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_part_manager.h"
#include "intrinsic/icon/control/realtime_session_channels.h"
#include "intrinsic/icon/control/rtcl_controller_bridge_interface.h"
#include "intrinsic/icon/control/rtcl_controller_session_bridge.h"
#include "intrinsic/icon/control/rtcl_session_bridge_interface.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {
namespace {

absl::flat_hash_map<std::string, RealtimePartStatus> MakePartStatusMap(
    absl::Span<const std::string> part_names,
    absl::Span<const RealtimePartStatus> part_statuses) {
  absl::flat_hash_map<std::string, RealtimePartStatus> status_by_part_name;
  for (size_t i = 0; i < part_names.size() && i < part_statuses.size(); ++i) {
    status_by_part_name.emplace(part_names[i], part_statuses[i]);
  }
  return status_by_part_name;
}

absl::StatusOr<absl::flat_hash_set<size_t>> FindPartIndices(
    const absl::flat_hash_set<std::string>& part_names,
    absl::Span<const std::string> part_names_in_index_order) {
  absl::flat_hash_set<size_t> part_indices;
  for (const auto& part_name : part_names) {
    std::optional<size_t> current_index = std::nullopt;
    for (size_t i = 0; i < part_names_in_index_order.size(); ++i) {
      if (part_names_in_index_order[i] == part_name) {
        current_index = i;
        break;
      }
    }
    if (current_index == std::nullopt) {
      return absl::NotFoundError(
          absl::StrCat("Part '", part_name, "' does not exist."));
    }
    part_indices.insert(current_index.value());
  }
  return part_indices;
}

absl::flat_hash_map<std::string, size_t> MakePartIndexMap(
    const std::vector<std::string>& part_names_in_index_order) {
  absl::flat_hash_map<std::string, size_t> map;
  for (size_t part_index = 0; part_index < part_names_in_index_order.size();
       ++part_index) {
    map[part_names_in_index_order.at(part_index)] = part_index;
  }

  return map;
}

absl::flat_hash_map<std::string, absl::flat_hash_map<std::string, size_t>>
MakePropertyIndexMap(const std::vector<std::string>& part_names_in_index_order,
                     const std::vector<std::vector<std::string>>&
                         part_index_to_property_names_in_index_order) {
  absl::flat_hash_map<std::string, absl::flat_hash_map<std::string, size_t>>
      map;
  for (size_t part_index = 0;
       part_index <
       std::min(part_names_in_index_order.size(),
                part_index_to_property_names_in_index_order.size());
       ++part_index) {
    const auto& part_name = part_names_in_index_order.at(part_index);
    const auto& properties_for_part =
        part_index_to_property_names_in_index_order.at(part_index);
    // Even if size is 0, this will create an entry in the map for part_name.
    // Having that entry is useful because it allows us to provide nicer error
    // messages.
    map[part_name].reserve(properties_for_part.size());
    for (size_t property_index = 0; property_index < properties_for_part.size();
         ++property_index) {
      map[part_name][properties_for_part.at(property_index)] = property_index;
    }
  }
  return map;
}

}  // namespace

RtclControllerBridge::RtclControllerBridge(
    std::vector<std::string> part_names_in_index_order,
    const std::vector<std::vector<std::string>>
        part_index_to_property_names_in_index_order,
    AsyncBuffer<RealtimePartManager::AllPartProperties>&
        part_properties_rt_to_non_rt_buffer ABSL_ATTRIBUTE_LIFETIME_BOUND,
    AsyncBuffer<RealtimePartManager::AllPartProperties>&
        part_properties_non_rt_to_rt_buffer ABSL_ATTRIBUTE_LIFETIME_BOUND,
    StartRealtimeSessionBridge& start_realtime_session_bridge,
    InitializedAsyncBuffer<PublishOutput>& robot_status_buffer,
    std::atomic<double>& speed_override_buffer,
    InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer,
    AsyncBuffer<FixedVector<OperationalState, kMaxRealtimeParts>>&
        part_states_buffer)
    : part_names_in_index_order_(std::move(part_names_in_index_order)),
      part_name_to_part_index_(MakePartIndexMap(part_names_in_index_order_)),
      part_index_to_property_names_in_index_order_(
          part_index_to_property_names_in_index_order),
      part_property_to_index_map_(
          MakePropertyIndexMap(part_names_in_index_order_,
                               part_index_to_property_names_in_index_order_)),
      part_properties_rt_to_non_rt_buffer_(part_properties_rt_to_non_rt_buffer),
      part_properties_non_rt_to_rt_buffer_(part_properties_non_rt_to_rt_buffer),
      part_states_buffer_(part_states_buffer),
      start_realtime_session_bridge_(start_realtime_session_bridge),
      robot_status_buffer_(robot_status_buffer),
      speed_override_buffer_(speed_override_buffer),
      logging_mode_buffer_(logging_mode_buffer) {}

absl::StatusOr<RealtimePartStatuses> RtclControllerBridge::GetPartStatuses(
    absl::Time deadline) {
  std::optional<PublishOutput> output =
      robot_status_buffer_.GetCurrentValue(deadline);
  if (!output.has_value()) {
    return absl::DeadlineExceededError(
        "Deadline exceeded, shutdown or error while trying to read Part status "
        "from realtime thread.");
  }
  if (output->robot_status.part_statuses.size() !=
      part_names_in_index_order_.size()) {
    return absl::InternalError(absl::StrCat(
        "Part status vector has ", output->robot_status.part_statuses.size(),
        " elements, but we expected there to be ",
        part_names_in_index_order_.size(),
        " Parts. This is a programming error in ICON."));
  }
  return RealtimePartStatuses{
      .wall_time = output->wall_time,
      .status_by_part_name = MakePartStatusMap(
          part_names_in_index_order_, output->robot_status.part_statuses)};
}

absl::StatusOr<std::unique_ptr<RtclSessionBridgeInterface>>
RtclControllerBridge::StartSession(
    SessionId session_id, const absl::flat_hash_set<std::string>& part_names,
    const RealtimeLogContext& context) {
  INTR_ASSIGN_OR_RETURN(
      absl::flat_hash_set<size_t> part_ids,
      FindPartIndices(part_names, part_names_in_index_order_));

  auto channels = std::make_unique<RealtimeSessionChannels>();
  INTR_ASSIGN_OR_RETURN(RealtimeStatus start_session_status,
                        start_realtime_session_bridge_.Call(
                            session_id, part_ids, channels.get(), context));
  INTR_RETURN_IF_ERROR(start_session_status);
  return std::make_unique<RtclControllerSessionBridge>(session_id,
                                                       std::move(channels));
}

void RtclControllerBridge::SetSpeedOverride(double new_speed_override) {
  absl::MutexLock lock(speed_override_mutex_);
  speed_override_buffer_ = new_speed_override;
}

double RtclControllerBridge::GetSpeedOverride() const {
  absl::MutexLock lock(speed_override_mutex_);
  return speed_override_buffer_;
}

absl::StatusOr<RealtimeSafetyStatus> RtclControllerBridge::GetSafetyStatus(
    absl::Time deadline) const {
  std::optional<PublishOutput> output =
      robot_status_buffer_.GetCurrentValue(deadline);
  if (!output.has_value()) {
    return absl::DeadlineExceededError(
        "Deadline exceeded while trying to read SafetyStatus from realtime "
        "thread.");
  }
  return RealtimeSafetyStatus{
      .wall_time = output->wall_time,
      .safety_status = output->robot_status.safety_status};
}

absl::StatusOr<TimestampedPartProperties>
RtclControllerBridge::GetPartProperties() {
  absl::MutexLock lock(part_property_mutex_);
  RealtimePartManager::AllPartProperties* current_properties_from_rt_thread;
  part_properties_rt_to_non_rt_buffer_.GetActiveBuffer(
      &current_properties_from_rt_thread);
  TimestampedPartProperties timestamped_properties;
  timestamped_properties.timestamp_control =
      current_properties_from_rt_thread->timestamp_control;
  timestamped_properties.timestamp_wall =
      current_properties_from_rt_thread->timestamp_wall;
  if (part_index_to_property_names_in_index_order_.size() !=
      current_properties_from_rt_thread->properties.size()) {
    return absl::InternalError(absl::StrCat(
        "Size mismatch for part properties, expected ",
        part_index_to_property_names_in_index_order_.size(), " parts, got ",
        current_properties_from_rt_thread->properties.size()));
  }
  for (size_t part_index = 0;
       part_index < part_index_to_property_names_in_index_order_.size();
       ++part_index) {
    const auto& property_names_in_index_order =
        part_index_to_property_names_in_index_order_.at(part_index);
    const auto& properties =
        current_properties_from_rt_thread->properties.at(part_index);
    const auto& part_name = part_names_in_index_order_.at(part_index);
    if (property_names_in_index_order.size() != properties.size()) {
      return absl::InternalError(
          absl::StrCat("Size mismatch for properties of part '", part_name,
                       "': expected ", property_names_in_index_order.size(),
                       " parts, got ", properties.size()));
    }
    absl::flat_hash_map<std::string, std::variant<bool, double>> property_map;
    for (size_t property_index = 0;
         property_index <
         std::min(property_names_in_index_order.size(), properties.size());
         ++property_index) {
      property_map[property_names_in_index_order.at(property_index)] =
          properties.at(property_index);
    }
    timestamped_properties.properties[part_name] = std::move(property_map);
  }
  return timestamped_properties;
}

absl::Status RtclControllerBridge::SetPartProperties(
    const PartPropertyMap& properties) {
  absl::MutexLock lock(part_property_mutex_);
  // It's safe to call this multiple times before calling CommitFreeBuffer(), so
  // we don't need to set up an absl::Cleanup in case we need to bail out due to
  // errors below.
  RealtimePartManager::AllPartProperties* current_properties_to_rt_thread =
      part_properties_non_rt_to_rt_buffer_.GetFreeBuffer();

  for (const auto& [part_name, property_map] : properties.properties) {
    auto property_indices_for_part =
        part_property_to_index_map_.find(part_name);
    if (property_indices_for_part == part_property_to_index_map_.end()) {
      return absl::NotFoundError(absl::StrCat(
          "Cannot set properties for unknown part '", part_name, "'"));
    }
    size_t part_index = part_name_to_part_index_.at(part_name);
    for (const auto& [property_name, property_value] : property_map) {
      auto property_index =
          property_indices_for_part->second.find(property_name);
      if (property_index == property_indices_for_part->second.end()) {
        return absl::NotFoundError(absl::StrCat("Cannot set unknown property '",
                                                property_name, "' for part '",
                                                part_name, "'"));
      }
      PartPropertyValue& property_value_out =
          current_properties_to_rt_thread->properties.at(part_index)
              .at(property_index->second);
      INTR_RETURN_IF_ERROR(
          std::visit(AssignPropertyValue{.property_name = property_name},
                     /*src=*/property_value, /*dst=*/property_value_out));
    }
  }
  part_properties_non_rt_to_rt_buffer_.CommitFreeBuffer();
  return absl::OkStatus();
}

absl::StatusOr<absl::flat_hash_map<std::string, OperationalState>>
RtclControllerBridge::GetPartStates() {
  absl::MutexLock lock(part_property_mutex_);
  FixedVector<OperationalState, kMaxRealtimeParts>* current_part_states;
  part_states_buffer_.GetActiveBuffer(&current_part_states);
  if (current_part_states->empty()) {
    return absl::NotFoundError("No part states available.");
  }
  absl::flat_hash_map<std::string, OperationalState> part_states;
  for (size_t i = 0;
       i < current_part_states->size() && i < part_names_in_index_order_.size();
       ++i) {
    part_states[part_names_in_index_order_[i]] = (*current_part_states)[i];
  }
  return part_states;
}

void RtclControllerBridge::SetLoggingMode(LoggingMode logging_mode) {
  absl::MutexLock lock(logging_mode_mutex_);
  logging_mode_ = logging_mode;
  *logging_mode_buffer_.GetFreeBuffer() = logging_mode_;
  (void)logging_mode_buffer_.CommitFreeBuffer();
}

LoggingMode RtclControllerBridge::GetLoggingMode() const {
  absl::MutexLock lock(logging_mode_mutex_);
  return logging_mode_;
}

}  // namespace intrinsic::icon
