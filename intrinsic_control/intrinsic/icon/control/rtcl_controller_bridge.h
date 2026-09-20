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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_BRIDGE_H_
#define INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_BRIDGE_H_

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/control/initialized_async_buffer.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_function_bridge.h"
#include "intrinsic/icon/control/realtime_part_manager.h"
#include "intrinsic/icon/control/realtime_session_channels.h"
#include "intrinsic/icon/control/rtcl_controller_bridge_interface.h"
#include "intrinsic/icon/control/rtcl_session_bridge_interface.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/logging/proto/context.pb.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

// RtclControllerBridge is the immediate interface between RtclController's
// realtime thread on the one hand, and the gRPC server thread (via
// RobotConnectionRtclController) on the other.
class RtclControllerBridge : public RtclControllerBridgeInterface {
 public:
  using StartRealtimeSessionBridge = RealtimeFunctionBridge<RealtimeStatus(
      SessionId session_id, absl::flat_hash_set<size_t> part_indices,
      RealtimeSessionChannels* channels, RealtimeLogContext log_context)>;

  // Consumes `part_names_in_index_order`.
  //
  // MUST NOT outlive `start_realtime_session_bridge`, `robot_status_buffer` or
  // `speed_override_buffer`!
  //
  // Assumes the exclusive right to write to `speed_override_buffer`
  RtclControllerBridge(
      std::vector<std::string> part_names_in_index_order,
      // A nested vector that helps determine the index for each part property
      // of each part. Each entry of the outer vector contains the part property
      // names for that part (to determine the name, we can use
      // `part_names_in_index_order` above). The inner vector is built so that
      // the order of names is the same as the order of property values in the
      // FixedArrays that `part_properties_rt_to_non_rt_buffer` and
      // `part_properties_non_rt_to_rt_buffer` hold.
      std::vector<std::vector<std::string>>
          part_index_to_property_names_in_index_order,
      AsyncBuffer<RealtimePartManager::AllPartProperties>&
          part_properties_rt_to_non_rt_buffer ABSL_ATTRIBUTE_LIFETIME_BOUND,
      AsyncBuffer<RealtimePartManager::AllPartProperties>&
          part_properties_non_rt_to_rt_buffer ABSL_ATTRIBUTE_LIFETIME_BOUND,
      StartRealtimeSessionBridge& start_realtime_session_bridge
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      InitializedAsyncBuffer<PublishOutput>& robot_status_buffer
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      std::atomic<double>& speed_override_buffer ABSL_ATTRIBUTE_LIFETIME_BOUND,
      InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      AsyncBuffer<FixedVector<OperationalState, kMaxRealtimeParts>>&
          part_states_buffer ABSL_ATTRIBUTE_LIFETIME_BOUND);

  absl::StatusOr<RealtimePartStatuses> GetPartStatuses(
      absl::Time deadline) override;

  absl::StatusOr<std::unique_ptr<RtclSessionBridgeInterface>> StartSession(
      SessionId session_id, const absl::flat_hash_set<std::string>& part_names,
      const RealtimeLogContext& context) ABSL_ATTRIBUTE_LIFETIME_BOUND override;

  void SetSpeedOverride(double new_speed_override) override;

  double GetSpeedOverride() const override;

  absl::StatusOr<RealtimeSafetyStatus> GetSafetyStatus(
      absl::Time deadline) const override;

  absl::StatusOr<TimestampedPartProperties> GetPartProperties() override;

  absl::Status SetPartProperties(const PartPropertyMap& properties) override;

  absl::StatusOr<absl::flat_hash_map<std::string, OperationalState>>
  GetPartStates() override;

  void SetLoggingMode(LoggingMode logging_mode) override;

  LoggingMode GetLoggingMode() const override;

 private:
  const std::vector<std::string> part_names_in_index_order_;
  const absl::flat_hash_map<std::string, size_t> part_name_to_part_index_;
  const std::vector<std::vector<std::string>>
      part_index_to_property_names_in_index_order_;
  const absl::flat_hash_map<std::string,
                            absl::flat_hash_map<std::string, size_t>>
      part_property_to_index_map_;
  absl::Mutex part_property_mutex_;
  AsyncBuffer<RealtimePartManager::AllPartProperties>&
      part_properties_rt_to_non_rt_buffer_
          ABSL_GUARDED_BY(part_property_mutex_);
  AsyncBuffer<RealtimePartManager::AllPartProperties>&
      part_properties_non_rt_to_rt_buffer_
          ABSL_GUARDED_BY(part_property_mutex_);
  AsyncBuffer<FixedVector<OperationalState, kMaxRealtimeParts>>&
      part_states_buffer_ ABSL_GUARDED_BY(part_property_mutex_);

  // Attempts to start a Session in the realtime thread.
  //
  // Returns a pointer to the RealtimeSessionChannels object to use for
  // the Session if successful.
  StartRealtimeSessionBridge& start_realtime_session_bridge_;
  InitializedAsyncBuffer<PublishOutput>& robot_status_buffer_;
  mutable absl::Mutex speed_override_mutex_
      ABSL_ACQUIRED_AFTER(part_property_mutex_);
  std::atomic<double>& speed_override_buffer_
      ABSL_GUARDED_BY(speed_override_mutex_);
  mutable absl::Mutex logging_mode_mutex_
      ABSL_ACQUIRED_AFTER(speed_override_mutex_);
  LoggingMode logging_mode_ ABSL_GUARDED_BY(logging_mode_mutex_) =
      LoggingMode::kThrottled;
  InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer_
      ABSL_GUARDED_BY(logging_mode_mutex_);
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_BRIDGE_H_
