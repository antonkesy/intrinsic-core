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

#ifndef INTRINSIC_ICON_CC_CLIENT_TESTING_RTCL_CONTROLLER_FAKE_ROBOT_CONNECTION_H_
#define INTRINSIC_ICON_CC_CLIENT_TESTING_RTCL_CONTROLLER_FAKE_ROBOT_CONNECTION_H_

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
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/cc_client/testing/channel_fake.h"
#include "intrinsic/icon/cc_client/testing/rtcl_controller_fake_session.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/common/slot_part_map.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/proto/v1/service.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/operational_state_interface.h"
#include "intrinsic/icon/server/part_collection.h"
#include "intrinsic/icon/server/robot_connection_interface.h"
#include "intrinsic/icon/server/session_interface.h"
#include "intrinsic/icon/server/testing/fake_operational_state.h"

namespace intrinsic::icon {

// RtclControllerChannelFake uses this class to implement a fake ICON server.
// This class basically just forwards expectations to
// RtclControllerFakeSessions.
// This class is thread-safe, as required by the API contract of
// RobotConnectionInterface.
class RtclControllerFakeRobotConnection : public RobotConnectionInterface {
 public:
  static absl::StatusOr<std::unique_ptr<RtclControllerFakeRobotConnection>>
  Create(
      absl::string_view server_name,
      RtclControllerFakeSession::ValidationMode validation_mode,
      const intrinsic_proto::icon::v1::GetStatusResponse& initial_robot_status,
      absl::Span<const intrinsic_proto::icon::v1::PartConfig> part_configs,
      absl::Span<const SessionWill> session_behaviors,
      double control_frequency_hz = 1000.0);

  absl::StatusOr<std::unique_ptr<SessionInterface>> CreateSession(
      const absl::flat_hash_set<std::string>& part_names, SessionId session_id,
      const RealtimeLogContext& context) ABSL_ATTRIBUTE_LIFETIME_BOUND override;

  OperationalStateInterface& MutableOperationalStateInterface()
      ABSL_ATTRIBUTE_LIFETIME_BOUND override {
    return operational_state_;
  }

  FakeOperationalState& MutableFakeOperationalState()
      ABSL_ATTRIBUTE_LIFETIME_BOUND {
    return operational_state_;
  }

  void InduceFault(absl::string_view fault_reason) {
    operational_state_.Fault(fault_reason);
  }

  absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::ActionSignature>
  ActionTypeToSignature() const override;

  PartCollection& MutablePartCollection() override { return part_collection_; }

  const PartCollection& GetPartCollection() const
      ABSL_ATTRIBUTE_LIFETIME_BOUND override {
    return part_collection_;
  }

  const intrinsic_proto::icon::v1::ServerConfig& config() const
      ABSL_ATTRIBUTE_LIFETIME_BOUND override {
    return config_;
  }

  absl::Status ActionCompatibleWithSlotPartMap(
      absl::string_view action_type_name,
      const SlotPartMap& slot_part_map) const override;

  absl::Status ActionCompatibleWithPart(
      absl::string_view action_type_name,
      absl::string_view part_name) const override;

  void SetSpeedOverride(double new_speed_override) override;

  double GetSpeedOverride() const override;

  void UpdateCurrentStatus(
      const intrinsic_proto::icon::v1::GetStatusResponse& new_status);

  intrinsic_proto::icon::v1::GetStatusResponse GetCurrentStatus() const;

  // Returns the current map of part properties. Use this to validate that an
  // ICON client has updated the part properties that you expect it to.
  // Thread-safe.
  PartPropertyMap GetPartPropertiesTestOnly() const;

  // Overwrites the current part property map. Use this to initialize part
  // properties – `part_properties` determines which parts and properties
  // "exist" to the ICON API, and which type each property has.
  // Thread-safe.
  void SetPartPropertiesTestOnly(const PartPropertyMap& part_properties);

  void SetLoggingMode(LoggingMode logging_mode) override;

  LoggingMode GetLoggingMode() const override;

  // Returns the fake operational state for all parts.
  absl::StatusOr<absl::flat_hash_map<std::string, OperationalState>>
  GetPartStates() override;

 private:
  RtclControllerFakeRobotConnection(
      absl::string_view server_name,
      RtclControllerFakeSession::ValidationMode validation_mode,
      absl::Span<const SessionWill> session_behaviors,
      std::unique_ptr<absl::Mutex> status_mutex,
      std::unique_ptr<PartPropertyMap> part_property_map,
      std::unique_ptr<intrinsic_proto::icon::v1::GetStatusResponse>
          initial_robot_status,
      std::unique_ptr<int> current_time_ns, PartCollection part_collection,
      const absl::flat_hash_map<std::string, size_t>& part_name_to_index_map,
      double control_frequency_hz);

  const intrinsic_proto::icon::v1::ServerConfig config_;
  const absl::flat_hash_map<std::string, size_t> part_name_to_index_map_;
  std::unique_ptr<absl::Mutex> status_mutex_;
  std::unique_ptr<PartPropertyMap> part_property_map_
      ABSL_GUARDED_BY(status_mutex_);
  std::unique_ptr<intrinsic_proto::icon::v1::GetStatusResponse>
      current_robot_status_ ABSL_GUARDED_BY(status_mutex_);
  std::unique_ptr<int> current_time_ns_ ABSL_GUARDED_BY(status_mutex_);
  PartCollection part_collection_;
  const RtclControllerFakeSession::ValidationMode validation_mode_;
  const std::vector<SessionWill> session_behaviors_;
  absl::Mutex session_mutex_;
  size_t next_session_expectation_index_ ABSL_GUARDED_BY(session_mutex_) = 0;
  const absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::PartConfig>
      part_configs_by_name_;
  FakeOperationalState operational_state_;
  mutable absl::Mutex speed_override_mutex_;
  double speed_override_ ABSL_GUARDED_BY(speed_override_mutex_) = 1.0;
  mutable absl::Mutex logging_mode_mutex_;
  LoggingMode logging_mode_ ABSL_GUARDED_BY(logging_mode_mutex_) =
      LoggingMode::kThrottled;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CC_CLIENT_TESTING_RTCL_CONTROLLER_FAKE_ROBOT_CONNECTION_H_
