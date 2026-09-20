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

#ifndef INTRINSIC_ICON_CONTROL_ROBOT_CONNECTION_RTCL_CONTROLLER_H_
#define INTRINSIC_ICON_CONTROL_ROBOT_CONNECTION_RTCL_CONTROLLER_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/slot_part_map.h"
#include "intrinsic/icon/control/initialized_async_buffer.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/rtcl_controller_bridge_interface.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/operational_state_interface.h"
#include "intrinsic/icon/server/part_collection.h"
#include "intrinsic/icon/server/robot_connection_interface.h"
#include "intrinsic/icon/server/session_interface.h"
#include "intrinsic/logging/proto/context.pb.h"

namespace intrinsic::icon {

// A RobotConnection that talks to an RtclController. RtclController runs
// RtclAction instances in a realtime thread.
//
// All methods of this class are thread-safe, to fulfill the API contract of
// RobotConnectionInterface.
class RobotConnectionRtclController final : public RobotConnectionInterface {
 public:
  // Creates a new RobotConnectionRtclController.
  // Consumes `part_configs_in_index_order`.
  // Holds references to `rtcl_controller_bridge` and `operational_state`.
  // Holds a pointer to a PubSub instance. This may be nullptr, but if it is
  // not, the instance must outlive this class.
  // Holds a reference to `robot_status_buffer`, which must outlive this class.
  static absl::StatusOr<std::unique_ptr<RobotConnectionRtclController>> Create(
      const intrinsic_proto::icon::v1::ServerConfig& server_config,
      std::vector<intrinsic_proto::icon::v1::PartConfig>
          part_configs_in_index_order,
      RtclControllerBridgeInterface& rtcl_controller_bridge
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      OperationalStateInterface& operational_state
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      InitializedAsyncBuffer<PublishOutput>& robot_status_buffer
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer
          ABSL_ATTRIBUTE_LIFETIME_BOUND);

  absl::StatusOr<std::unique_ptr<SessionInterface>> CreateSession(
      const absl::flat_hash_set<std::string>& part_names, SessionId session_id,
      const RealtimeLogContext& log_context) override;

  OperationalStateInterface& MutableOperationalStateInterface() override;

  absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::ActionSignature>
  ActionTypeToSignature() const override;

  PartCollection& MutablePartCollection()
      ABSL_ATTRIBUTE_LIFETIME_BOUND override;

  const PartCollection& GetPartCollection() const
      ABSL_ATTRIBUTE_LIFETIME_BOUND override;

  const intrinsic_proto::icon::v1::ServerConfig& config() const
      ABSL_ATTRIBUTE_LIFETIME_BOUND override;

  absl::Status ActionCompatibleWithSlotPartMap(
      absl::string_view action_type_name,
      const SlotPartMap& slot_part_map) const override;

  absl::Status ActionCompatibleWithPart(
      absl::string_view action_type_name,
      absl::string_view part_name) const override;

  void SetSpeedOverride(double new_speed_override) override;

  double GetSpeedOverride() const override;

  void SetLoggingMode(LoggingMode logging_mode) override;

  LoggingMode GetLoggingMode() const override;

  absl::StatusOr<absl::flat_hash_map<std::string, OperationalState>>
  GetPartStates() override;

 private:
  // Copies `server_config` and `part_name_to_realtime_index`, consumes
  // `part_collection`. The remaining parameters must outlive the
  // RobotConnectionRtclController.
  RobotConnectionRtclController(
      const intrinsic_proto::icon::v1::ServerConfig& server_config,
      const absl::flat_hash_map<std::string, size_t>&
          part_name_to_realtime_index,
      PartCollection part_collection,
      RtclControllerBridgeInterface& rtcl_controller_bridge
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      OperationalStateInterface& operational_state
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      InitializedAsyncBuffer<PublishOutput>& robot_status_buffer
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer
          ABSL_ATTRIBUTE_LIFETIME_BOUND);

  const intrinsic_proto::icon::v1::ServerConfig server_config_;
  const absl::flat_hash_map<std::string,
                            intrinsic_proto::icon::v1::ActionSignature>
      action_signature_map_;
  const absl::flat_hash_map<std::string, size_t> part_name_to_realtime_index_;
  PartCollection part_collection_;
  RtclControllerBridgeInterface& rtcl_controller_bridge_;
  OperationalStateInterface& operational_state_;
  InitializedAsyncBuffer<PublishOutput>& robot_status_buffer_;
  InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ROBOT_CONNECTION_RTCL_CONTROLLER_H_
