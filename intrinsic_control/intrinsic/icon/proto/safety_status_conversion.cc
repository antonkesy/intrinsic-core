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

#include "intrinsic/icon/proto/safety_status_conversion.h"

#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/proto/safety_status.pb.h"

namespace intrinsic::icon {

intrinsic_proto::icon::ButtonStatus ToProto(
    const intrinsic_fbs::ButtonStatus& button_status) {
  switch (button_status) {
    case intrinsic_fbs::ButtonStatus::NOT_AVAILABLE:
      return intrinsic_proto::icon::BUTTON_STATUS_NOT_AVAILABLE;
    case intrinsic_fbs::ButtonStatus::ENGAGED:
      return intrinsic_proto::icon::BUTTON_STATUS_ENGAGED;
    case intrinsic_fbs::ButtonStatus::DISENGAGED:
      return intrinsic_proto::icon::BUTTON_STATUS_DISENGAGED;
    default:
      return intrinsic_proto::icon::BUTTON_STATUS_UNKNOWN;
  }
}

intrinsic_proto::icon::ModeOfSafeOperation ToProto(
    const intrinsic_fbs::ModeOfSafeOperation& mode_of_safe_operation) {
  switch (mode_of_safe_operation) {
    case intrinsic_fbs::ModeOfSafeOperation::TEACH_PENDANT_1:
      return intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_TEACHING_1;
    case intrinsic_fbs::ModeOfSafeOperation::TEACH_PENDANT_2:
      return intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_TEACHING_2;
    case intrinsic_fbs::ModeOfSafeOperation::AUTOMATIC:
      return intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_AUTOMATIC;
    case intrinsic_fbs::ModeOfSafeOperation::CONFIGURATION:
      return intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_CONFIGURATION;
    case intrinsic_fbs::ModeOfSafeOperation::UPDATING:
      return intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_UPDATING;
    default:
      return intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_UNKNOWN;
  }
}

intrinsic_proto::icon::SafetyStatus ToProto(
    const intrinsic::icon::SafetyStatus& safety_status) {
  intrinsic_proto::icon::SafetyStatus proto;

  proto.set_mode_of_safe_operation(
      ToProto(safety_status.mode_of_safe_operation));
  proto.set_enable_button_status(ToProto(safety_status.enable_button_status));
  proto.set_estop_button_status(ToProto(safety_status.estop_button_status));
  proto.set_requested_behavior(ToProto(safety_status.requested_behavior));

  return proto;
}

intrinsic_proto::icon::RequestedBehavior ToProto(
    const intrinsic_fbs::RequestedBehavior& requested_behavior) {
  switch (requested_behavior) {
    case intrinsic_fbs::RequestedBehavior::NORMAL_OPERATION:
      return intrinsic_proto::icon::REQUESTED_BEHAVIOR_NORMAL_OPERATION;
    case intrinsic_fbs::RequestedBehavior::SAFE_STOP_0:
      return intrinsic_proto::icon::REQUESTED_BEHAVIOR_SAFE_STOP_0;
    case intrinsic_fbs::RequestedBehavior::SAFE_STOP_1_TIME_MONITORED:
      return intrinsic_proto::icon::
          REQUESTED_BEHAVIOR_SAFE_STOP_1_TIME_MONITORED;
    case intrinsic_fbs::RequestedBehavior::SAFE_STOP_2_TIME_MONITORED:
      return intrinsic_proto::icon::
          REQUESTED_BEHAVIOR_SAFE_STOP_2_TIME_MONITORED;
    case intrinsic_fbs::RequestedBehavior::PAUSE:
      return intrinsic_proto::icon::REQUESTED_BEHAVIOR_PAUSE;
    default:
      return intrinsic_proto::icon::REQUESTED_BEHAVIOR_UNKNOWN;
  }
}

intrinsic_fbs::ModeOfSafeOperation FromProto(
    const intrinsic_proto::icon::ModeOfSafeOperation& mode_of_safe_operation) {
  switch (mode_of_safe_operation) {
    case intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_TEACHING_1:
      return intrinsic_fbs::ModeOfSafeOperation::TEACH_PENDANT_1;
    case intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_TEACHING_2:
      return intrinsic_fbs::ModeOfSafeOperation::TEACH_PENDANT_2;
    case intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_AUTOMATIC:
      return intrinsic_fbs::ModeOfSafeOperation::AUTOMATIC;
    case intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_UPDATING:
      return intrinsic_fbs::ModeOfSafeOperation::UPDATING;
      break;
    case intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_CONFIGURATION:
      return intrinsic_fbs::ModeOfSafeOperation::CONFIGURATION;
      break;
    case intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_UNKNOWN:
      return intrinsic_fbs::ModeOfSafeOperation::UNKNOWN;
    default:
      // Default is required for the sentinel values of the proto.
      return intrinsic_fbs::ModeOfSafeOperation::UNKNOWN;
  }
}

intrinsic_fbs::RequestedBehavior FromProto(
    const intrinsic_proto::icon::RequestedBehavior& requested_behavior) {
  switch (requested_behavior) {
    case intrinsic_proto::icon::REQUESTED_BEHAVIOR_NORMAL_OPERATION:
      return intrinsic_fbs::RequestedBehavior::NORMAL_OPERATION;
    case intrinsic_proto::icon::REQUESTED_BEHAVIOR_SAFE_STOP_0:
      return intrinsic_fbs::RequestedBehavior::SAFE_STOP_0;
    case intrinsic_proto::icon::REQUESTED_BEHAVIOR_SAFE_STOP_1_TIME_MONITORED:
      return intrinsic_fbs::RequestedBehavior::SAFE_STOP_1_TIME_MONITORED;
    case intrinsic_proto::icon::REQUESTED_BEHAVIOR_SAFE_STOP_2_TIME_MONITORED:
      return intrinsic_fbs::RequestedBehavior::SAFE_STOP_2_TIME_MONITORED;
    case intrinsic_proto::icon::REQUESTED_BEHAVIOR_PAUSE:
      return intrinsic_fbs::RequestedBehavior::PAUSE;
    case intrinsic_proto::icon::REQUESTED_BEHAVIOR_UNKNOWN:
      return intrinsic_fbs::RequestedBehavior::UNKNOWN;
    default:
      // Default is required for the sentinel values of the proto.
      return intrinsic_fbs::RequestedBehavior::UNKNOWN;
  }
}

intrinsic_fbs::ButtonStatus FromProto(
    const intrinsic_proto::icon::ButtonStatus& button_status) {
  switch (button_status) {
    case intrinsic_proto::icon::BUTTON_STATUS_NOT_AVAILABLE:
      return intrinsic_fbs::ButtonStatus::NOT_AVAILABLE;
    case intrinsic_proto::icon::BUTTON_STATUS_ENGAGED:
      return intrinsic_fbs::ButtonStatus::ENGAGED;
    case intrinsic_proto::icon::BUTTON_STATUS_DISENGAGED:
      return intrinsic_fbs::ButtonStatus::DISENGAGED;
    case intrinsic_proto::icon::BUTTON_STATUS_UNKNOWN:
      return intrinsic_fbs::ButtonStatus::UNKNOWN;
    default:
      // Default is required for the sentinel values of the proto.
      return intrinsic_fbs::ButtonStatus::UNKNOWN;
  }
}

intrinsic::icon::SafetyStatus FromProto(
    const intrinsic_proto::icon::SafetyStatus& safety_status) {
  SafetyStatus status;
  status.mode_of_safe_operation =
      FromProto(safety_status.mode_of_safe_operation());
  status.enable_button_status = FromProto(safety_status.enable_button_status());
  status.estop_button_status = FromProto(safety_status.estop_button_status());
  status.requested_behavior = FromProto(safety_status.requested_behavior());
  return status;
}

}  // namespace intrinsic::icon
