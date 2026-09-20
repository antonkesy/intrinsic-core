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

#include "intrinsic/icon/control/actions/force_torque_sensor_tare_action.h"

#include <memory>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/tare_force_torque_sensor_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// static
absl::StatusOr<std::unique_ptr<ForceTorqueSensorTareAction>>
ForceTorqueSensorTareAction::Create(
    TareForceTorqueSensorInfo::FixedParams params_proto,
    ActionFactoryContext& context) {
  INTR_ASSIGN_OR_RETURN(
      SlotInfo slot_info,
      context.GetSlotInfo(
          TareForceTorqueSensorInfo::kForceTorqueSensorSlotName));

  int num_taring_cycles = 1;
  if (params_proto.has_num_taring_cycles()) {
    if (params_proto.num_taring_cycles() == 0) {
      return absl::InvalidArgumentError("num_taring_cycles must be > 0");
    }
    num_taring_cycles = params_proto.num_taring_cycles();
  }

  return std::make_unique<ForceTorqueSensorTareAction>(slot_info.slot_id,
                                                       num_taring_cycles);
}

RealtimeStatus ForceTorqueSensorTareAction::OnEnter(OnEnterParameters params) {
  tare_requested_ = false;
  tare_completed_ = false;
  return OkStatus();
}

RealtimeStatus ForceTorqueSensorTareAction::Sense(SenseParameters params) {
  const ForceTorqueSensor* const force_torque_sensor_interface =
      params.slot_map.GetInterfaceForSlot<ForceTorqueSensor>(slot_id_);
  if (force_torque_sensor_interface == nullptr) {
    return InternalError("Slot doesn't have a ForceTorqueSensor.");
  }

  // Only listen for "completed" if a tare was actually requested.
  if (tare_requested_) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(tare_completed_,
                                  force_torque_sensor_interface->TareIsDone());
    if (tare_completed_) {
      INTRINSIC_RT_LOG_THROTTLED(INFO)
          << "Tare completed in ForceTorqueSensorTareAction. Expecting Action "
             "to be terminated.";
    }
  }

  return OkStatus();
}

RealtimeStatus ForceTorqueSensorTareAction::Control(ControlParameters params) {
  ForceTorqueSensor* const force_torque_sensor_interface =
      params.slot_map.GetMutableInterfaceForSlot<ForceTorqueSensor>(slot_id_);
  if (force_torque_sensor_interface == nullptr) {
    return InternalError("Slot doesn't have a ForceTorqueSensor.");
  }

  // This logic makes sure we do not request a tare repeatedly within a single
  // Action call.
  if (!tare_requested_) {
    INTRINSIC_RT_LOG(INFO) << "Tare requested in ForceTorqueSensorTareAction.";
    INTRINSIC_RT_RETURN_IF_ERROR(
        force_torque_sensor_interface->Tare(num_taring_cycles_));
    tare_requested_ = true;
  }
  return OkStatus();
}

RealtimeStatusOr<StateVariableValue>
ForceTorqueSensorTareAction::GetStateVariable(absl::string_view name) const {
  if (name == kIsDone) {
    return StateVariableValue(tare_completed_);
  }
  return NotFoundError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
      "ForceTorqueSensorTareAction, state variable not found ", name));
}

}  // namespace intrinsic::icon
