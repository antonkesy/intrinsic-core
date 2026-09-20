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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_FORCE_TORQUE_SENSOR_TARE_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_FORCE_TORQUE_SENSOR_TARE_ACTION_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/tare_force_torque_sensor_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Action that issues a Tare() command on a part implementing the
// ForceTorqueSensor interface.
// The StateVariable 'done' is set to true when the taring command was sent to
// the part. See
// intrinsic/icon/actions/tare_force_torque_sensor_info.h
class ForceTorqueSensorTareAction final : public RtclActionInterface {
 public:
  explicit ForceTorqueSensorTareAction(RealtimeSlotId slot_id,
                                       int num_taring_cycles)
      : slot_id_(slot_id), num_taring_cycles_(num_taring_cycles) {}

  static absl::StatusOr<std::unique_ptr<ForceTorqueSensorTareAction>> Create(
      TareForceTorqueSensorInfo::FixedParams params_proto,
      ActionFactoryContext& context) INTRINSIC_NON_REALTIME_ONLY;

  RealtimeStatus OnEnter(OnEnterParameters params)
      INTRINSIC_CHECK_REALTIME_SAFE override;

  RealtimeStatus Sense(SenseParameters params)
      INTRINSIC_CHECK_REALTIME_SAFE override;

  RealtimeStatus Control(ControlParameters params)
      INTRINSIC_CHECK_REALTIME_SAFE override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const INTRINSIC_CHECK_REALTIME_SAFE override;

 private:
  RealtimeSlotId slot_id_;
  int num_taring_cycles_;

  // Taring may take a couple control cycles, so we distinguish between request
  // and completion.
  bool tare_requested_ = false;
  bool tare_completed_ = false;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_FORCE_TORQUE_SENSOR_TARE_ACTION_H_
