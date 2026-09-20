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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_HOMING_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_HOMING_ACTION_H_

#include <stdbool.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/homing_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Action to be used with a Part implementing the Homing Feature
// Interface.
// The action will perform a homing operation based on the provided parameters.
class HomingAction final : public RtclActionInterface {
 public:
  // Creates a HomingAction.
  // While the DS402 standard defines some `homing_method`s, manufacturers tend
  // to additionally provide custom methods (often using negative values), thus
  // we use an int8_t.
  // `slot_id`: The slot ID of the part to control.
  // `drive_name`: The name of the drive to home. `homing_method`: The homing
  // method to use (default: 37).
  // `search_speed`: The speed to use during the search phase (default: 10.0,
  // consult drive manual for units).
  // `creep_speed`: The speed to use during the creep phase (default: 10.0,
  // consult drive manual for units).
  // `acceleration`: The acceleration to use during the homing move (default:
  // 100.0, consult drive manual for units).
  // `offset`: The offset to apply to the final position (default: 0.0, consult
  // drive manual for units). `is_settled_criterion`: The criterion to check if
  // the robot is settled before homing is started.
  // `ignore_is_settled_condition`: Whether or not to ignore the is_settled
  // criterion before homing execution.
  explicit HomingAction(
      RealtimeSlotId slot_id, std::string drive_name, int8_t homing_method,
      float search_speed, float creep_speed, float acceleration, float offset,
      std::unique_ptr<IsSettledCriterion> is_settled_criterion,
      bool ignore_is_settled_condition)
      : slot_id_(slot_id),
        drive_name_(std::move(drive_name)),
        homing_method_(homing_method),
        search_speed_(search_speed),
        creep_speed_(creep_speed),
        acceleration_(acceleration),
        offset_(offset),
        is_settled_criterion_(std::move(is_settled_criterion)),
        ignore_is_settled_condition_(ignore_is_settled_condition) {}

  // Default homing method, homing on current position (new version).
  static constexpr int8_t kDefaultHomingMethod = 37;

  // Creates a HomingAction from the given parameters.
  // Returns:
  //   * `InvalidArgumentError` if `drive_name` is empty, if `homing_method` is
  //     out of range, or if the `IsSettledCriterion` could not be created.
  //   * A `NotFoundError` if the slot `HomingInfo::kSlotName` cannot be found.
  static absl::StatusOr<std::unique_ptr<HomingAction>> Create(
      const HomingInfo::FixedParams& params_proto,
      ActionFactoryContext& context) INTRINSIC_NON_REALTIME_ONLY;

  // Called once when the action becomes active.
  // Returns:
  //   * `OkStatus` if the action was entered successfully (not supposed to
  //     fault).
  RealtimeStatus OnEnter(OnEnterParameters params)
      INTRINSIC_CHECK_REALTIME_SAFE override;

  // Called in every cycle when the action is active.
  // Returns:
  //   * `InternalError` if the slot does not provide the required interfaces,
  //     or if the `IsSettledCriterion` could not be updated.
  //   * `OkStatus` if the sense operation was successful.
  RealtimeStatus Sense(SenseParameters params)
      INTRINSIC_CHECK_REALTIME_SAFE override;

  // Called in every cycle when the action is active.
  // Returns:
  //   * `InternalError` if the slot does not provide the required interfaces or
  //     if the homing command failed.
  //   * `OkStatus` if the control operation was successful.
  RealtimeStatus Control(ControlParameters params)
      INTRINSIC_CHECK_REALTIME_SAFE override;

  // Returns the current state of the action.
  // Returns:
  //   * `NotFound` if the requested state variable is not known.
  //   * The value of the requested state variable otherwise.
  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const INTRINSIC_CHECK_REALTIME_SAFE override;

 private:
  // The ID of the slot for the controlled part.
  RealtimeSlotId slot_id_;

  // State variables.
  // Indicates if the homing is in progress.
  bool is_homing_in_progress_ = false;
  // Indicates if the homing is done.
  bool is_homing_done_ = false;
  // Indicates if the robot is settled.
  bool is_settled_ = false;

  // Homing params.
  // The name of the drive to home.
  const std::string drive_name_;
  // The homing method to use.
  const int8_t homing_method_;
  // The speed to use during the search phase.
  const float search_speed_;
  // The speed to use during the creep phase.
  const float creep_speed_;
  // The acceleration to use during the homing move.
  const float acceleration_;
  // The offset to apply to the final position.
  const float offset_;
  // The criterion to check if the robot is settled before homing is started.
  std::unique_ptr<IsSettledCriterion> is_settled_criterion_;
  // Whether or not to ignore the is_settled criterion before homing execution.
  const bool ignore_is_settled_condition_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_HOMING_ACTION_H_
