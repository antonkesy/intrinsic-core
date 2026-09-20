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

#include "intrinsic/icon/control/actions/homing_action.h"

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/homing_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
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
absl::StatusOr<std::unique_ptr<HomingAction>> HomingAction::Create(
    const HomingInfo::FixedParams& params_proto,
    ActionFactoryContext& context) {
  INTR_ASSIGN_OR_RETURN(SlotInfo slot_info,
                        context.GetSlotInfo(HomingInfo::kSlotName));
  std::string drive_name = params_proto.drive_name();
  if (drive_name.empty()) {
    return absl::InvalidArgumentError("drive_name is empty");
  }
  int8_t homing_method = HomingAction::kDefaultHomingMethod;
  if (params_proto.has_homing_method()) {
    // The homing method is an int8_t, but the proto field is an int32_t.
    // We need to check that the value is within the range of an int8_t.
    using HomingMethod = decltype(homing_method);
    if (params_proto.homing_method() <
            std::numeric_limits<HomingMethod>::min() ||
        params_proto.homing_method() >
            std::numeric_limits<HomingMethod>::max()) {
      return absl::InvalidArgumentError(
          absl::StrCat("homing_method is out of range [",
                       std::numeric_limits<HomingMethod>::min(), ", ",
                       std::numeric_limits<HomingMethod>::max(), "]"));
    }
    homing_method = static_cast<HomingMethod>(params_proto.homing_method());
  }
  float search_speed = 10.0;
  if (params_proto.has_search_speed()) {
    search_speed = params_proto.search_speed();
  }
  float creep_speed = 10.0;
  if (params_proto.has_creep_speed()) {
    creep_speed = params_proto.creep_speed();
  }
  float acceleration = 100.0;
  if (params_proto.has_acceleration()) {
    acceleration = params_proto.acceleration();
  }
  float offset = 0.0;
  if (params_proto.has_offset()) {
    offset = params_proto.offset();
  }
  bool ignore_is_settled_condition = false;
  if (params_proto.has_ignore_is_settled_condition()) {
    ignore_is_settled_condition = params_proto.ignore_is_settled_condition();
  }
  INTR_ASSIGN_OR_RETURN(
      auto is_settled_criterion,
      IsSettledCriterion::Create(context.ServerConfig().frequency_hz()));

  return std::make_unique<HomingAction>(
      slot_info.slot_id, std::move(drive_name), homing_method, search_speed,
      creep_speed, acceleration, offset, std::move(is_settled_criterion),
      ignore_is_settled_condition);
}

RealtimeStatus HomingAction::OnEnter(OnEnterParameters params) {
  is_homing_done_ = false;
  is_homing_in_progress_ = false;
  is_settled_ = false;
  // Initialize the criterion to determine the settled state.
  INTRINSIC_RT_RETURN_IF_ERROR(is_settled_criterion_->Initialize());
  return OkStatus();
}

RealtimeStatus HomingAction::Sense(SenseParameters params) {
  const Homing* const homing_interface =
      params.slot_map.GetInterfaceForSlot<Homing>(slot_id_);
  if (homing_interface == nullptr) {
    return InternalError("Slot doesn't have a Homing interface.");
  }
  if (ignore_is_settled_condition_) {
    if (!is_settled_) {
      INTRINSIC_RT_LOG_THROTTLED(INFO)
          << "Ignoring the is_settled criterion as configured.";
    }
    is_settled_ = true;
  } else {
    const JointVelocityEstimator* const velocity_estimator =
        params.slot_map.GetInterfaceForSlot<JointVelocityEstimator>(slot_id_);
    if (velocity_estimator == nullptr) {
      return InternalError(
          "Slot doesn't have a JointVelocityEstimator interface.");
    }
    auto velocity = velocity_estimator->GetVelocityEstimate().velocity;
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        is_settled_,
        is_settled_criterion_->Update(velocity,
                                      /*commanded_joint_velocities=*/velocity,
                                      /*has_trajectory_ended=*/
                                      true));
    if (!is_settled_) {
      INTRINSIC_RT_LOG_THROTTLED(INFO)
          << "Homing not started. Waiting for robot to settle.";
    }
  }

  is_homing_in_progress_ =
      homing_interface->IsHoming(absl::string_view(drive_name_));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      bool is_homing_done,
      homing_interface->IsHomingDone(absl::string_view(drive_name_)));
  is_homing_done_ |= is_homing_done;

  if (is_homing_done_) {
    is_homing_in_progress_ = false;
    INTRINSIC_RT_LOG_FIRST(INFO) << "Homing completed in HomingAction. "
                                    "Expecting Action to be terminated.";
  }

  return OkStatus();
}

RealtimeStatus HomingAction::Control(ControlParameters params) {
  Homing* const homing_interface =
      params.slot_map.GetMutableInterfaceForSlot<Homing>(slot_id_);
  if (homing_interface == nullptr) {
    return InternalError("Slot doesn't have a Homing Interface.");
  }

  if (is_settled_ && !is_homing_in_progress_ && !is_homing_done_) {
    return homing_interface->CommandHoming(
        absl::string_view(drive_name_), homing_method_, search_speed_,
        creep_speed_, acceleration_, offset_);
  }
  return OkStatus();
}

RealtimeStatusOr<StateVariableValue> HomingAction::GetStateVariable(
    absl::string_view name) const {
  if (name == kIsDone) {
    return StateVariableValue(is_homing_done_);
  }
  return NotFoundError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
      HomingInfo::kActionTypeName,
      " does not expose any state variables. Requested: ", name));
}

}  // namespace intrinsic::icon
