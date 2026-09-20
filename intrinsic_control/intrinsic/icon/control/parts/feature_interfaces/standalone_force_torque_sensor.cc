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

#include "intrinsic/icon/control/parts/feature_interfaces/standalone_force_torque_sensor.h"

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/hal/force_torque_sensor_part/force_sensor_controller_migration_utils.h"
#include "intrinsic/icon/control/parts/hal/force_torque_sensor_part/hal_force_torque_sensor_part_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/flatbuffers/transform_view.h"
#include "intrinsic/icon/hal/interfaces/force_sensor.fbs.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/force_torque_utils.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/to_fixed_string.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

namespace {
// This defines the default number of constant unprocessed FT readings before an
// error is returned. This is only used if no value is defined in the proto
// config.
constexpr int kDefaultMaxNumberConstantFTReadings = 10;

}  // namespace

StandaloneForceTorqueSensorFeature::StandaloneForceTorqueSensorFeature(
    ForceTorqueStatusHardwareInterface force_torque_status,
    ForceTorqueCommandHardwareInterface force_torque_command,
    int num_acceptable_constant_readings)
    : force_torque_status_handle_(std::move(force_torque_status)),
      force_torque_command_handle_(std::move(force_torque_command)),
      num_acceptable_constant_readings_(num_acceptable_constant_readings) {}

// static
absl::StatusOr<std::unique_ptr<StandaloneForceTorqueSensorFeature>>
StandaloneForceTorqueSensorFeature::Create(
    const intrinsic_proto::icon::HalForceTorqueSensorPartConfig& config,
    ForceTorqueStatusHardwareInterface force_torque_status_handle,
    ForceTorqueCommandHardwareInterface force_torque_command_handle) {
  int num_acceptable_constant_readings = kDefaultMaxNumberConstantFTReadings;
  if (config.has_num_acceptable_constant_readings()) {
    num_acceptable_constant_readings =
        config.num_acceptable_constant_readings();
  }

  return absl::WrapUnique(new StandaloneForceTorqueSensorFeature(
      std::move(force_torque_status_handle),
      std::move(force_torque_command_handle),
      num_acceptable_constant_readings));
}

RealtimeStatus StandaloneForceTorqueSensorFeature::ReadWrenchFromDevice() {
  wrench_at_ft_unprocessed_ = intrinsic_fbs::ViewAs<eigenmath::Vector6d>(
      force_torque_status_handle_->wrench());
  return OkStatus();
}
RealtimeStatus StandaloneForceTorqueSensorFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  // Fail in case the force-sensor-controller reports non-ok statuses. This
  // will allow ICON to fail cleanly in case of errors at device level.
  if (force_torque_status_handle_->status_code() !=
      intrinsic_fbs::ForceSensorStatusCode::Ok) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << force_torque_status_handle_->status_code();
    return icon::InternalError(RealtimeStatus::StrCat(
        intrinsic_fbs::ToFixedString(
            force_torque_status_handle_->status_code()),
        " (0x", absl::Hex(force_torque_status_handle_->raw_status_code()),
        ")"));
  }

  Wrench wrench_at_ft = Wrench::ZERO;

  // Store the "taring completed" state if completed.
  taring_state_.completed = force_torque_status_handle_->retare_completed();

  // Check if we are still receiving data from the force sensor, but only if
  // no taring operation is ongoing. During taring all values are
  // meaningless and therefore need to be disregarded.
  if (taring_state_.completed && force_torque_status_handle_->enabled()) {
    constant_readings_counter_.AddReading(
        wrench_at_ft_unprocessed_,  // still holds values from
                                    // previous read cycle!
        intrinsic_fbs::ViewAs<eigenmath::Vector6d>(
            force_torque_status_handle_->wrench())  // new measurement.
    );

    if ((constant_readings_counter_.num_constant_readings() >
         num_acceptable_constant_readings_)) {
      return InternalError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
          "Force sensor reporting constant values: count= ",
          constant_readings_counter_.num_constant_readings(), ", force= [",
          eigenmath::ToFixedString(-intrinsic_fbs::ViewAs<eigenmath::Vector6d>(
              force_torque_status_handle_->wrench())),
          ". This indicates a possible FT-sensor failure, will raise an "
          "error in force_sensor_status."));
    }
  }

  // This updates wrench_at_ft_unprocessed to the current reading.
  INTRINSIC_RT_RETURN_IF_ERROR(ReadWrenchFromDevice());

  wrench_at_ft = -wrench_at_ft_unprocessed_;

  // Copy into variables made available in feature interfaces.
  wrench_at_ft_ = wrench_at_ft;

  return OkStatus();
}

RealtimeStatus StandaloneForceTorqueSensorFeature::ApplyCommand(
    RealtimePartInterface::ApplyCommandParameters params) {
  // Record the current F/T due to support_mass as bias when taring. Will
  // be subtracted later during load compensation.
  if (taring_state_.requested) {
    force_torque_command_handle_->mutate_retare(true);
    force_torque_command_handle_->mutate_num_taring_cycles(
        taring_state_.requested_num_taring_cycles);
    // Reset to avoid triggering taring multiple times.
    taring_state_.requested = false;
    taring_state_.completed = false;
  } else {
    force_torque_command_handle_->mutate_retare(false);
  }

  force_torque_command_handle_.UpdatedAt(Clock::now());

  return OkStatus();
}

RealtimeStatus StandaloneForceTorqueSensorFeature::Tare(int num_taring_cycles) {
  // Only allow a new taring request when the ongoing taring cycle has been
  // completed.
  if (taring_state_.completed) {
    taring_state_.requested = true;
    taring_state_.completed = false;
    taring_state_.requested_num_taring_cycles = num_taring_cycles;
  }
  return OkStatus();
}

RealtimeStatusOr<bool> StandaloneForceTorqueSensorFeature::TareIsDone() const {
  return taring_state_.completed;
}

}  // namespace intrinsic::icon
