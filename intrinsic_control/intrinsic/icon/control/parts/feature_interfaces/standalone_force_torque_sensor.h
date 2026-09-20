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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_STANDALONE_FORCE_TORQUE_SENSOR_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_STANDALONE_FORCE_TORQUE_SENSOR_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/hal/force_torque_sensor_part/force_sensor_controller_migration_utils.h"
#include "intrinsic/icon/control/parts/hal/force_torque_sensor_part/hal_force_torque_sensor_part_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

// Implementation of the ForceTorqueSensor feature interface. This feature
// interface implementation exposes StandaloneForceTorqueSensor which only
// exposes the raw (with correct sign convention) wrench values for the FT
// sensor without considering attachment to robot or compensation for attached
// masses.
class StandaloneForceTorqueSensorFeature : public HalFeatureInterfaceBase,
                                           public StandaloneForceTorqueSensor {
  using ForceTorqueStatusHardwareInterface =
      intrinsic::icon::HardwareInterfaceHandle<
          intrinsic_fbs::ForceTorqueStatus>;
  using ForceTorqueCommandHardwareInterface =
      intrinsic::icon::MutableHardwareInterfaceHandle<
          intrinsic_fbs::ForceTorqueCommand>;

 public:
  static absl::StatusOr<std::unique_ptr<StandaloneForceTorqueSensorFeature>>
  Create(const intrinsic_proto::icon::HalForceTorqueSensorPartConfig& config,
         ForceTorqueStatusHardwareInterface force_torque_status_handle,
         ForceTorqueCommandHardwareInterface force_torque_command_handle);

  StandaloneForceTorqueSensorFeature(
      const StandaloneForceTorqueSensorFeature&) = delete;
  StandaloneForceTorqueSensorFeature& operator=(
      const StandaloneForceTorqueSensorFeature&) = delete;
  StandaloneForceTorqueSensorFeature(
      StandaloneForceTorqueSensorFeature&& other) = delete;
  StandaloneForceTorqueSensorFeature& operator=(
      StandaloneForceTorqueSensorFeature&& other) = delete;
  ~StandaloneForceTorqueSensorFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  RealtimeStatus ApplyCommand(
      RealtimePartInterface::ApplyCommandParameters params) override;

  Wrench WrenchAtSensorUncompensated() const override { return wrench_at_ft_; }

  RealtimeStatus Tare(int num_taring_cycles) override;
  RealtimeStatusOr<bool> TareIsDone() const override;

 private:
  struct TaringState {
    bool requested = false;

    // Initialize with true, since taring will never be requested if
    // completed=false.
    bool completed = true;

    // Number of taring cycles that is sent to the FT module when requesting a
    // taring operation.
    int requested_num_taring_cycles = 1;
  };

  Wrench wrench_at_ft_unprocessed_ = Wrench::ZERO;
  Wrench wrench_at_ft_ = Wrench::ZERO;

  TaringState taring_state_;

  StandaloneForceTorqueSensorFeature(
      ForceTorqueStatusHardwareInterface force_torque_status,
      ForceTorqueCommandHardwareInterface force_torque_command,
      int num_acceptable_constant_readings);

  RealtimeStatus ReadWrenchFromDevice();

  ForceTorqueStatusHardwareInterface force_torque_status_handle_;
  ForceTorqueCommandHardwareInterface force_torque_command_handle_;

  // Number of constant readings to accept, this defaults to 10.
  const int num_acceptable_constant_readings_;
  ConstSensorReadingsCounter<eigenmath::Vector6d> constant_readings_counter_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_STANDALONE_FORCE_TORQUE_SENSOR_H_
