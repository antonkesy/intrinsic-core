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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_HARDWARE_MODULE_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_HARDWARE_MODULE_H_

#include <array>
#include <cstdint>
#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/hardware_module_interface.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_client.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/math/signals/butter_filter2.h"

namespace intrinsic::icon {

// A hardware module implementing Kuka RSI capabilities in the HAL.
class KukaRsiHwModule final : public HardwareModuleInterface {
 public:
  // Constructor for the hardware module, with an optional RsiClient. If
  // specified, the RsiClient will be moved into this class. It will live as
  // long as this class lives. If an RsiClient is not provided, it is
  // constructed.
  explicit KukaRsiHwModule(
      std::unique_ptr<kuka::RealtimeKukaRsiClient> client = nullptr);

  static constexpr char kHwModuleTypeName[] = "KukaRsiHwModule";
  constexpr static int kFrequency = 250;

  absl::Status Init(HardwareModuleInitContext& init_context) override;

  absl::Status Prepare() override;

  RealtimeStatus Activate() override;

  RealtimeStatus Deactivate() override;

  RealtimeStatus Enabled() override;
  RealtimeStatus Disabled() override;

  absl::Status EnableMotion() override;

  absl::Status DisableMotion() override;

  absl::Status ClearFaults() override;

  absl::Status Shutdown() override;

  RealtimeStatus ReadStatus() override;

  // If 'Stop' was called previously, the value from the joint position command
  // interface is ignored and we instead apply the same joint position command
  // as the last time this module was active.
  RealtimeStatus ApplyCommand() override;

  absl::Status ProvideInspectionData(
      intrinsic_proto::icon::v1::HardwareModuleInspectionData& data) override;

 private:
  void ComputeAndUpdateJointVelocities(
      const eigenmath::Vectord<kuka::kKukaNumJoints>& positions,
      uint64_t timestamp_ms);

  std::optional<int64_t> previous_cycle_timestamp_ms_;
  std::optional<eigenmath::Vectord<kuka::kKukaNumJoints>>
      previous_joint_position_state_;
  ButterFilter2<eigenmath::VectorNd> butterworth_joint_velocity_filter_;
  eigenmath::Vectord<kuka::kKukaNumJoints> filtered_joint_velocities_;

  // HAL interface handles.
  MutableHardwareInterfaceHandle<intrinsic_fbs::JointPositionState>
      joint_position_state_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::JointVelocityState>
      joint_velocity_state_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::JointAccelerationState>
      joint_acceleration_state_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::JointTorqueState>
      joint_torque_state_;
  StrictHardwareInterfaceHandle<::intrinsic_fbs::JointPositionCommand>
      joint_position_command_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::DIOStatus>
      digital_input_status_;
  HardwareInterfaceHandle<intrinsic_fbs::DIOCommand> digital_output_command_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::DIOStatus>
      digital_output_status_;
  HardwareInterfaceHandle<intrinsic_fbs::PayloadCommand> payload_command_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::PayloadState> payload_state_;

  // The client used for api calls on low level. This may be used to replace the
  // api calls with a mock implementation. In most cases, this is owned solely
  // by the HardwareModule, but tests may also own the mock.
  std::unique_ptr<kuka::RealtimeKukaRsiClient> client_;

  std::array<double, kuka::kKukaNumJoints> last_position_command_{};
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_HARDWARE_MODULE_H_
