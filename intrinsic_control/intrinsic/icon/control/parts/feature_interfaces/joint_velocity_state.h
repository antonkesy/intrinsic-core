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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_VELOCITY_STATE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_VELOCITY_STATE_H_

#include <memory>
#include <optional>
#include <variant>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/signals/butter_filter2.h"

namespace intrinsic::icon {

// Implementation of the JointVelocityStateFeatureInterface.
// The implementation wraps a hardware interface allocated on a shared memory
// segment and thus links a hardware module to a part.
class JointVelocityStateFeature : public HalFeatureInterfaceBase,
                                  public JointVelocityEstimator {
  using JointVelocityStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointVelocityState>;
  using JointPositionStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointPositionState>;

 public:
  // If JointVelocityStateHardwareInterface is provided, the velocity state is
  // read directly from the hardware interface. If
  // JointPositionStateHardwareInterface is provided, the velocity state is
  // calculated from the position state via finite differences. In this case the
  // control frequency must be provided. If a velocity filter cutoff frequency
  // is additionally provided, a low-pass filter is applied to the velocity
  // state, both for measured velocity and calculated velocity from position.
  static absl::StatusOr<JointVelocityStateFeature> Create(
      std::variant<JointVelocityStateHardwareInterface,
                   JointPositionStateHardwareInterface>
          hardware_interface,
      std::optional<double> control_frequency_hz = std::nullopt,
      std::optional<double> velocity_filter_cutoff_frequency = std::nullopt);

  JointVelocityStateFeature(const JointVelocityStateFeature&) = delete;
  JointVelocityStateFeature& operator=(const JointVelocityStateFeature&) =
      delete;
  JointVelocityStateFeature(JointVelocityStateFeature&& other) = default;
  JointVelocityStateFeature& operator=(JointVelocityStateFeature&& other) =
      default;
  ~JointVelocityStateFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;
  RealtimeStatus Reset() override;
  JointStateV GetVelocityEstimate() const override;

 private:
  JointVelocityStateFeature(
      std::variant<JointVelocityStateHardwareInterface,
                   JointPositionStateHardwareInterface>
          hardware_interface,
      JointStateV sensed_velocity,
      std::optional<double> control_frequency_hz = std::nullopt,
      std::unique_ptr<ButterFilter2<eigenmath::VectorNd>>
          butterworth_joint_velocity_filter = nullptr);

  std::variant<JointVelocityStateHardwareInterface,
               JointPositionStateHardwareInterface>
      hardware_interface_;
  JointStateV sensed_velocity_;
  std::optional<eigenmath::VectorNd> previous_position_;
  std::optional<double> control_frequency_hz_;
  std::unique_ptr<ButterFilter2<eigenmath::VectorNd>>
      butterworth_joint_velocity_filter_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_VELOCITY_STATE_H_
