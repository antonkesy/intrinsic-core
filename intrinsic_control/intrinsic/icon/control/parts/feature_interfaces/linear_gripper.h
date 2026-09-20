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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_LINEAR_GRIPPER_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_LINEAR_GRIPPER_H_

#include <optional>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/hal/linear_gripper_part/hal_linear_gripper_part_config.pb.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/gripper.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

// Implementation of the LinearGripper- and SimpleGripperFeatureInterface.
// The implementation wraps a hardware interface allocated on a shared memory
// segment and thus links a hardware module to a part.
class LinearGripperFeature : public HalFeatureInterfaceBase,
                             public LinearGripper,
                             public SimpleGripper {
  using LinearGripperStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::GripperStatus>;
  using LinearGripperCommandHardwareInterface =
      MutableHardwareInterfaceHandle<intrinsic_fbs::GripperCommand>;

 public:
  static absl::StatusOr<LinearGripperFeature> Create(
      LinearGripperStateHardwareInterface
          linear_gripper_state_hardware_interface,
      LinearGripperCommandHardwareInterface
          linear_gripper_command_hardware_interface,
      const intrinsic_proto::icon::HalLinearGripperPartConfig&
          linear_gripper_config);

  LinearGripperFeature(const LinearGripperFeature&) = delete;
  LinearGripperFeature& operator=(const LinearGripperFeature&) = delete;
  LinearGripperFeature(LinearGripperFeature&& other) = default;
  LinearGripperFeature& operator=(LinearGripperFeature&& other) = default;
  ~LinearGripperFeature() override = default;

  // LinearGripper methods.
  // Sets the gripper command to the given width (in m), using the given force
  // (N) and speed (m/s).
  // Force and speed are optional; if not given, the default values from the
  // configuration will be used.
  // Some gripper devices may choose to ignore the force and/or speed parameters
  // if the underlying driver does not support it.
  // Returns an error if the command is not valid.
  RealtimeStatus SetGripperCommand(double width, std::optional<double> force,
                                   std::optional<double> speed) override;
  double GetGripperWidth() const override;

  // SimpleGripper methods.
  // `kGrasp` commands the gripper to move to default_grasp_width with default
  // force and speed.
  // `kRelease` commands the gripper to move to default_release_width with
  // default force and speed.
  // Other commands are ignored.
  RealtimeStatus SetGripperCommand(
      const SimpleGripper::GripperCommand& command) override;

  // Returns the gripper state based on the current width and the default grasp
  // and release configuration.
  // Returns `kGrasped` if width <= default_grasp_width.
  // Returns `kReleased` if width >= default_release_width.
  // Returns `kUnknown` otherwise.
  SimpleGripper::GripperState GetGripperState() const override;

 private:
  LinearGripperFeature(
      LinearGripperStateHardwareInterface
          linear_gripper_state_hardware_interface,
      LinearGripperCommandHardwareInterface
          linear_gripper_command_hardware_interface,
      const intrinsic_proto::icon::HalLinearGripperPartConfig& gripper_config);

  LinearGripperStateHardwareInterface state_interface_;
  LinearGripperCommandHardwareInterface command_interface_;
  intrinsic_proto::icon::HalLinearGripperPartConfig gripper_config_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_LINEAR_GRIPPER_H_
