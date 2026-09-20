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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_IMU_STATE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_IMU_STATE_H_

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/imu.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

// Implementation of the InertialMeasurementUnit FeatureInterface.
// The implementation wraps a hardware interface allocated on a shared memory
// segment and thus links a hardware module to a part.
class InertialMeasurementUnitStateFeature : public HalFeatureInterfaceBase,
                                            public InertialMeasurementUnit {
  using ImuHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::ImuStatus>;

 public:
  static absl::StatusOr<InertialMeasurementUnitStateFeature> Create(
      ImuHardwareInterface imu_hardware_interface,
      const Pose3d& flange_t_sensor);

  InertialMeasurementUnitStateFeature(
      const InertialMeasurementUnitStateFeature&) = delete;
  InertialMeasurementUnitStateFeature& operator=(
      const InertialMeasurementUnitStateFeature&) = delete;
  InertialMeasurementUnitStateFeature(
      InertialMeasurementUnitStateFeature&& other);
  InertialMeasurementUnitStateFeature& operator=(
      InertialMeasurementUnitStateFeature&& other);
  ~InertialMeasurementUnitStateFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  eigenmath::Vector3d GetSensedLinearAcceleration() const override {
    return sensed_linear_acceleration_;
  }
  eigenmath::Vector3d GetSensedAngularVelocity() const override {
    return sensed_angular_velocity_;
  }
  eigenmath::Quaterniond GetSensedOrientation() const override {
    return sensed_orientation_;
  }
  Pose3d GetPoseInFlangeFrame() const override { return flange_t_sensor_; }

 private:
  InertialMeasurementUnitStateFeature(
      ImuHardwareInterface&& imu_hardware_interface,
      const Pose3d& flange_t_sensor);

  ImuHardwareInterface imu_hardware_interface_;

  eigenmath::Vector3d sensed_linear_acceleration_;
  eigenmath::Vector3d sensed_angular_velocity_;
  eigenmath::Quaterniond sensed_orientation_;
  Pose3d flange_t_sensor_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_IMU_STATE_H_
