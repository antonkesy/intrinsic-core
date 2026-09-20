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

#include "intrinsic/icon/control/parts/feature_interfaces/imu_state.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

absl::StatusOr<InertialMeasurementUnitStateFeature>
InertialMeasurementUnitStateFeature ::Create(
    ImuHardwareInterface imu_hardware_interface,
    const Pose3d& flange_t_sensor) {
  if (*imu_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for InertialMeasurementUnitStateFeature is "
        "not initialized.");
  }
  return InertialMeasurementUnitStateFeature(std::move(imu_hardware_interface),
                                             flange_t_sensor);
}

InertialMeasurementUnitStateFeature::InertialMeasurementUnitStateFeature(
    InertialMeasurementUnitStateFeature&& other)
    : imu_hardware_interface_(std::move(other.imu_hardware_interface_)),
      sensed_linear_acceleration_(std::exchange(
          other.sensed_linear_acceleration_, eigenmath::Vector3d::Zero())),
      sensed_angular_velocity_(std::exchange(other.sensed_angular_velocity_,
                                             eigenmath::Vector3d::Zero())),
      sensed_orientation_(std::exchange(other.sensed_orientation_,
                                        eigenmath::Quaterniond::Identity())),
      flange_t_sensor_(
          std::exchange(other.flange_t_sensor_, Pose3d::Identity())) {}

InertialMeasurementUnitStateFeature&
InertialMeasurementUnitStateFeature::operator=(
    InertialMeasurementUnitStateFeature&& other) {
  imu_hardware_interface_ = std::move(other.imu_hardware_interface_);
  sensed_linear_acceleration_ = std::exchange(other.sensed_linear_acceleration_,
                                              eigenmath::Vector3d::Zero());
  sensed_angular_velocity_ = std::exchange(other.sensed_angular_velocity_,
                                           eigenmath::Vector3d::Zero());
  sensed_orientation_ = std::exchange(other.sensed_orientation_,
                                      eigenmath::Quaterniond::Identity());
  flange_t_sensor_ = std::exchange(other.flange_t_sensor_, Pose3d::Identity());
  return *this;
}

InertialMeasurementUnitStateFeature::InertialMeasurementUnitStateFeature(
    ImuHardwareInterface&& imu_hardware_interface,
    const Pose3d& flange_t_sensor)
    : imu_hardware_interface_(std::move(imu_hardware_interface)),
      sensed_linear_acceleration_(eigenmath::Vector3d::Zero()),
      sensed_angular_velocity_(eigenmath::Vector3d::Zero()),
      sensed_orientation_(eigenmath::Quaterniond::Identity()),
      flange_t_sensor_(flange_t_sensor) {}

RealtimeStatus InertialMeasurementUnitStateFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  sensed_linear_acceleration_.x() =
      imu_hardware_interface_->linear_acceleration()->x();
  sensed_linear_acceleration_.y() =
      imu_hardware_interface_->linear_acceleration()->y();
  sensed_linear_acceleration_.z() =
      imu_hardware_interface_->linear_acceleration()->z();

  sensed_angular_velocity_.x() =
      imu_hardware_interface_->angular_velocity()->x();
  sensed_angular_velocity_.y() =
      imu_hardware_interface_->angular_velocity()->y();
  sensed_angular_velocity_.z() =
      imu_hardware_interface_->angular_velocity()->z();

  sensed_orientation_.x() = imu_hardware_interface_->orientation()->qx();
  sensed_orientation_.y() = imu_hardware_interface_->orientation()->qy();
  sensed_orientation_.z() = imu_hardware_interface_->orientation()->qz();
  sensed_orientation_.w() = imu_hardware_interface_->orientation()->qw();

  return OkStatus();
}

}  // namespace intrinsic::icon
