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

#include "intrinsic/icon/control/parts/fake_feature_interfaces.h"

#include <stddef.h>

#include <algorithm>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/joint_acceleration_command.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/io_block.h"
#include "intrinsic/icon/control/parts/realtime_robot_payload.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

FakeJointPosition::FakeJointPosition(int ndof) {
  setpoints_ =
      JointPositionCommand::Create(eigenmath::VectorNd::Zero(ndof)).value();
}

RealtimeStatus FakeJointPosition::SetPositionSetpoints(
    const JointPositionCommand& setpoints) {
  setpoints_ = setpoints;
  return next_status_;
}

JointPositionCommand FakeJointPosition::PreviousPositionSetpoints() const {
  return setpoints_;
}

void FakeJointPosition::SetReturnStatus(RealtimeStatus s) { next_status_ = s; }

RealtimeStatus FakeJointVelocity::SetVelocitySetpoints(
    const eigenmath::VectorNd& setpoints) {
  setpoints_ = setpoints;
  return next_status_;
}

FakeJointVelocity::FakeJointVelocity(int ndof)
    : setpoints_(eigenmath::VectorNd::Zero(ndof)) {}

void FakeJointVelocity::SetReturnStatus(RealtimeStatus s) { next_status_ = s; }

const eigenmath::VectorNd& FakeJointVelocity::GetVelocitySetpoints() const {
  return setpoints_;
}

FakeJointAcceleration::FakeJointAcceleration(int ndof) {
  RealtimeStatusOr<JointAccelerationCommand> setpoints_or =
      JointAccelerationCommand::Create(eigenmath::VectorNd::Zero(ndof));
  CHECK_OK(setpoints_or.status());
  setpoints_ = setpoints_or.value();
}

RealtimeStatus FakeJointAcceleration::SetAccelerationSetpoints(
    const JointAccelerationCommand& setpoints) {
  setpoints_ = setpoints;
  return next_status_;
}

JointAccelerationCommand FakeJointAcceleration::PreviousAccelerationSetpoints()
    const {
  return setpoints_;
}

void FakeJointAcceleration::SetReturnStatus(RealtimeStatus s) {
  next_status_ = s;
}

FakeJointPositionSensor::FakeJointPositionSensor(int ndof)
    : state_(eigenmath::VectorNd::Zero(ndof)) {}

void FakeJointPositionSensor::SetSensedPosition(const JointStateP& state) {
  state_ = state;
}

JointStateP FakeJointPositionSensor::GetSensedPosition() const {
  return state_;
}

FakeJointVelocityEstimator::FakeJointVelocityEstimator(int ndof)
    : state_(eigenmath::VectorNd::Zero(ndof)) {}

void FakeJointVelocityEstimator::SetVelocityEstimate(const JointStateV& state) {
  state_ = state;
}

JointStateV FakeJointVelocityEstimator::GetVelocityEstimate() const {
  return state_;
}

FakeJointAccelerationEstimator::FakeJointAccelerationEstimator(int ndof)
    : state_(eigenmath::VectorNd::Zero(ndof)) {}

void FakeJointAccelerationEstimator::SetAccelerationEstimate(
    const JointStateA& state) {
  state_ = state;
}

JointStateA FakeJointAccelerationEstimator::GetAccelerationEstimate() const {
  return state_;
}

FakeJointLimits::FakeJointLimits(JointLimits application_limits,
                                 JointLimits system_limits)
    : application_limits_(std::move(application_limits)),
      system_limits_(std::move(system_limits)) {}

FakeJointLimits::FakeJointLimits(
    const JointLimits& application_and_system_limits)
    : application_limits_(application_and_system_limits),
      system_limits_(application_and_system_limits) {}

void FakeJointLimits::SetApplicationAndSystemLimits(
    const JointLimits& application_and_system_limits) {
  application_limits_ = application_and_system_limits;
  system_limits_ = application_and_system_limits;
}

void FakeJointLimits::SetApplicationLimits(
    const JointLimits& application_limits) {
  application_limits_ = application_limits;
}

void FakeJointLimits::SetSystemLimits(const JointLimits& system_limits) {
  system_limits_ = system_limits;
}

void FakeJointLimits::SetJointAccelerationLimitsFromDynamics(
    const std::optional<
        JointLimitsInterface::JointAccelerationLimitsFromDynamics>&
        joint_acceleration_limits_from_dynamics) {
  joint_acceleration_limits_from_dynamics_ =
      joint_acceleration_limits_from_dynamics;
}

JointLimits FakeJointLimits::GetApplicationLimits() const {
  return application_limits_;
}

JointLimits FakeJointLimits::GetSystemLimits() const { return system_limits_; }

icon::RealtimeStatusOr<
    std::optional<JointLimitsInterface::JointAccelerationLimitsFromDynamics>>
FakeJointLimits::GetJointAccelerationLimitsFromDynamics() const {
  std::optional<JointLimitsInterface::JointAccelerationLimitsFromDynamics>
      joint_acceleration_limits_from_dynamics =
          joint_acceleration_limits_from_dynamics_;
  return joint_acceleration_limits_from_dynamics;
}

FakeCartesianLimits::FakeCartesianLimits(const CartesianLimits& default_limits)
    : default_limits_(default_limits) {}

CartesianLimits FakeCartesianLimits::GetDefaultCartesianLimits() const {
  return default_limits_;
}

void FakeCartesianLimits::SetDefaultCartesianLimits(
    const CartesianLimits& default_limits) {
  default_limits_ = default_limits;
}

FakeRangeFinder::FakeRangeFinder(double fake_dist, Pose3d fake_pose)
    : fake_distance_m_(fake_dist), fake_pose_(std::move(fake_pose)) {}

double FakeRangeFinder::GetSensedDistance() const { return fake_distance_m_; }

bool FakeRangeFinder::IsMeasurementValid() const {
  return fake_measurement_valid_;
}

void FakeRangeFinder::SetMeasurementValid(bool valid) {
  fake_measurement_valid_ = valid;
}

void FakeRangeFinder::SetSensedDistance(double distance) {
  fake_distance_m_ = distance;
}

FakeInertialMeasurementUnit::FakeInertialMeasurementUnit(
    eigenmath::Vector3d sensed_linear_acceleration,
    eigenmath::Vector3d sensed_angular_velocity,
    eigenmath::Quaterniond sensed_orientation, Pose3d fake_pose)
    : sensed_linear_acceleration_(sensed_linear_acceleration),
      sensed_angular_velocity_(sensed_angular_velocity),
      sensed_orientation_(sensed_orientation),
      fake_pose_(std::move(fake_pose)) {}

eigenmath::Vector3d FakeInertialMeasurementUnit::GetSensedLinearAcceleration()
    const {
  return sensed_linear_acceleration_;
}

eigenmath::Vector3d FakeInertialMeasurementUnit::GetSensedAngularVelocity()
    const {
  return sensed_angular_velocity_;
}

eigenmath::Quaterniond FakeInertialMeasurementUnit::GetSensedOrientation()
    const {
  return sensed_orientation_;
}

Pose3d FakeInertialMeasurementUnit::GetPoseInFlangeFrame() const {
  return fake_pose_;
}

void FakeInertialMeasurementUnit::SetSensedLinearAcceleration(
    const eigenmath::Vector3d& linear_acceleration) {
  sensed_linear_acceleration_ = linear_acceleration;
}

void FakeInertialMeasurementUnit::SetSensedAngularVelocity(
    const eigenmath::Vector3d& angular_velocity) {
  sensed_angular_velocity_ = angular_velocity;
}

void FakeInertialMeasurementUnit::SetSensedOrientation(
    const eigenmath::Quaterniond& orientation) {
  sensed_orientation_ = orientation;
}

Pose3d FakeRangeFinder::GetPoseInTCPFrame() const { return fake_pose_; }

RealtimeStatus FakeGripper::SetGripperCommand(const GripperCommand& command) {
  command_ = command;
  return next_status_;
}

void FakeGripper::SetGripperState(const GripperState& state) { state_ = state; }

SimpleGripper::GripperState FakeGripper::GetGripperState() const {
  return state_;
}

SimpleGripper::GripperCommand FakeGripper::GetGripperCommand() {
  return command_;
}

void FakeGripper::SetReturnStatus(RealtimeStatus s) { next_status_ = s; }

RealtimeStatus FakeLinearGripper::SetGripperCommand(
    double width, std::optional<double> force, std::optional<double> speed) {
  commanded_width_ = width;
  commanded_force_ = force;
  commanded_speed_ = speed;
  return next_status_;
}

double FakeLinearGripper::GetGripperWidth() const { return sensed_width_; }

void FakeLinearGripper::SetGripperWidth(double width) { sensed_width_ = width; }

std::optional<double> FakeLinearGripper::GetCommandedWidth() const {
  return commanded_width_;
}

std::optional<double> FakeLinearGripper::GetCommandedForce() const {
  return commanded_force_;
}

std::optional<double> FakeLinearGripper::GetCommandedSpeed() const {
  return commanded_speed_;
}

void FakeLinearGripper::SetReturnStatus(RealtimeStatus s) { next_status_ = s; }

FakeJointTorque::FakeJointTorque(int ndof)
    : setpoints_(eigenmath::VectorNd::Zero(ndof)) {}

RealtimeStatus FakeJointTorque::SetTorqueSetpoints(
    const eigenmath::VectorNd& setpoints) {
  setpoints_ = setpoints;
  return next_status_;
}

eigenmath::VectorNd FakeJointTorque::PreviousTorqueSetpoints() const {
  return setpoints_;
}

void FakeJointTorque::SetReturnStatus(RealtimeStatus s) { next_status_ = s; }

const eigenmath::VectorNd& FakeJointTorque::GetTorqueSetpoints() const {
  return setpoints_;
}

FakeJointTorqueSensor::FakeJointTorqueSensor(int ndof)
    : state_(eigenmath::VectorNd::Zero(ndof)) {}

void FakeJointTorqueSensor::SetSensedTorque(const JointStateT& state) {
  state_ = state;
}

JointStateT FakeJointTorqueSensor::GetSensedTorque() const { return state_; }

void FakeForceTorqueSensor::SetReturnStatus(RealtimeStatus s) {
  next_status_ = s;
}

RealtimeStatus FakeForceTorqueSensor::Tare(int num_taring_cycles) {
  tare_requested_ = true;
  return next_status_;
}

Wrench FakeForceTorqueSensor::PostSensorDynamicLoadAtSensor() const {
  return post_sensor_dynamic_load_at_ft_;
}

Wrench FakeForceTorqueSensor::PostSensorDynamicLoadAtTip() const {
  return post_sensor_dynamic_load_at_tip_;
}

void FakeForceTorqueSensor::SetPostSensorDynamicLoadAtSensor(const Wrench& w) {
  post_sensor_dynamic_load_at_ft_ = w;
}

void FakeForceTorqueSensor::SetPostSensorDynamicLoadAtTip(const Wrench& w) {
  post_sensor_dynamic_load_at_tip_ = w;
}

void FakeStandaloneForceTorqueSensor::SetReturnStatus(RealtimeStatus s) {
  next_status_ = s;
}

RealtimeStatus FakeStandaloneForceTorqueSensor::Tare(int num_taring_cycles) {
  tare_requested_ = true;
  return next_status_;
}

FakeADIO::FakeADIO(FakeADIOState initial_state)
    : state_(std::move(initial_state)) {
  for (const auto& [name, value] : state_.analog_inputs) {
    analog_input_names_.emplace_back(name);

    std::vector<std::string> signal_names;
    if (state_.analog_input_signal_names.contains(name)) {
      CHECK_EQ(state_.analog_input_signal_names[name].size(),
               value.Values().size());
      signal_names = state_.analog_input_signal_names[name];
    } else {
      for (size_t i = 0; i < value.Values().size(); ++i) {
        signal_names.push_back(absl::StrCat(i));
      }
    }
    analog_input_signal_names_[name] = signal_names;
  }
  for (const auto& [name, value] : state_.analog_outputs) {
    analog_output_names_.emplace_back(name);

    std::vector<std::string> signal_names;
    if (state_.analog_output_signal_names.contains(name)) {
      CHECK_EQ(state_.analog_output_signal_names[name].size(),
               value.Values().size());
      signal_names = state_.analog_output_signal_names[name];
    } else {
      for (size_t i = 0; i < value.Values().size(); ++i) {
        signal_names.push_back(absl::StrCat(i));
      }
    }
    analog_output_signal_names_[name] = signal_names;
  }
  for (const auto& [name, value] : state_.digital_inputs) {
    digital_input_names_.emplace_back(name);

    std::vector<std::string> signal_names;
    if (state_.digital_input_signal_names.contains(name)) {
      CHECK_EQ(state_.digital_input_signal_names[name].size(),
               value.Values().size());
      signal_names = state_.digital_input_signal_names[name];
    } else {
      for (size_t i = 0; i < value.Values().size(); ++i) {
        signal_names.push_back(absl::StrCat(i));
      }
    }
    digital_input_signal_names_[name] = signal_names;
  }
  for (const auto& [name, value] : state_.digital_outputs) {
    digital_output_names_.emplace_back(name);

    std::vector<std::string> signal_names;
    if (state_.digital_output_signal_names.contains(name)) {
      CHECK_EQ(state_.digital_output_signal_names[name].size(),
               value.Values().size());
      signal_names = state_.digital_output_signal_names[name];
    } else {
      for (size_t i = 0; i < value.Values().size(); ++i) {
        signal_names.push_back(absl::StrCat(i));
      }
    }
    digital_output_signal_names_[name] = signal_names;
  }

  // Sort names in default order for status export and nicer access .
  std::sort(std::begin(analog_input_names_), std::end(analog_input_names_));
  std::sort(std::begin(analog_output_names_), std::end(analog_output_names_));
  std::sort(std::begin(digital_input_names_), std::end(digital_input_names_));
  std::sort(std::begin(digital_output_names_), std::end(digital_output_names_));
}

absl::Span<const std::string> FakeADIO::AnalogInputBlockNames() const {
  return analog_input_names_;
}

absl::Span<const std::string> FakeADIO::AnalogOutputBlockNames() const {
  return analog_output_names_;
}

absl::Span<const std::string> FakeADIO::DigitalInputBlockNames() const {
  return digital_input_names_;
}

absl::Span<const std::string> FakeADIO::DigitalOutputBlockNames() const {
  return digital_output_names_;
}

absl::Span<const std::string> FakeADIO::DigitalInputSignalNames(
    absl::string_view block_name) const {
  auto it = digital_input_signal_names_.find(block_name);
  if (it == digital_input_signal_names_.end()) {
    return {};
  }
  return it->second;
}

absl::Span<const std::string> FakeADIO::DigitalOutputSignalNames(
    absl::string_view block_name) const {
  auto it = digital_output_signal_names_.find(block_name);
  if (it == digital_output_signal_names_.end()) {
    return {};
  }
  return it->second;
}

absl::Span<const std::string> FakeADIO::AnalogInputSignalNames(
    absl::string_view block_name) const {
  auto it = analog_input_signal_names_.find(block_name);
  if (it == analog_input_signal_names_.end()) {
    return {};
  }
  return it->second;
}

absl::Span<const std::string> FakeADIO::AnalogOutputSignalNames(
    absl::string_view block_name) const {
  auto it = analog_output_signal_names_.find(block_name);
  if (it == analog_output_signal_names_.end()) {
    return {};
  }
  return it->second;
}

const AnalogBlock* FakeADIO::AnalogInputBlock(absl::string_view name) const {
  auto it = state_.analog_inputs.find(name);
  if (it == state_.analog_inputs.end()) return nullptr;
  return &it->second;
}

AnalogBlock* FakeADIO::MutableAnalogInputBlock(absl::string_view name) {
  auto it = state_.analog_inputs.find(name);
  if (it == state_.analog_inputs.end()) return nullptr;
  return &it->second;
}

AnalogBlock* FakeADIO::MutableAnalogOutputBlock(absl::string_view name) {
  auto it = state_.analog_outputs.find(name);
  if (it == state_.analog_outputs.end()) return nullptr;
  return &it->second;
}

const DioBlock* FakeADIO::DigitalInputBlock(absl::string_view name) const {
  auto it = state_.digital_inputs.find(name);
  if (it == state_.digital_inputs.end()) return nullptr;
  return &it->second;
}

DioBlock* FakeADIO::MutableDigitalInputBlock(absl::string_view name) {
  auto it = state_.digital_inputs.find(name);
  if (it == state_.digital_inputs.end()) return nullptr;
  return &it->second;
}

DioBlock* FakeADIO::MutableDigitalOutputBlock(absl::string_view name) {
  auto it = state_.digital_outputs.find(name);
  if (it == state_.digital_outputs.end()) return nullptr;
  return &it->second;
}

ADIO::ADIOState FakeADIO::GetADIOState() const {
  ADIOState state;
  state.analog_input_block_names = AnalogInputBlockNames();
  state.analog_output_block_names = AnalogOutputBlockNames();
  state.digital_input_block_names = DigitalInputBlockNames();
  state.digital_output_block_names = DigitalOutputBlockNames();
  // Fill the status blocks in the order of the block_names.
  for (const auto& block_name : AnalogInputBlockNames()) {
    auto iter = state_.analog_inputs.find(block_name);
    if (iter != state_.analog_inputs.end()) {
      state.analog_inputs.emplace_back(iter->second);
    }
  }
  for (const auto& block_name : AnalogOutputBlockNames()) {
    auto iter = state_.analog_outputs.find(block_name);
    if (iter != state_.analog_outputs.end()) {
      state.analog_outputs.emplace_back(iter->second);
    }
  }
  for (const auto& block_name : DigitalInputBlockNames()) {
    auto iter = state_.digital_inputs.find(block_name);
    if (iter != state_.digital_inputs.end()) {
      state.digital_inputs.emplace_back(iter->second);
    }
  }
  for (const auto& block_name : DigitalOutputBlockNames()) {
    auto iter = state_.digital_outputs.find(block_name);
    if (iter != state_.digital_outputs.end()) {
      state.digital_outputs.emplace_back(iter->second);
    }
  }
  return state;
}

FakeMoveOk::FakeMoveOk() { current_is_move_ok_ = false; }

void FakeMoveOk::SetIsMoveOk(bool is_ok) { current_is_move_ok_ = is_ok; }

bool FakeMoveOk::IsMoveOkWithPartLimits(const JointPositionCommand& setpoints) {
  return current_is_move_ok_;
}

bool FakeMoveOk::IsMoveOkWithUserLimits(const JointPositionCommand& setpoints,
                                        const JointLimits& joint_limits) {
  return current_is_move_ok_;
}

std::optional<RealtimeRobotPayload> FakePayloadState::GetActivePayload() const {
  return active_payload_;
}

void FakePayloadState::SetActivePayload(const RealtimeRobotPayload& payload) {
  active_payload_ = payload;
}

}  // namespace intrinsic::icon
