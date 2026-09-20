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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_velocity_state.h"

#include <memory>
#include <optional>
#include <utility>
#include <variant>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/signals/butter_filter2.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::StatusOr<JointVelocityStateFeature> JointVelocityStateFeature::Create(
    std::variant<JointVelocityStateHardwareInterface,
                 JointPositionStateHardwareInterface>
        hardware_interface,
    std::optional<double> control_frequency_hz,
    std::optional<double> velocity_filter_cutoff_frequency) {
  if (velocity_filter_cutoff_frequency.has_value() &&
      !control_frequency_hz.has_value()) {
    return absl::InvalidArgumentError(
        absl::StrCat("velocity_filter_cutoff_frequency is set (",
                     velocity_filter_cutoff_frequency.value(),
                     "Hz) but control_frequency_hz is not set."));
  }
  JointStateV sensed_velocity;
  if (std::holds_alternative<JointPositionStateHardwareInterface>(
          hardware_interface)) {
    if (*std::get<JointPositionStateHardwareInterface>(hardware_interface) ==
        nullptr) {
      return absl::InvalidArgumentError(
          "JointPositionStateHardwareInterface handle for "
          "JointVelocityStateFeature is not initialized.");
    }
    INTR_RETURN_IF_ERROR(sensed_velocity.SetSize(
        std::get<JointPositionStateHardwareInterface>(hardware_interface)
            ->position()
            ->size()));
    LOG(INFO) << "`JointVelocityEstimator` initialized using position state "
                 "hardware interface.";
  } else if (std::holds_alternative<JointVelocityStateHardwareInterface>(
                 hardware_interface)) {
    if (*std::get<JointVelocityStateHardwareInterface>(hardware_interface) ==
        nullptr) {
      return absl::InvalidArgumentError(
          "JointVelocityStateHardwareInterface handle for "
          "JointVelocityStateFeature is not initialized.");
    }
    INTR_RETURN_IF_ERROR(sensed_velocity.SetSize(
        std::get<JointVelocityStateHardwareInterface>(hardware_interface)
            ->velocity()
            ->size()));
  } else {
    return absl::InvalidArgumentError(
        "hardware_interface must be either a "
        "JointVelocityStateHardwareInterface or a "
        "JointPositionStateHardwareInterface.");
  }
  sensed_velocity.velocity = eigenmath::VectorNd::Zero(sensed_velocity.size());
  std::unique_ptr<ButterFilter2<eigenmath::VectorNd>>
      butterworth_joint_velocity_filter = nullptr;
  if (control_frequency_hz.has_value() &&
      velocity_filter_cutoff_frequency.has_value()) {
    butterworth_joint_velocity_filter =
        std::make_unique<ButterFilter2<eigenmath::VectorNd>>();
    butterworth_joint_velocity_filter->Init(
        sensed_velocity.velocity, control_frequency_hz.value(),
        velocity_filter_cutoff_frequency.value());
  }
  return JointVelocityStateFeature(
      std::move(hardware_interface), std::move(sensed_velocity),
      control_frequency_hz, std::move(butterworth_joint_velocity_filter));
}

JointVelocityStateFeature::JointVelocityStateFeature(
    std::variant<JointVelocityStateHardwareInterface,
                 JointPositionStateHardwareInterface>
        hardware_interface,
    JointStateV sensed_velocity, std::optional<double> control_frequency_hz,
    std::unique_ptr<ButterFilter2<eigenmath::VectorNd>>
        butterworth_joint_velocity_filter)
    : hardware_interface_(std::move(hardware_interface)),
      sensed_velocity_(std::move(sensed_velocity)),
      control_frequency_hz_(control_frequency_hz),
      butterworth_joint_velocity_filter_(
          std::move(butterworth_joint_velocity_filter)) {}

RealtimeStatus JointVelocityStateFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  const auto dofs = sensed_velocity_.size();
  if (std::holds_alternative<JointVelocityStateHardwareInterface>(
          hardware_interface_)) {
    for (int i = 0; i < dofs; ++i) {
      sensed_velocity_.velocity[i] =
          std::get<JointVelocityStateHardwareInterface>(hardware_interface_)
              ->velocity()
              ->Get(i);
    }
  } else if (std::holds_alternative<JointPositionStateHardwareInterface>(
                 hardware_interface_)) {
    eigenmath::VectorNd position(dofs);
    for (int i = 0; i < dofs; ++i) {
      position[i] =
          std::get<JointPositionStateHardwareInterface>(hardware_interface_)
              ->position()
              ->Get(i);
    }
    if (previous_position_.has_value() && control_frequency_hz_.has_value()) {
      sensed_velocity_.velocity = (position - previous_position_.value()) *
                                  control_frequency_hz_.value();
    }
    previous_position_ = position;
  }

  if (butterworth_joint_velocity_filter_ != nullptr) {
    butterworth_joint_velocity_filter_->Update(sensed_velocity_.velocity);
  }
  return OkStatus();
}

RealtimeStatus JointVelocityStateFeature::Reset() {
  const auto dofs = sensed_velocity_.size();
  sensed_velocity_.velocity = eigenmath::VectorNd::Zero(dofs);
  previous_position_ = std::nullopt;
  if (butterworth_joint_velocity_filter_ != nullptr) {
    butterworth_joint_velocity_filter_->Reset(eigenmath::VectorNd::Zero(dofs));
  }
  return OkStatus();
}

JointStateV JointVelocityStateFeature::GetVelocityEstimate() const {
  if (butterworth_joint_velocity_filter_ != nullptr) {
    return butterworth_joint_velocity_filter_->GetOutput();
  }
  return sensed_velocity_;
}

}  // namespace intrinsic::icon
