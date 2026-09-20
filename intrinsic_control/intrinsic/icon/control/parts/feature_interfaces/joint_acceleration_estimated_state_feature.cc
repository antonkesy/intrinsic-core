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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_acceleration_estimated_state_feature.h"

#include <cstddef>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/linear_joint_acceleration_filter.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/proto/linear_joint_acceleration_filter_config.pb.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::StatusOr<JointAccelerationEstimatedStateFeature>
JointAccelerationEstimatedStateFeature::Create(
    JointPositionStateHardwareInterface joint_position_state_hardware_interface,
    JointVelocityStateHardwareInterface joint_velocity_state_hardware_interface,
    intrinsic_proto::icon::LinearJointAccelerationFilterConfig
        linear_joint_acceleration_filter_config,
    const double control_frequency_hz) {
  if (*joint_position_state_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for JointAccelerationStateFeature is not "
        "initialized.");
  }
  if (*joint_velocity_state_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for JointAccelerationStateFeature is not "
        "initialized.");
  }
  if (joint_position_state_hardware_interface->position()->size() !=
      joint_velocity_state_hardware_interface->velocity()->size()) {
    return absl::FailedPreconditionError(
        "The position and velocity hardware interfaces have different sizes.");
  }
  JointStatePV sensed_state;
  INTR_RETURN_IF_ERROR(sensed_state.SetSize(
      joint_position_state_hardware_interface->position()->size()));
  JointStateA estimated_acceleration;
  INTR_RETURN_IF_ERROR(estimated_acceleration.SetSize(
      joint_position_state_hardware_interface->position()->size()));

  // Initialize joint acceleration estimators.
  eigenmath::MatrixXd P_init = eigenmath::MatrixXd::Identity(
      icon::LinearJointAccelerationFilter::kStateDim,
      icon::LinearJointAccelerationFilter::kStateDim);
  std::vector<intrinsic::icon::LinearJointAccelerationFilter>
      joint_acceleration_estimators;
  // Initialize the set of linear filters.
  for (size_t dof = 0;
       dof < joint_position_state_hardware_interface->position()->size();
       dof++) {
    INTR_ASSIGN_OR_RETURN(auto acceleration_filter,
                          icon::LinearJointAccelerationFilter::Create(
                              linear_joint_acceleration_filter_config,
                              control_frequency_hz, P_init));
    INTRINSIC_RT_LOG(INFO)
        << "Acceleration estimator for joint " << dof << " converged in "
        << acceleration_filter.GetDareSolution().iteration_count
        << " iterations.";
    // Cache converged covariance matrix to speed up initialization of
    // filter for subsequent joint.
    P_init = acceleration_filter.GetCovariance();
    joint_acceleration_estimators.push_back(std::move(acceleration_filter));
  }
  return JointAccelerationEstimatedStateFeature(
      std::move(joint_position_state_hardware_interface),
      std::move(joint_velocity_state_hardware_interface),
      std::move(sensed_state), std::move(estimated_acceleration),
      std::move(joint_acceleration_estimators));
}

JointAccelerationEstimatedStateFeature::JointAccelerationEstimatedStateFeature(
    JointPositionStateHardwareInterface joint_position_state_hardware_interface,
    JointVelocityStateHardwareInterface joint_velocity_state_hardware_interface,
    JointStatePV sensed_state, JointStateA estimated_acceleration,
    std::vector<intrinsic::icon::LinearJointAccelerationFilter>
        joint_acceleration_estimators)
    : joint_position_state_hardware_interface_(
          std::move(joint_position_state_hardware_interface)),
      joint_velocity_state_hardware_interface_(
          std::move(joint_velocity_state_hardware_interface)),
      sensed_state_(std::move(sensed_state)),
      estimated_acceleration_(std::move(estimated_acceleration)),
      joint_acceleration_estimators_(std::move(joint_acceleration_estimators)) {
}

RealtimeStatus JointAccelerationEstimatedStateFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  estimated_acceleration_.acceleration.setZero();
  for (int dof = 0; dof < sensed_state_.size(); ++dof) {
    sensed_state_.position[dof] =
        joint_position_state_hardware_interface_->position()->Get(dof);
    sensed_state_.velocity[dof] =
        joint_velocity_state_hardware_interface_->velocity()->Get(dof);

    icon::LinearJointAccelerationFilter::SingleJointState dof_state_in;
    icon::LinearJointAccelerationFilter::SingleJointState dof_joint_input;
    dof_state_in.position = sensed_state_.position[dof];
    dof_state_in.velocity = sensed_state_.velocity[dof];
    joint_acceleration_estimators_[dof].Predict(dof_joint_input);
    icon::LinearJointAccelerationFilter::SingleJointState dof_state_out =
        joint_acceleration_estimators_[dof].Correct(dof_state_in);
    estimated_acceleration_.acceleration[dof] = dof_state_out.acceleration;
  }

  return OkStatus();
}

RealtimeStatus JointAccelerationEstimatedStateFeature::Reset() {
  for (size_t dof = 0; dof < sensed_state_.size(); dof++) {
    icon::LinearJointAccelerationFilter::SingleJointState dof_state_in;
    dof_state_in.position =
        joint_position_state_hardware_interface_->position()->Get(dof);
    dof_state_in.velocity =
        joint_velocity_state_hardware_interface_->velocity()->Get(dof);
    joint_acceleration_estimators_[dof].SetInitialState(dof_state_in);
    joint_acceleration_estimators_[dof].Predict(dof_state_in);
    icon::LinearJointAccelerationFilter::SingleJointState dof_state_out =
        joint_acceleration_estimators_[dof].Correct(dof_state_in);
    estimated_acceleration_.acceleration[dof] = dof_state_out.acceleration;
  }
  return OkStatus();
}

JointStateA JointAccelerationEstimatedStateFeature::GetAccelerationEstimate()
    const {
  return estimated_acceleration_;
}

}  // namespace intrinsic::icon
