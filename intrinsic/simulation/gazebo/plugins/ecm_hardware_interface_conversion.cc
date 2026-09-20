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

#include "intrinsic/simulation/gazebo/plugins/ecm_hardware_interface_conversion.h"

#include <chrono>  // NOLINT(build/c++11)
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "gz/msgs/MessageTypes.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/JointForce.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/WrenchMeasured.hh"
#include "intrinsic/icon/hal/interfaces/force_sensor.fbs.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/robot_payload_utils.h"
#include "intrinsic/icon/utils/sensor_utils.h"
#include "intrinsic/simulation/gazebo/components/digital_io_components.h"
#include "intrinsic/simulation/gazebo/components/ft_sensor_tare_components.h"
#include "intrinsic/simulation/gazebo/components/joint_commanded_position.h"
#include "intrinsic/simulation/gazebo/components/kinematic_chain_payload_component.h"
#include "intrinsic/simulation/gazebo/plugins/gravity_compensator.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::simulation {
using intrinsic::simulation::JointCommandedPosition;

namespace {
constexpr int kChattyLogInterval = 3;
constexpr double kMaxJointVelocityEstimatedRadPerSec = M_PI * 3;

// Attempts to get the component data for `ComponentT` for `entity` from `ecm`.
//
// `ComponentT` must be an instantiation of gz::sim::components::Component, with
// a std::vector payload, or this won't compile.
template <class ComponentT>
absl::StatusOr<typename ComponentT::Type> GetNonEmptyVectorComponentData(
    const ::gz::sim::EntityComponentManager& ecm, gz::sim::Entity entity) {
  std::optional<typename ComponentT::Type> component_data =
      ecm.ComponentData<ComponentT>(entity);
  if (!component_data.has_value()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ECM entity ", entity, " is missing required component"));
  }
  if (component_data->empty()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ECM entity ", entity, " component data has no values"));
  }
  return *component_data;
}
}  // namespace

absl::Status ReadJointPositionsFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    absl::Span<double> joint_position_buffer) {
  if (joint_entities.size() != joint_position_buffer.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Position state interface has ",
                     joint_position_buffer.size(), " joints, but there are ",
                     joint_entities.size(), " Gazebo joint entities"));
  }
  for (int i = 0; i < joint_entities.size(); ++i) {
    const ::gz::sim::Entity& joint_entity = joint_entities.at(i);

    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "ECM has no entity ", joint_entity, " (at joint index ", i, ")"));
    }
    INTR_ASSIGN_OR_RETURN(
        std::vector<double> joint_pos_component_data,
        GetNonEmptyVectorComponentData<gz::sim::components::JointPosition>(
            ecm, joint_entity),
        _ << " looking up JointPosition component for joint " << i);

    joint_position_buffer[i] = joint_pos_component_data.at(0);
  }
  return absl::OkStatus();
}

absl::Status ReadJointCommandedPositionsFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    absl::Span<double> joint_commanded_position_buffer) {
  if (joint_entities.size() != joint_commanded_position_buffer.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Commanded position interface has ",
        joint_commanded_position_buffer.size(), " joints, but there are ",
        joint_entities.size(), " Gazebo joint entities"));
  }
  for (int i = 0; i < joint_entities.size(); ++i) {
    const ::gz::sim::Entity& joint_entity = joint_entities.at(i);

    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "ECM has no entity ", joint_entity, " (at joint index ", i, ")"));
    }

    INTR_ASSIGN_OR_RETURN(
        std::vector<double> joint_pos_component_data,
        GetNonEmptyVectorComponentData<JointCommandedPosition>(ecm,
                                                               joint_entity),
        _ << " looking up JointCommandedPosition component for joint " << i);

    joint_commanded_position_buffer[i] = joint_pos_component_data.at(0);
  }

  return absl::OkStatus();
}

absl::Status ReadJointVelocitiesFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    intrinsic_fbs::JointVelocityState& velocity_state_interface) {
  if (joint_entities.size() != velocity_state_interface.velocity()->size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Velocity state interface has ",
        velocity_state_interface.velocity()->size(), " joints, but there are ",
        joint_entities.size(), " Gazebo joint entities"));
  }
  for (int i = 0; i < joint_entities.size(); ++i) {
    const gz::sim::Entity& joint_entity = joint_entities.at(i);
    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "ECM has no entity ", joint_entity, " (at joint index ", i, ")"));
    }
    INTR_ASSIGN_OR_RETURN(
        std::vector<double> joint_vel_component_data,
        GetNonEmptyVectorComponentData<gz::sim::components::JointVelocity>(
            ecm, joint_entity),
        _ << " looking up JointVelocity component for joint " << i);

    velocity_state_interface.mutable_velocity()->Mutate(
        i, joint_vel_component_data.at(0));
  }
  return absl::OkStatus();
}

absl::Status ReadJointVelocityResetFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    intrinsic_fbs::JointVelocityState& velocity_state_interface) {
  if (joint_entities.size() != velocity_state_interface.velocity()->size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Velocity state interface has ",
        velocity_state_interface.velocity()->size(), " joints, but there are ",
        joint_entities.size(), " Gazebo joint entities"));
  }
  for (int i = 0; i < joint_entities.size(); ++i) {
    const gz::sim::Entity& joint_entity = joint_entities.at(i);
    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "ECM has no entity ", joint_entity, " (at joint index ", i, ")"));
    }
    if (absl::StatusOr<std::vector<double>> joint_vel_component_data =
            GetNonEmptyVectorComponentData<
                gz::sim::components::JointVelocityReset>(ecm, joint_entity);
        joint_vel_component_data.ok()) {
      velocity_state_interface.mutable_velocity()->Mutate(
          i, joint_vel_component_data->at(0));
    } else {
      // If there's no position reset component, assume zero velocity
      velocity_state_interface.mutable_velocity()->Mutate(i, 0);
    }
  }
  return absl::OkStatus();
}

absl::Status ReadJointTorquesFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    intrinsic_fbs::JointTorqueState& torque_state_interface) {
  if (joint_entities.size() != torque_state_interface.torque()->size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Torque state interface has ", torque_state_interface.torque()->size(),
        " joints, but there are ", joint_entities.size(),
        " Gazebo joint entities"));
  }
  for (int i = 0; i < joint_entities.size(); ++i) {
    const gz::sim::Entity& joint_entity = joint_entities.at(i);
    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "ECM has no entity ", joint_entity, " (at joint index ", i, ")"));
    }
    INTR_ASSIGN_OR_RETURN(
        std::vector<double> joint_torque_component_data,
        GetNonEmptyVectorComponentData<gz::sim::components::JointForce>(
            ecm, joint_entity),
        _ << " looking up JointForce component for joint " << i);

    torque_state_interface.mutable_torque()->Mutate(
        i, joint_torque_component_data.at(0));
  }
  return absl::OkStatus();
}

absl::Status ApplyJointPositionCommandsToEcmUsingVelocityCommand(
    double dt_seconds,
    const intrinsic_fbs::JointPositionCommand& position_command_interface,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    ::gz::sim::EntityComponentManager& ecm) {
  if (dt_seconds <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "dt_seconds must be a positive number, but is ", dt_seconds));
  }
  size_t num_joints = position_command_interface.position()->size();
  if (num_joints != joint_entities.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Position command interface has ", num_joints,
                     " joints, but there are ", joint_entities.size(),
                     " Gazebo joint entities"));
  }
  for (int i = 0; i < joint_entities.size(); ++i) {
    const gz::sim::Entity& joint_entity = joint_entities.at(i);
    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "ECM has no entity ", joint_entity, " (at joint index ", i, ")"));
    }
    double pos_cmd = position_command_interface.position()->Get(i);
    INTR_ASSIGN_OR_RETURN(
        std::vector<double> joint_pos_component_data,
        GetNonEmptyVectorComponentData<gz::sim::components::JointPosition>(
            ecm, joint_entity),
        _ << " looking up JointPosition component for joint " << i);

    // Get dt in seconds
    auto dt = std::chrono::duration<double>(dt_seconds).count();
    double current_pos = joint_pos_component_data.at(0);
    double error = pos_cmd - current_pos;
    // set target joint velocity to eliminate position error in one timestep
    double target_vel = error / dt;
    ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint_entity,
                                                                {target_vel});

    ecm.SetComponentData<::intrinsic::simulation::JointCommandedPosition>(
        joint_entity, {pos_cmd});

    // Remove the components for position / velocity reset and torque control
    ecm.RemoveComponent<gz::sim::components::JointPositionReset>(joint_entity);
    ecm.RemoveComponent<gz::sim::components::JointVelocityReset>(joint_entity);
    ecm.RemoveComponent<gz::sim::components::JointForceCmd>(joint_entity);
    // TODO(scpeters@): Consider feedforwards and stuff? The code as-is is
    // essentially what JointPositionController does in ABS mode
    // ("use_velocity_commands"):
    // https://github.com/gazebosim/gz-sim/blob/gz-sim9/src/systems/joint_position_controller/JointPositionController.cc#L608
  }
  return absl::OkStatus();
}

// Applies a zero velocity command to `joint_entities`, meaning the joints
// should not move in this simulation step.
absl::Status ApplyZeroVelocityCommandToEcm(
    absl::Span<const ::gz::sim::Entity> joint_entities,
    ::gz::sim::EntityComponentManager& ecm) {
  for (int i = 0; i < joint_entities.size(); ++i) {
    const gz::sim::Entity& joint_entity = joint_entities.at(i);
    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "ECM has no entity ", joint_entity, " (at joint index ", i, ")"));
    }
    ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint_entity,
                                                                {0});

    ecm.SetComponentData<::intrinsic::simulation::JointCommandedPosition>(
        joint_entity,
        {ecm.ComponentDefault<gz::sim::components::JointPosition>(joint_entity)
             ->Data()});
    // Remove the components for position / velocity reset and torque control
    ecm.RemoveComponent<gz::sim::components::JointPositionReset>(joint_entity);
    ecm.RemoveComponent<gz::sim::components::JointVelocityReset>(joint_entity);
    ecm.RemoveComponent<gz::sim::components::JointForceCmd>(joint_entity);
  }
  return absl::OkStatus();
}

absl::Status ApplyCurrentPositionJointResetToEcm(
    absl::Span<const ::gz::sim::Entity> joint_entities,
    absl::Span<const std::unique_ptr<GravityCompensator>> gravity_compensators,
    ::gz::sim::EntityComponentManager& ecm) {
  if (joint_entities.size() != gravity_compensators.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Joint group has ", joint_entities.size(), " joint entities, but ",
        gravity_compensators.size(), " gravity compensator pointers"));
  }
  for (size_t i = 0; i < joint_entities.size(); ++i) {
    const auto& joint_entity = joint_entities.at(i);
    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "ECM has no entity ", joint_entity, " (at joint index ", i, ")"));
    }
    // Always reset velocity to zero
    ecm.SetComponentData<::gz::sim::components::JointVelocityReset>(
        joint_entity, {0});

    // Reset position to the current position if `JointPosition` is available,
    // leave it alone otherwise.
    if (absl::StatusOr<std::vector<double>> joint_pos_component_data =
            GetNonEmptyVectorComponentData<gz::sim::components::JointPosition>(
                ecm, joint_entity);
        joint_pos_component_data.ok()) {
      // Don't change any pre-existing reset (that could come from
      // SetModelState!)
      // We overwrite the JointPositionReset component later, but using the same
      // value we read here.
      if (auto data =
              ecm.ComponentData<::gz::sim::components::JointPositionReset>(
                  joint_entity);
          data != std::nullopt) {
        // If there is a reset command, save it to JointCommandedPosition
        ecm.SetComponentData<::intrinsic::simulation::JointCommandedPosition>(
            joint_entity, *data);
      } else {
        // If there isn't already a JointCommandedPosition, set it to the
        // current position.
        // We'll reset to the last known JointCommandedPosition to freeze the
        // joint there, in order to prevent drifting because of physics engine
        // reasons (if we just reset to the current position, then the physics
        // engine will run for one step afterwards, potentially moving the
        // robot).
        if (ecm.ComponentData<::intrinsic::simulation::JointCommandedPosition>(
                joint_entity) == std::nullopt) {
          ecm.SetComponentData<::intrinsic::simulation::JointCommandedPosition>(
              joint_entity, *joint_pos_component_data);
        }
      }

      ecm.SetComponentData<::gz::sim::components::JointPositionReset>(
          joint_entity,
          ecm.ComponentDefault<::intrinsic::simulation::JointCommandedPosition>(
                 joint_entity)
              ->Data());
    }
    if (gravity_compensators.at(i) != nullptr) {
      ecm.SetComponentData<::gz::sim::components::JointForceCmd>(
          joint_entity, {gravity_compensators.at(i)->ComputeTorque()});
    }
  }
  return absl::OkStatus();
}

absl::Status ApplyJointPositionCommandsToEcmUsingPositionResetAndVelocityReset(
    double dt_seconds, absl::Span<const double> previous_position_setpoints,
    absl::Span<const std::unique_ptr<GravityCompensator>> gravity_compensators,
    const intrinsic_fbs::JointPositionCommand& position_command_interface,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    ::gz::sim::EntityComponentManager& ecm) {
  if (dt_seconds <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "dt_seconds must be a positive number, but is ", dt_seconds));
  }
  size_t num_joints = position_command_interface.position()->size();
  if (num_joints != previous_position_setpoints.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Position command interface has ", num_joints,
        " joints, but there are ", previous_position_setpoints.size(),
        " previous setpoints"));
  }
  if (num_joints != gravity_compensators.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Position command interface has ", num_joints,
                     " joints, but there are ", gravity_compensators.size(),
                     " gravity compensator pointers"));
  }
  if (num_joints != joint_entities.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Position command interface has ", num_joints,
                     " joints, but there are ", joint_entities.size(),
                     " Gazebo joint entities"));
  }
  for (int i = 0; i < joint_entities.size(); ++i) {
    const gz::sim::Entity& joint_entity = joint_entities.at(i);
    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "ECM has no entity ", joint_entity, " (at joint index ", i, ")"));
    }

    // Remove the component for velocity control
    ecm.RemoveComponent<gz::sim::components::JointVelocityCmd>(joint_entity);

    // Reset the joint velocity to an estimate of the velocity based on past and
    // current joint position. Otherwise, the simulator thinks the robot is
    // static and does not compute collision responses.
    //
    // We don't use the actual joint position since it can lead to an under
    // estimate of the link velocity as per the trajectory designed by ICON.
    // Instead we use the difference between the last set point and the
    // current set point for joint position.
    double velocity_value = (position_command_interface.position()->Get(i) -
                             previous_position_setpoints.at(i)) /
                            dt_seconds;
    if (std::fabs(velocity_value) > kMaxJointVelocityEstimatedRadPerSec) {
      LOG_EVERY_N_SEC(WARNING, kChattyLogInterval)
          << "Computed joint velocity (" << velocity_value
          << ") for kinematic-mode position control exceeds threshold ("
          << kMaxJointVelocityEstimatedRadPerSec << ") for joint " << i
          << ". Setting velocity to 0 instead";
      velocity_value = 0.0;
    }
    // Reset joint position to setpoint minus velocity * dt. This ensures that
    // the joint is at the setpoint by the *end* of the simulation tick.
    const double position_reset_value =
        position_command_interface.position()->Get(i) -
        (velocity_value * dt_seconds);
    ecm.SetComponentData<::gz::sim::components::JointPositionReset>(
        joint_entity, {position_reset_value});
    ecm.SetComponentData<::gz::sim::components::JointVelocityReset>(
        joint_entity, {velocity_value});

    ecm.SetComponentData<::intrinsic::simulation::JointCommandedPosition>(
        joint_entity, {position_command_interface.position()->Get(i)});

    if (gravity_compensators.at(i) != nullptr) {
      ecm.SetComponentData<::gz::sim::components::JointForceCmd>(
          joint_entity, {gravity_compensators.at(i)->ComputeTorque()});
    }
  }
  return absl::OkStatus();
}

absl::Status ApplyJointTorqueCommandsToEcm(
    absl::Span<const std::unique_ptr<GravityCompensator>> gravity_compensators,
    const intrinsic_fbs::JointTorqueCommand& torque_command_interface,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    ::gz::sim::EntityComponentManager& ecm) {
  size_t num_joints = torque_command_interface.torque()->size();
  if (num_joints != joint_entities.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Torque command interface has ", num_joints, " joints, but there are ",
        joint_entities.size(), " Gazebo joint entities"));
  }
  if (num_joints != gravity_compensators.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Torque command interface has ", num_joints, " joints, but there are ",
        joint_entities.size(), " gravity compensator pointers"));
  }
  for (int i = 0; i < joint_entities.size(); ++i) {
    const gz::sim::Entity& joint_entity = joint_entities.at(i);
    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "ECM has no entity ", joint_entity, " (at joint index ", i, ")"));
    }
    double torque_cmd = torque_command_interface.torque()->Get(i);
    if (gravity_compensators.at(i) != nullptr) {
      torque_cmd += gravity_compensators.at(i)->ComputeTorque();
    }
    // Remove the components for position / velocity reset and velocity control
    ecm.RemoveComponent<gz::sim::components::JointPositionReset>(joint_entity);
    ecm.RemoveComponent<gz::sim::components::JointVelocityReset>(joint_entity);
    ecm.RemoveComponent<gz::sim::components::JointVelocityCmd>(joint_entity);
    // Set the torque command
    ecm.SetComponentData<gz::sim::components::JointForceCmd>(joint_entity,
                                                             {torque_cmd});

    ecm.SetComponentData<::intrinsic::simulation::JointCommandedPosition>(
        joint_entity,
        {ecm.ComponentDefault<::gz::sim::components::JointPosition>(
                joint_entity)
             ->Data()});
  }
  return absl::OkStatus();
}

absl::Status ReadKinematicChainPayloadStateFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    intrinsic_fbs::PayloadState& payload_state_interface) {
  if (joint_entities.empty()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Payload state interface requires at least one joint, but there are ",
        "no joints in this group."));
  }
  const gz::sim::Entity& joint_entity = joint_entities.front();
  if (!ecm.HasEntity(joint_entity)) {
    return absl::NotFoundError(absl::StrCat("ECM has no entity ", joint_entity,
                                            " (at joint index ", 0, ")"));
  }
  std::optional<RobotPayloadBase> payload =
      ecm.ComponentData<intrinsic::simulation::KinematicChainPayload>(
          joint_entity);
  return intrinsic_fbs::CopyTo(payload,
                               *payload_state_interface.mutable_full_payload());
}

absl::Status ApplyKinematicChainPayloadCommandToEcm(
    const intrinsic_fbs::PayloadCommand& payload_command_interface,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    ::gz::sim::EntityComponentManager& ecm) {
  if (joint_entities.empty()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Payload command interface requires at least one joint, but there are ",
        "no joints in this group."));
  }
  const gz::sim::Entity& joint_entity = joint_entities.front();
  if (!ecm.HasEntity(joint_entity)) {
    return absl::NotFoundError(absl::StrCat("ECM has no entity ", joint_entity,
                                            " (at joint index ", 0, ")"));
  }
  std::optional<RobotPayloadBase> payload;
  INTR_RETURN_IF_ERROR(
      icon::CopyTo(*payload_command_interface.full_payload(), payload));
  if (payload.has_value()) {
    // Set the payload component if it has a value.
    ecm.SetComponentData<intrinsic::simulation::KinematicChainPayload>(
        joint_entity, *payload);
  } else {
    // Otherwise remove the payload component.
    ecm.RemoveComponent<intrinsic::simulation::KinematicChainPayload>(
        joint_entity);
  }
  return absl::OkStatus();
}

// F/T sensor methods

absl::Status ReadForceTorqueSensorFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    ::gz::sim::Entity sensor_entity,
    intrinsic_fbs::ForceTorqueStatus& force_torque_status_interface) {
  if (!ecm.HasEntity(sensor_entity)) {
    return absl::NotFoundError(
        absl::StrCat("ECM has no F/T sensor entity ", sensor_entity));
  }
  std::optional<ForceTorqueTaringData> ft_taring_component_data =
      ecm.ComponentData<ForceTorqueTaring>(sensor_entity);
  if (!ft_taring_component_data.has_value()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ECM entity ", sensor_entity,
                     " does not have ForceTorqueTaring component"));
  }
  std::optional<gz::msgs::Wrench> wrench_measured_component_data =
      ecm.ComponentData<gz::sim::components::WrenchMeasured>(sensor_entity);
  if (!wrench_measured_component_data.has_value()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ECM entity ", sensor_entity,
                     " does not have WrenchMeasured component"));
  }

  // Write only zeros if taring is in progress.
  if (ft_taring_component_data->taring_in_progress) {
    force_torque_status_interface.mutable_wrench()->mutate_x(0.0);
    force_torque_status_interface.mutable_wrench()->mutate_y(0.0);
    force_torque_status_interface.mutable_wrench()->mutate_z(0.0);
    force_torque_status_interface.mutable_wrench()->mutate_rx(0.0);
    force_torque_status_interface.mutable_wrench()->mutate_ry(0.0);
    force_torque_status_interface.mutable_wrench()->mutate_rz(0.0);
    force_torque_status_interface.mutate_retare_completed(false);
    return absl::OkStatus();
  }
  auto& mutable_wrench = *force_torque_status_interface.mutable_wrench();
  const auto& measured_force = wrench_measured_component_data->force();
  const auto& measured_torque = wrench_measured_component_data->torque();
  const auto& ft_sensor_bias = ft_taring_component_data->ft_sensor_bias;
  mutable_wrench.mutate_x(measured_force.x() - ft_sensor_bias[0].Bias());
  mutable_wrench.mutate_y(measured_force.y() - ft_sensor_bias[1].Bias());
  mutable_wrench.mutate_z(measured_force.z() - ft_sensor_bias[2].Bias());
  mutable_wrench.mutate_rx(measured_torque.x() - ft_sensor_bias[3].Bias());
  mutable_wrench.mutate_ry(measured_torque.y() - ft_sensor_bias[4].Bias());
  mutable_wrench.mutate_rz(measured_torque.z() - ft_sensor_bias[5].Bias());

  force_torque_status_interface.mutate_retare_completed(true);

  force_torque_status_interface.mutate_status_code(
      intrinsic_fbs::ForceSensorStatusCode::Ok);
  return absl::OkStatus();
}

absl::Status ApplyForceTorqueSensorCommandToEcm(
    const ::intrinsic_fbs::ForceTorqueCommand& ft_command_interface,
    ::gz::sim::Entity sensor_entity, ::gz::sim::EntityComponentManager& ecm) {
  if (!ecm.HasEntity(sensor_entity)) {
    return absl::NotFoundError(
        absl::StrCat("ECM has no F/T sensor entity ", sensor_entity));
  }
  std::optional<gz::msgs::Wrench> wrench_measured_component_data =
      ecm.ComponentData<gz::sim::components::WrenchMeasured>(sensor_entity);
  if (!wrench_measured_component_data.has_value()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ECM entity ", sensor_entity,
                     " does not have WrenchMeasured component"));
  }
  auto* ft_taring_component = ecm.Component<ForceTorqueTaring>(sensor_entity);
  if (!ft_taring_component) {
    return absl::FailedPreconditionError(
        absl::StrCat("ECM entity ", sensor_entity,
                     " does not have ForceTorqueTaring component"));
  }

  auto& ft_taring_data = ft_taring_component->Data();
  // Start the taring process if
  // * the command interface requests a retare
  // * we're not already taring the sensor
  if (ft_command_interface.retare() && !ft_taring_data.taring_in_progress) {
    // Adapted from intrinsic::ati::AtiForceTorqueBusDevice::StartTaring()
    for (auto& bias : ft_taring_data.ft_sensor_bias) {
      bias.ResetBias();
      bias.ResetSampleCount();
    }
    ft_taring_data.taring_in_progress = true;
    ft_taring_data.taring_cycles = ft_command_interface.num_taring_cycles();
  }

  if (!ft_taring_data.taring_in_progress) {
    // Nothing to do if we're not taring
    return absl::OkStatus();
  }

  // Adapted from intrinsic::ati::AtiForceTorqueBusDevice::Tare()
  auto add_bias_sample = [](::intrinsic::icon::DofSensorBias& bias,
                            double sensor_reading) -> void {
    if (bias.SampleCount() == 0) {
      bias.ResetBias();
    }
    bias.AddSample(sensor_reading);
  };
  auto& ft_sensor_bias = ft_taring_data.ft_sensor_bias;
  auto& wrench = *wrench_measured_component_data;
  add_bias_sample(ft_sensor_bias[0], wrench.force().x());
  add_bias_sample(ft_sensor_bias[1], wrench.force().y());
  add_bias_sample(ft_sensor_bias[2], wrench.force().z());
  add_bias_sample(ft_sensor_bias[3], wrench.torque().x());
  add_bias_sample(ft_sensor_bias[4], wrench.torque().y());
  add_bias_sample(ft_sensor_bias[5], wrench.torque().z());

  // Adapted from intrinsic::ati::AtiForceTorqueBusDevice::EndTaring()
  if (ft_sensor_bias[0].SampleCount() >= ft_taring_data.taring_cycles) {
    ft_taring_data.taring_in_progress = false;
    ft_taring_data.taring_cycles = 0;
  }

  return absl::OkStatus();
}

// DIO methods

absl::Status ReadDigitalInputBlockFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    ::gz::sim::Entity input_block_entity,
    intrinsic_fbs::DIOStatus& digital_input_status) {
  if (!ecm.HasEntity(input_block_entity)) {
    return absl::NotFoundError(
        absl::StrCat("ECM has no DigitalInput entity ", input_block_entity));
  }
  std::optional<DigitalIoData> digital_input_component_data =
      ecm.ComponentData<DigitalInput>(input_block_entity);
  if (!digital_input_component_data.has_value()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ECM entity ", input_block_entity,
                     " does not have DigitalInput component"));
  }
  if (digital_input_component_data->data.size() !=
      digital_input_status.signals()->size()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ECM entity ", input_block_entity,
                     " has a different number of input bits (",
                     digital_input_component_data->data.size(),
                     ") than our hardware interface (",
                     digital_input_status.signals()->size(), ")"));
  }
  for (int i = 0; i < digital_input_status.signals()->size(); ++i) {
    digital_input_status.mutable_signals()->GetMutableObject(i)->mutate_value(
        digital_input_component_data->data.at(i));
  }

  return absl::OkStatus();
}

absl::Status ApplyDigitalOutputBlockCommandToEcm(
    const intrinsic_fbs::DIOCommand& digital_output_command,
    ::gz::sim::Entity output_block_entity,
    ::gz::sim::EntityComponentManager& ecm) {
  if (!ecm.HasEntity(output_block_entity)) {
    return absl::NotFoundError(
        absl::StrCat("ECM has no DigitalOutput entity ", output_block_entity));
  }
  DigitalOutput* digital_output_component =
      ecm.Component<DigitalOutput>(output_block_entity);
  if (digital_output_component == nullptr) {
    return absl::FailedPreconditionError(
        absl::StrCat("ECM entity ", output_block_entity,
                     " does not have DigitalOutput component"));
  }
  if (digital_output_component->Data().data.size() !=
      digital_output_command.signals()->size()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ECM entity ", output_block_entity,
                     " has a different number of output bits (",
                     digital_output_component->Data().data.size(),
                     ") than our hardware interface (",
                     digital_output_command.signals()->size(), ")"));
  }
  for (int i = 0; i < digital_output_command.signals()->size(); ++i) {
    digital_output_component->Data().data.at(i) =
        digital_output_command.signals()->Get(i)->value();
  }

  return absl::OkStatus();
}

}  // namespace intrinsic::simulation
