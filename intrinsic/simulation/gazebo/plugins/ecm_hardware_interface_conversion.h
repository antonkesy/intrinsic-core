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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_ECM_HARDWARE_INTERFACE_CONVERSION_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_ECM_HARDWARE_INTERFACE_CONVERSION_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/simulation/gazebo/plugins/gravity_compensator.h"

namespace intrinsic::simulation {

// This file contains methods that convert between Intrinsic hardware interfaces
// (more precisely, the Flatbuffer message types they use) and the data in a
// Gazebo EntityComponentManager (ECM).
//
// They're used by GazeboHwm / HardwareModuleLauncher.
//
// It's the caller's responsibility to select the correct ECM Entities for each
// of these calls. Check the individual comments for requirements on which
// Components those Entities must have.

// Reads joint positions from the ECM into `joint_position_buffer`.
//
// For each Entity in `joint_entities`, this function:
// 1. Checks that the Entity has a gz::sim::components::JointPosition Component
// 2. Ensures that the JointPosition Component has at least one axis
// 3. Copies the value of the JointPosition Component's first axis to the
//    corresponding index in `joint_position_buffer`
//
// The order of `joint_entities` determines the order of joint positions in
// `joint_position_buffer`. Make sure that `joint_entities` is in index
// order!
//
// A common use case is to pass in a Span made from an
// intrinsic_fbs::JointPositionState flatbuffer's `position` member, like so:
//
// ```c++
// INTR_RETURN_IF_ERROR(ReadJointPositionsFromEcm(
//     ecm,
//     joint_entities,
//     absl::MakeSpan(joint_position_state_fb.mutable_position()->data(),
//                    joint_position_state_fb.mutable_position()->size())));
// ```
//
// Returns InvalidArgument if `joint_entities` has a different size than
// `joint_position_buffer`.
// Returns NotFound if any of `joint_entities` do not exist in `ecm`.
// Returns FailedPrecondition if any of `joint_entities` do not have a
// JointPosition Component, or its JointPosition Component does not have any
// data in it.
absl::Status ReadJointPositionsFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    absl::Span<double> joint_position_buffer);

// Reads joint commanded positions from the ECM into
// `joint_commanded_position_buffer`.
//
// The order of `joint_entities` determines the order of joint commanded
// positions in `joint_commanded_position_buffer`. Make sure that
// `joint_entities` is in index order!
//
// Returns InvalidArgument if `joint_entities` has a different size than
// `joint_commanded_position_buffer`.
// Returns NotFound if any of `joint_entities` do not exist in `ecm`.
// Returns FailedPrecondition if any of `joint_entities` do not have a
// JointCommandedPosition Component, or its JointCommandedPosition Component
// does not have any data in it.
absl::Status ReadJointCommandedPositionsFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    absl::Span<double> joint_commanded_position_buffer);

// Reads joint velocities from the ECM into a hardware interface flatbuffer.
//
// For each Entity in `joint_entities`, this function:
// 1. Checks that the Entity has a gz::sim::components::JointVelocity Component
// 2. Ensures that the JointVelocity Component has at least one axis
// 3. Copies the value of the JointVelocity Component's first axis into the
//    corresponding field in `velocity_state_interface.velocity`
//
// The order of `joint_entities` determines the order of joint velocities in
// `velocity_state_interface`. Make sure that `joint_entities` is in index
// order!
//
// Returns InvalidArgument if `joint_entities` has a different size than
// `velocity_state_interface`.
// Returns NotFound if any of `joint_entities` do not exist in `ecm`.
// Returns FailedPrecondition if any of `joint_entities` do not have a
// JointVelocity Component, or its JointVelocity Component does not have any
// data in it.
absl::Status ReadJointVelocitiesFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    intrinsic_fbs::JointVelocityState& velocity_state_interface);

// Reads joint velocity reset information from the ECM into a hardware interface
// flatbuffer.
//
// For each Entity in `joint_entities`, this function:
// 1. Checks that the Entity has a gz::sim::components::JointVelocityReset
//    Component
// 2. Ensures that the JointVelocityReset Component has at least one axis
// 3a. If so, copies the value of the JointVelocityReset Component's first axis
//     into the corresponding field in `velocity_state_interface.velocity`
// 3b. If not, assumes zero velocity for that joint
//
// The order of `joint_entities` determines the order of joint velocities in
// `velocity_state_interface`. Make sure that `joint_entities` is in index
// order!
//
// Returns InvalidArgument if `joint_entities` has a different size than
// `velocity_state_interface`.
// Returns NotFound if any of `joint_entities` do not exist in `ecm`.
absl::Status ReadJointVelocityResetFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    intrinsic_fbs::JointVelocityState& velocity_state_interface);

// Reads joint torques from the ECM into a hardware interface flatbuffer.
//
// For each Entity in `joint_entities`, this function:
// 1. Checks that the Entity has a gz::sim::components::JointForce Component
// 2. Ensures that the JointForce Component has at least one axis
// 3. Copies the value of the JointForce Component's first axis into the
//    corresponding field in `torque_state_interface.torque`
//
// The order of `joint_entities` determines the order of joint torques in
// `torque_state_interface`. Make sure that `joint_entities` is in index
// order!
//
// Returns InvalidArgument if `joint_entities` has a different size than
// `torque_state_interface`.
// Returns NotFound if any of `joint_entities` do not exist in `ecm`.
// Returns FailedPrecondition if any of `joint_entities` do not have a
// JointForce Component, or its JointForce Component does not have any
// data in it.
absl::Status ReadJointTorquesFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    intrinsic_fbs::JointTorqueState& torque_state_interface);

// Applies position commands from a hardware interface flatbuffer to the ECM,
// using the gz::sim::components::JointVelocityCmd Component.
//
// For each joint, this function
// 1. Checks that the corresponding ECM Entity has the
//    gz::sim::components::JointPosition Component, and that the Component has
//    at least one axis
// 2. Computes the delta between the current position as reported by the ECM,
//    and the setpoint in `position_command_interface`
// 3. Using the step size `dt_seconds`, computes the velocity that joint needs
//    to travel at in order to reach the setpoint by the end of the simulation
//    step
// 4. Applies a JointVelocityCmd Component to the ECM Entity, so that Gazebo
//    applies that velocity during the next simulation step. Gazebo obeys the
//    position, velocity and effort limits of each joint. These correspond to
//    the Intrinsic system limits. Note, however, that Gazebo, unlike the
//    Intrinsic stack, does not have a concept of acceleration limits.
//
// Returns InvalidArgument if `dt` is less than or equal to zero.
// Returns InvalidArgument if `joint_entities` has a different size than
// `position_command_interface`.
// Returns NotFound if any of `joint_entities` do not exist in `ecm`.
// Returns FailedPrecondition if any of `joint_entities` do not have a
// JointPosition Component, or its JointPosition Component does not have any
// data in it.
absl::Status ApplyJointPositionCommandsToEcmUsingVelocityCommand(
    double dt_seconds,
    const intrinsic_fbs::JointPositionCommand& position_command_interface,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    ::gz::sim::EntityComponentManager& ecm);

// Applies a zero velocity command to `joint_entities`, meaning the joints
// should not move in this simulation step.
absl::Status ApplyZeroVelocityCommandToEcm(
    absl::Span<const ::gz::sim::Entity> joint_entities,
    ::gz::sim::EntityComponentManager& ecm);

// Applies JointPositionReset and JointVelocityReset components to
// `joint_entities`, meaning the joints should not move in this simulation step.
//
// Sets the data on the JointVelocityReset component to zero.
// Sets the data on the JointPositionReset component to the current joint
// position if two conditions are met:
// 1. There is a non-empty JointPosition component on the joint entity, the
//    first value of which is taken as the current joint position.
// 2. The joint does not already have a JointPositionReset component.
//    To avoid interfering with other systems (such as SetModelState) that may
//    use JointPositionReset to set joint position, if the joint entity already
//    has a JointPositionReset component, this does not overwrite the existing
//    component data.
//
// Only use this if the `use_sim_velocity_control` is set to false. Otherwise,
// we should simply add a HaltMotion component to the joints' parents if we want
// them to stay where they are.
absl::Status ApplyCurrentPositionJointResetToEcm(
    absl::Span<const ::gz::sim::Entity> joint_entities,
    absl::Span<const std::unique_ptr<GravityCompensator>> gravity_compensators,
    ::gz::sim::EntityComponentManager& ecm);

// Applies position commands from a hardware interface flatbuffer to the ECM,
// by resetting the joint position and velocity to the desired values.
//
// This method causes the joint states to closely track the desired trajectory
// by ignoring dynamics, which can be unrealistic and lead to incorrect
// simulation results. But on the other hand, it does not depend on correct
// dynamics models, which makes it a lot more robust against things like links
// with default (1-diagonal) inertial matrices.
//
// Also uses `gravity_compensators` to apply a torque command that should keep
// the robot stable, even when no new commands arrive. `gravity_compensators`
// must have the same size as `position_command_interface`,
// `previous_position_setpoints` and `joint_entities`, but the entries in
// `gravity_compensators` may be nullptr. If they are, this function does not do
// any gravity compensation.
//
// For each joint, this function
// 1. Computes the delta between the current setpoint (from
//    `position_command_interface`) and the previous setpoint (from
//    `previous_position_setpoints`)
// 2. Applies a gz::sim::components::JointPositionReset Component to the ECM
//    Entity to force the joint to its position setpoint
// 3. Using the step size `dt_seconds`, computes the velocity that the joint
//    would have by the end of the simulation step, if it moved from the
//    previous position setpoint to the new setpoint in `dt_seconds`
// 4. Applies a gz::sim::components::JointVelocityReset Component to the ECM
//    Entity to force the joint to the computed velocity
// 5. If `gravity_compensators` has a non-null compensator for the current
//    joint, applies a gz::sim::components::JointForceCmd Component with the
//    gravity compensation term to the ECM entity.
//
// Returns InvalidArgument if `dt` is less than or equal to zero.
// Returns InvalidArgument if `joint_entities` has a different size than
// `position_command_interface`.
// Returns InvalidArgument if `previous_position_setpoints` has a different size
// than `position_command_interface`.
// Returns InvalidArgument if `gravity_compensators` has a different size than
// `position_command_interface`.
// Returns NotFound if any of `joint_entities` do not exist in `ecm`.
// Returns FailedPrecondition if any of `joint_entities` do not have a
// JointPosition Component, or its JointPosition Component does not have any
// data in it.
absl::Status ApplyJointPositionCommandsToEcmUsingPositionResetAndVelocityReset(
    double dt_seconds, absl::Span<const double> previous_position_setpoints,
    absl::Span<const std::unique_ptr<GravityCompensator>> gravity_compensators,
    const intrinsic_fbs::JointPositionCommand& position_command_interface,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    ::gz::sim::EntityComponentManager& ecm);

// Applies torque commands from a hardware module interface flatbuffer to the
// ECM.
//
// Uses the gz::sim::components::JointForceCmd Component.
//
// Also uses `gravity_compensators` to add a torque offset to the command. This
// offset counteracts the effects of gravity. `gravity_compensators` must have
// the same size as `torque_command_interface`, but its entries may be nullptr.
// If they are, this function does not do any gravity compensation.
//
// *Do not call this without gravity compensators outside of tests!*
//
// Doing so would mean that a simulated hardware module does not fulfill the API
// contract for the intrinsic_fbs::JointTorqueCommand hardware interface.
// Commands on that interface do not take into account gravity compensation, and
// the hardware module *must* do so instead.
//
// Returns InvalidArgument if `joint_entities` has a different size than
// `torque_command_interface`.
// Returns InvalidArgument if `gravity_compensators` has a different size than
// `torque_command_interface`.
// Returns NotFound if any of `joint_entities` do not exist in `ecm`.
absl::Status ApplyJointTorqueCommandsToEcm(
    absl::Span<const std::unique_ptr<GravityCompensator>> gravity_compensators,
    const intrinsic_fbs::JointTorqueCommand& torque_command_interface,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    ::gz::sim::EntityComponentManager& ecm);

// Reads KinematicChainPayload data from the ECM into a payload state hardware
// module interface flatbuffer.
//
// The intrinsic_fbs::PayloadState flatbuffer used by the hardware interface
// contains an OptionalRobotPayload data structure. This method attempts to
// read a KinematicChainPayload component from the first joint entity in
// `joint_entities`. If the component exists, its data is copied to the
// flatbuffer. If the component does not exist, the equivalent of `std::nullopt`
// is written to the flatbuffer.
//
// Returns InvalidArgument if `joint_entities` is empty.
// Returns NotFound if the first element of `joint_entities` does not exist in
// `ecm`.
absl::Status ReadKinematicChainPayloadStateFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    intrinsic_fbs::PayloadState& payload_state_interface);

// Applies a KinematicChainPayload command from a hardware module interface
// flatbuffer to the ECM.
//
// The intrinsic_fbs::PayloadCommand flatbuffer used by the hardware interface
// contains an OptionalRobotPayload data structure. If the OptionalRobotPayload
// contains a payload value, this method copies the value to a
// KinematicChainPayload component attached to the first joint entity in
// `joint_entities`. If the OptionalRobotPayload does not contain a value, the
// KinematicChainPayload component is removed from the first joint entity in
// `joint_entities`.
//
// Returns InvalidArgument if `joint_entities` is empty.
// Returns NotFound if the first element of `joint_entities` does not exist in
// `ecm`.
absl::Status ApplyKinematicChainPayloadCommandToEcm(
    const intrinsic_fbs::PayloadCommand& payload_command_interface,
    absl::Span<const ::gz::sim::Entity> joint_entities,
    ::gz::sim::EntityComponentManager& ecm);

// Copies a force torque sensor reading from the ECM into a ForceTorqueStatus
// hardware interface.
//
// Uses the gz::sim::components::WrenchMeasured Component to read sensor values.
//
// Returns NotFound if `sensor_entity` does not exist in `ecm`.
// Returns FailedPrecondition if `sensor_entity` does not have a
// ForceTorqueTaring Component.
// Returns FailedPrecondition if `sensor_entity` does not have a WrenchMeasured
// Component.
absl::Status ReadForceTorqueSensorFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    ::gz::sim::Entity sensor_entity,
    ::intrinsic_fbs::ForceTorqueStatus& force_torque_status_interface);

// Applies retare commands from a ForceTorqueCommand hardware interface to the
// ECM.
//
// Returns NotFound if `sensor_entity` does not exist in `ecm`.
// Returns FailedPrecondition if `sensor_entity` does not have a
// ForceTorqueTaring Component.
// Returns FailedPrecondition if `sensor_entity` does not have a WrenchMeasured
// Component.
absl::Status ApplyForceTorqueSensorCommandToEcm(
    const ::intrinsic_fbs::ForceTorqueCommand& ft_command_interface,
    ::gz::sim::Entity sensor_entity, ::gz::sim::EntityComponentManager& ecm);

// DIO methods

// Copies the state of a single digital input block from the ECM to a DIOStatus
// hardware interface.
//
// Uses the custom ECM Component intrinsic::simulation::DigitalInput. Note that
// this copies input bits in the order they appear in the DigitalInput
// Component!
//
// Returns NotFound if `input_block_entity` does not exist in `ecm`.
// Returns FailedPrecondition if `input_block_entity` does not have a
// DigitalInput Component.
// Returns FailedPrecondition if `input_block_entity` has a different number of
// input bits than `digital_input_status`.
absl::Status ReadDigitalInputBlockFromEcm(
    const ::gz::sim::EntityComponentManager& ecm,
    ::gz::sim::Entity input_block_entity,
    ::intrinsic_fbs::DIOStatus& digital_input_status);

// Applies the bit values from a DIOCommand hardware interface to the ECM.
//
// Uses the custom ECM Component intrinsic::simulation::DigitalOutput. Note that
// this copies output bits in the order they appear in the DIOCommand hardware
// interface!
//
// Returns NotFound if `output_block_entity` does not exist in `ecm`.
// Returns FailedPrecondition if `output_block_entity` does not have a
// DigitalOutput Component.
// Returns FailedPrecondition if `output_block_entity` has a different number of
// output bits than `digital_output_command`.
absl::Status ApplyDigitalOutputBlockCommandToEcm(
    const ::intrinsic_fbs::DIOCommand& digital_output_command,
    ::gz::sim::Entity output_block_entity,
    ::gz::sim::EntityComponentManager& ecm);

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_ECM_HARDWARE_INTERFACE_CONVERSION_H_
