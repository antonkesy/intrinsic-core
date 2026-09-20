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

#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"

#include <stddef.h>

#include <string>

#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/proto/cart_space.pb.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/kinematics/proto/kinematics_conversion.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.pb.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/proto/pose.pb.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

intrinsic_proto::icon::GenericPartConfig ExtractGenericConfig(
    const FeatureInterfaceRegistry& feature_interfaces) {
  intrinsic_proto::icon::GenericPartConfig generic_config;

  if (const auto* joint_position =
          feature_interfaces.GetInterface<icon::JointPosition>();
      joint_position != nullptr) {
    generic_config.mutable_joint_position_config()->set_num_joints(
        joint_position->PreviousPositionSetpoints().Size());
  }
  // Save the JointPositionSensor pointer. We use it as a fallback for feature
  // interfaces whose config contains num_joints, but do not make that
  // information available.
  const auto* joint_position_sensor =
      feature_interfaces.GetInterface<icon::JointPositionSensor>();
  if (joint_position_sensor != nullptr) {
    generic_config.mutable_joint_position_sensor_config()->set_num_joints(
        joint_position_sensor->GetSensedPosition().size());
  }
  if (const auto* joint_velocity =
          feature_interfaces.GetInterface<icon::JointVelocity>();
      joint_velocity != nullptr) {
    // No way to set num_dofs from here (the JointVelocity interface doesn't
    // expose that information).
    generic_config.mutable_joint_velocity_config();
    // But if we have a JointPositionSensor, assume that that uses the same
    // joints as the JointVelocity interface.
    if (joint_position_sensor != nullptr) {
      generic_config.mutable_joint_velocity_config()->set_num_joints(
          joint_position_sensor->GetSensedPosition().size());
      LOG(INFO) << "Inferring number of joints for JointVelocity feature "
                   "interface from JointPositionSensor.";
    } else {
      LOG(WARNING)
          << "Can't extract configuration for JointVelocty feature interface. "
             "The resulting GenericPartConfig is incomplete!";
    }
  }
  if (const auto* joint_velocity_estimator =
          feature_interfaces.GetInterface<icon::JointVelocityEstimator>();
      joint_velocity_estimator != nullptr) {
    generic_config.mutable_joint_velocity_estimator_config()->set_num_joints(
        joint_velocity_estimator->GetVelocityEstimate().size());
  }
  if (const auto* joint_acceleration =
          feature_interfaces.GetInterface<icon::JointAcceleration>();
      joint_acceleration != nullptr) {
    generic_config.mutable_joint_acceleration_config()->set_num_joints(
        joint_acceleration->PreviousAccelerationSetpoints().Size());
  }
  if (const auto* joint_acceleration_estimator =
          feature_interfaces.GetInterface<icon::JointAccelerationEstimator>();
      joint_acceleration_estimator != nullptr) {
    generic_config.mutable_joint_acceleration_estimator_config()
        ->set_num_joints(
            joint_acceleration_estimator->GetAccelerationEstimate().size());
  }
  if (const auto* joint_limits =
          feature_interfaces.GetInterface<icon::JointLimitsInterface>();
      joint_limits != nullptr) {
    *generic_config.mutable_joint_limits_config()
         ->mutable_application_limits() =
        ToProto(joint_limits->GetApplicationLimits());
    *generic_config.mutable_joint_limits_config()->mutable_system_limits() =
        ToProto(joint_limits->GetSystemLimits());
  }
  if (auto* cart_limits =
          feature_interfaces.GetInterface<CartesianLimitsInterface>();
      cart_limits != nullptr) {
    *generic_config.mutable_cartesian_limits_config()
         ->mutable_default_cartesian_limits() =
        ToProto(cart_limits->GetDefaultCartesianLimits());
  }
  if (const auto* simple_gripper =
          feature_interfaces.GetInterface<icon::SimpleGripper>();
      simple_gripper != nullptr) {
    generic_config.mutable_simple_gripper_config();
  }
  if (const auto* adio = feature_interfaces.GetInterface<icon::ADIO>();
      adio != nullptr) {
    intrinsic_proto::icon::GenericAdioConfig* adio_config =
        generic_config.mutable_adio_config();
    ADIO::ADIOState state = adio->GetADIOState();
    for (size_t i = 0; i < state.digital_inputs.size(); ++i) {
      auto block_name = state.digital_input_block_names.at(i);
      auto block_size = state.digital_inputs.at(i).Values().size();

      intrinsic_proto::icon::GenericAdioConfig::DigitalInputOutputConfig
          digital_input_config;
      digital_input_config.set_block_size(block_size);
      absl::Span<const std::string> signal_names =
          adio->DigitalInputSignalNames(block_name);
      if (signal_names.size() != block_size) {
        LOG(WARNING) << "Digital input block " << block_name << " has size "
                     << block_size << " but " << signal_names.size()
                     << " signal names.";
      } else {
        for (const std::string& signal_name : signal_names) {
          digital_input_config.add_signal_names(signal_name);
        }
        (*adio_config->mutable_digital_input_blocks())[block_name] =
            digital_input_config;
      }
    }
    for (size_t i = 0; i < state.digital_outputs.size(); ++i) {
      auto block_name = state.digital_output_block_names.at(i);
      auto block_size = state.digital_outputs.at(i).Values().size();

      intrinsic_proto::icon::GenericAdioConfig::DigitalInputOutputConfig
          digital_output_config;
      digital_output_config.set_block_size(block_size);
      absl::Span<const std::string> signal_names =
          adio->DigitalOutputSignalNames(block_name);
      if (signal_names.size() != block_size) {
        LOG(WARNING) << "Digital output block " << block_name << " has size "
                     << block_size << " but " << signal_names.size()
                     << " signal names.";
      } else {
        for (const std::string& signal_name : signal_names) {
          digital_output_config.add_signal_names(signal_name);
        }
        (*adio_config->mutable_digital_output_blocks())[block_name] =
            digital_output_config;
      }
    }
    for (size_t i = 0; i < state.analog_inputs.size(); ++i) {
      for (size_t j = 0; j < state.analog_inputs.at(i).Values().size(); ++j) {
        auto block_name = state.analog_input_block_names.at(i);
        auto block_size = state.analog_inputs.at(i).Values().size();

        intrinsic_proto::icon::GenericAdioConfig::AnalogInputOutputConfig
            analog_input_config;
        analog_input_config.set_block_size(block_size);
        absl::Span<const std::string> signal_names =
            adio->AnalogInputSignalNames(block_name);
        if (signal_names.size() != block_size) {
          LOG(WARNING) << "Analog input block " << block_name << " has size "
                       << block_size << " but " << signal_names.size()
                       << " signal names.";
        } else {
          for (const std::string& signal_name : signal_names) {
            analog_input_config.add_signal_names(signal_name);
            analog_input_config.add_units("unknown");
          }
          (*adio_config->mutable_analog_input_blocks())[block_name] =
              analog_input_config;
        }
      }
    }
    for (size_t i = 0; i < state.analog_outputs.size(); ++i) {
      for (size_t j = 0; j < state.analog_outputs.at(i).Values().size(); ++j) {
        auto block_name = state.analog_output_block_names.at(i);
        auto block_size = state.analog_outputs.at(i).Values().size();

        intrinsic_proto::icon::GenericAdioConfig::AnalogInputOutputConfig
            analog_output_config;
        analog_output_config.set_block_size(block_size);
        absl::Span<const std::string> signal_names =
            adio->AnalogOutputSignalNames(block_name);
        if (signal_names.size() != block_size) {
          LOG(WARNING) << "Analog output block " << block_name << " has size "
                       << block_size << " but " << signal_names.size()
                       << " signal names.";
        } else {
          for (const std::string& signal_name : signal_names) {
            analog_output_config.add_signal_names(signal_name);
            analog_output_config.add_units("unknown");
          }
          (*adio_config->mutable_analog_output_blocks())[block_name] =
              analog_output_config;
        }
      }
    }
  }
  if (const auto* process_wrench =
          feature_interfaces.GetInterface<icon::ProcessWrenchAtEndeffector>();
      process_wrench != nullptr) {
    generic_config.mutable_process_wrench_config();
  }
  if (const auto* range_finder =
          feature_interfaces.GetInterface<icon::RangeFinder>();
      range_finder != nullptr) {
    *generic_config.mutable_range_finder_config()->mutable_pose_in_tcp_frame() =
        ::intrinsic::ToProto(range_finder->GetPoseInTCPFrame());
  }
  if (const auto* imu =
          feature_interfaces.GetInterface<icon::InertialMeasurementUnit>();
      imu != nullptr) {
    *generic_config.mutable_imu_config()->mutable_pose_in_flange_frame() =
        ::intrinsic::ToProto(imu->GetPoseInFlangeFrame());
  }
  if (const auto* kinematics =
          feature_interfaces.GetInterface<icon::ManipulatorKinematics>();
      kinematics != nullptr) {
    if (absl::StatusOr<intrinsic_proto::Skeleton> skeleton_proto =
            ToProto(kinematics->GetKinematicsModel());
        skeleton_proto.ok()) {
      *generic_config.mutable_manipulator_kinematics_config()
           ->mutable_skeleton() = *skeleton_proto;
    }
    generic_config.mutable_manipulator_kinematics_config()->set_solver_key(
        kinematics->GetInverseKinematicsSolverName());
  }
  if (const auto* joint_torque =
          feature_interfaces.GetInterface<icon::JointTorque>();
      joint_torque != nullptr) {
    // No way to set num_dofs from here (the JointTorque interface doesn't
    // expose that information).
    generic_config.mutable_joint_torque_config();
    // But if we have a JointPositionSensor, assume that that uses the same
    // joints as the JointTorque interface.
    if (joint_position_sensor != nullptr) {
      generic_config.mutable_joint_torque_config()->set_num_joints(
          joint_position_sensor->GetSensedPosition().size());
      LOG(INFO) << "Inferring number of joints for JointTorque feature "
                   "interface from JointPositionSensor.";
    } else {
      LOG(WARNING)
          << "Can't extract configuration for JointTorque feature interface. "
             "The resulting GenericPartConfig is incomplete!";
    }
  }
  if (const auto* joint_torque_sensor =
          feature_interfaces.GetInterface<icon::JointTorqueSensor>();
      joint_torque_sensor != nullptr) {
    generic_config.mutable_joint_torque_sensor_config()->set_num_joints(
        joint_torque_sensor->GetSensedTorque().size());
  }
  if (const auto* dynamics = feature_interfaces.GetInterface<icon::Dynamics>();
      dynamics != nullptr) {
    // Nothing to set here.
    generic_config.mutable_dynamics_config();
  }
  if (const auto* force_torque_sensor =
          feature_interfaces.GetInterface<icon::ForceTorqueSensor>();
      force_torque_sensor != nullptr) {
    // Nothing to set here.
    generic_config.mutable_force_torque_sensor_config();
  }
  if (const auto* standalone_force_torque_sensor =
          feature_interfaces.GetInterface<icon::StandaloneForceTorqueSensor>();
      standalone_force_torque_sensor != nullptr) {
    // Nothing to set here.
    generic_config.mutable_standalone_force_torque_sensor_config();
  }
  if (const auto* linear_gripper =
          feature_interfaces.GetInterface<icon::LinearGripper>();
      linear_gripper != nullptr) {
    // No way to set the linear gripper bounds from here (the LinearGripper
    // interface doesn't expose that information).
    generic_config.mutable_linear_gripper_config();
    LOG(WARNING) << "Can't extract configuration for LinearGripper feature "
                    "interface. The resulting GenericPartConfig is invalid!";
  }
  if (const auto* native_hand_guiding =
          feature_interfaces.GetInterface<icon::HandGuiding>();
      native_hand_guiding != nullptr) {
    // Nothing to set here.
    generic_config.mutable_native_hand_guiding_config();
  }
  if (const auto* homing = feature_interfaces.GetInterface<icon::Homing>();
      homing != nullptr) {
    // Nothing to set here.
    generic_config.mutable_homing_config();
  }
  if (const auto* control_mode_exporter =
          feature_interfaces.GetInterface<icon::ControlModeExporter>();
      control_mode_exporter != nullptr) {
    // Nothing to set here.
    generic_config.mutable_control_mode_exporter_config();
  }
  if (const auto* move_ok = feature_interfaces.GetInterface<icon::MoveOk>();
      move_ok != nullptr) {
    // Nothing to set here.
    generic_config.mutable_move_ok_config();
  }
  if (const auto* payload = feature_interfaces.GetInterface<icon::Payload>();
      payload != nullptr) {
    // Nothing to set here.
    generic_config.mutable_payload_config();
  }
  if (const auto* payload_state =
          feature_interfaces.GetInterface<icon::PayloadState>();
      payload_state != nullptr) {
    // Nothing to set here.
    generic_config.mutable_payload_state_config();
  }
  if (const auto* cartesian_position_state =
          feature_interfaces.GetInterface<icon::CartesianPositionState>();
      cartesian_position_state != nullptr) {
    // Nothing to set here.
    generic_config.mutable_cartesian_position_state_config();
  }
  return generic_config;
}

absl::Status ValidateGenericConfig(
    absl::flat_hash_set<::intrinsic_proto::icon::v1::FeatureInterfaceTypes>
        feature_interfaces,
    const intrinsic_proto::icon::GenericPartConfig& generic_config) {
  for (const auto& feature_interface : feature_interfaces) {
    switch (feature_interface) {
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_JOINT_POSITION:
        if (!generic_config.has_joint_position_config()) {
          return absl::FailedPreconditionError(absl::StrCat(
              "Part claims to support JointPosition Feature Interface, but "
              "does not provide the corresponding GenericConfig. ",
              generic_config));
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_JOINT_VELOCITY:
        if (!generic_config.has_joint_velocity_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support JointVelocity Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_JOINT_ACCELERATION:
        if (!generic_config.has_joint_acceleration_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support JointAcceleration Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_JOINT_POSITION_SENSOR:
        if (!generic_config.has_joint_position_sensor_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support JointPositionSensor Feature Interface, "
              "but does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::
          FEATURE_INTERFACE_JOINT_VELOCITY_ESTIMATOR:
        if (!generic_config.has_joint_velocity_estimator_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support JointVelocityEstimator Feature "
              "Interface, but does not provide the corresponding "
              "GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::
          FEATURE_INTERFACE_JOINT_ACCELERATION_ESTIMATOR:
        if (!generic_config.has_joint_acceleration_estimator_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support JointAccelerationEstimator Feature "
              "Interface, but does not provide the corresponding "
              "GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_JOINT_LIMITS:
        if (!generic_config.has_joint_limits_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support JointLimits Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_CARTESIAN_LIMITS:
        if (!generic_config.has_cartesian_limits_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support CartesianLimits Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_SIMPLE_GRIPPER:
        if (!generic_config.has_simple_gripper_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support SimpleGripper Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_ADIO:
        if (!generic_config.has_adio_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support ADIO Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_RANGE_FINDER:
        if (!generic_config.has_range_finder_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support RangeFinder Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_IMU:
        if (!generic_config.has_imu_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support InertialMeasurementUnit Feature "
              "Interface, but does not provide the corresponding "
              "GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_MANIPULATOR_KINEMATICS:
        if (!generic_config.has_manipulator_kinematics_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support ManipulatorKinematics Feature Interface, "
              "but does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_JOINT_TORQUE:
        if (!generic_config.has_joint_torque_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support JointTorque Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_JOINT_TORQUE_SENSOR:
        if (!generic_config.has_joint_torque_sensor_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support JointTorqueSensor Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_DYNAMICS:
        if (!generic_config.has_dynamics_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support Dynamics Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::
          FEATURE_INTERFACE_STANDALONE_FORCE_TORQUE_SENSOR:
        if (!generic_config.has_standalone_force_torque_sensor_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support StandaloneForceTorqueSensor Feature "
              "Interface, but does not provide the corresponding "
              "GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_FORCE_TORQUE_SENSOR:
        if (!generic_config.has_force_torque_sensor_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support ForceTorqueSensor Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_LINEAR_GRIPPER:
        if (!generic_config.has_linear_gripper_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support LinearGripper Feature Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_HAND_GUIDING:
        if (!generic_config.has_native_hand_guiding_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support the HandGuiding Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_HOMING:
        if (!generic_config.has_homing_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support the Homing Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_CONTROL_MODE_EXPORTER:
        if (!generic_config.has_control_mode_exporter_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support the ControlModeExporter Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_MOVE_OK:
        if (!generic_config.has_move_ok_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support the MoveOk Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::
          FEATURE_INTERFACE_PROCESS_WRENCH_AT_ENDEFFECTOR:
        if (!generic_config.has_process_wrench_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support the ProcessWrenchCommand Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_PAYLOAD:
        if (!generic_config.has_payload_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support the Payload Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::FEATURE_INTERFACE_PAYLOAD_STATE:
        if (!generic_config.has_payload_state_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support the PayloadState Interface, but "
              "does not provide the corresponding GenericConfig.");
        }
        break;
      case intrinsic_proto::icon::v1::
          FEATURE_INTERFACE_CARTESIAN_POSITION_STATE:
        if (!generic_config.has_cartesian_position_state_config()) {
          return absl::FailedPreconditionError(
              "Part claims to support the CartesianPositionState Interface, "
              "but does not provide the corresponding GenericConfig.");
        }
        break;
      default:
        return absl::FailedPreconditionError(
            "Encountered unknown Feature Interface type.");
    }
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::icon
