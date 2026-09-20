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

#include "intrinsic/icon/control/parts/realtime_part_status.h"

#include <stdint.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/io_block.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/icon/proto/io_block.pb.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/status/status_macros.h"
#include "magic_enum/magic_enum.hpp"

namespace intrinsic::icon {
namespace {

RealtimeOperationalState ToRealtimeOperationalState(
    intrinsic_proto::icon::v1::OperationalState state) {
  switch (state) {
    case intrinsic_proto::icon::v1::OperationalState::ENABLED:
      return RealtimeOperationalState::kEnabled;
    case intrinsic_proto::icon::v1::OperationalState::DISABLED:
      return RealtimeOperationalState::kDisabled;
    case intrinsic_proto::icon::v1::OperationalState::UNKNOWN:
    case intrinsic_proto::icon::v1::OperationalState::FAULTED:
    default:
      return RealtimeOperationalState::kFaultedConnected;
  }
}

}  // namespace

absl::StatusOr<RealtimePartStatus> FromProto(
    const intrinsic_proto::icon::PartStatus& proto_status,
    const std::vector<std::string>* analog_input_block_names,
    const std::vector<std::string>* analog_output_block_names,
    const std::vector<std::string>* digital_input_block_names,
    const std::vector<std::string>* digital_output_block_names) {
  INTRINSIC_ASSERT_NON_REALTIME();
  RealtimePartStatus realtime_status;
  realtime_status.time_since_timeslicer_epoch =
      absl::Nanoseconds(proto_status.timestamp_ns());
  realtime_status.operational_status.state =
      ToRealtimeOperationalState(proto_status.operational_status().state());
  if (!proto_status.operational_status().fault_reason().empty()) {
    realtime_status.operational_status.fault_reason =
        RealtimeOperationalStatus::FaultReasonString(
            proto_status.operational_status().fault_reason());
  }

  size_t joint_states_size = proto_status.joint_states_size();
  if (joint_states_size > 0) {
    JointStateP rt_joint_state_p;
    JointStateV rt_joint_state_v;
    JointStateA rt_joint_state_a;
    JointStateT rt_joint_state_t;
    JointStateP rt_previously_commanded_p;
    JointStateV rt_previously_commanded_v;
    JointStateA rt_previously_commanded_a;
    JointStateT rt_previously_commanded_t;
    INTR_RETURN_IF_ERROR(rt_joint_state_p.SetSize(joint_states_size));
    INTR_RETURN_IF_ERROR(rt_joint_state_v.SetSize(joint_states_size));
    INTR_RETURN_IF_ERROR(rt_joint_state_a.SetSize(joint_states_size));
    INTR_RETURN_IF_ERROR(rt_joint_state_t.SetSize(joint_states_size));
    INTR_RETURN_IF_ERROR(rt_previously_commanded_p.SetSize(joint_states_size));
    INTR_RETURN_IF_ERROR(rt_previously_commanded_v.SetSize(joint_states_size));
    INTR_RETURN_IF_ERROR(rt_previously_commanded_a.SetSize(joint_states_size));
    INTR_RETURN_IF_ERROR(rt_previously_commanded_t.SetSize(joint_states_size));
    bool joint_state_has_velocity = false;
    bool joint_state_has_acceleration = false;
    bool joint_state_has_torque = false;
    bool joint_state_has_previously_commanded_position = false;
    bool joint_state_has_previously_commanded_velocity = false;
    bool joint_state_has_previously_commanded_acceleration = false;
    bool joint_state_has_previously_commanded_torque = false;
    for (size_t i = 0; i < joint_states_size; ++i) {
      const auto& joint_state = proto_status.joint_states().at(i);
      // TODO(b/174255735): position_sensed should be optional
      rt_joint_state_p.position[i] = joint_state.position_sensed();
      if (joint_state.has_velocity_sensed()) {
        rt_joint_state_v.velocity[i] = joint_state.velocity_sensed();
        joint_state_has_velocity = true;
      }
      if (joint_state.has_acceleration_sensed()) {
        rt_joint_state_a.acceleration[i] = joint_state.acceleration_sensed();
        joint_state_has_acceleration = true;
      }
      if (joint_state.has_torque_sensed()) {
        rt_joint_state_t.torque[i] = joint_state.torque_sensed();
        joint_state_has_torque = true;
      }
      if (joint_state.has_position_commanded_last_cycle()) {
        rt_previously_commanded_p.position[i] =
            joint_state.position_commanded_last_cycle();
        joint_state_has_previously_commanded_position = true;
      }
      if (joint_state.has_velocity_commanded_last_cycle()) {
        rt_previously_commanded_v.velocity[i] =
            joint_state.velocity_commanded_last_cycle();
        joint_state_has_previously_commanded_velocity = true;
      }
      if (joint_state.has_acceleration_commanded_last_cycle()) {
        rt_previously_commanded_a.acceleration[i] =
            joint_state.acceleration_commanded_last_cycle();
        joint_state_has_previously_commanded_acceleration = true;
      }
      if (joint_state.has_torque_commanded_last_cycle()) {
        rt_previously_commanded_t.torque[i] =
            joint_state.torque_commanded_last_cycle();
        joint_state_has_previously_commanded_torque = true;
      }
    }

    realtime_status.sensed_position = rt_joint_state_p;
    if (joint_state_has_velocity) {
      realtime_status.sensed_velocity = rt_joint_state_v;
    }
    if (joint_state_has_acceleration) {
      realtime_status.sensed_acceleration = rt_joint_state_a;
    }
    if (joint_state_has_torque) {
      realtime_status.sensed_torque = rt_joint_state_t;
    }
    if (joint_state_has_previously_commanded_position) {
      realtime_status.position_commanded_last_cycle = rt_previously_commanded_p;
    }
    if (joint_state_has_previously_commanded_velocity) {
      realtime_status.velocity_commanded_last_cycle = rt_previously_commanded_v;
    }
    if (joint_state_has_previously_commanded_acceleration) {
      realtime_status.acceleration_commanded_last_cycle =
          rt_previously_commanded_a;
    }
    if (joint_state_has_previously_commanded_torque) {
      realtime_status.torque_commanded_last_cycle = rt_previously_commanded_t;
    }
  }

  if (proto_status.has_gripper_state()) {
    const auto& sensed_state = proto_status.gripper_state().sensed_state();
    switch (sensed_state) {
      case intrinsic_proto::icon::GripperState::SENSED_STATE_UNKNOWN:
        realtime_status.gripper_state = SimpleGripper::GripperState::kUnknown;
        break;
      case intrinsic_proto::icon::GripperState::SENSED_STATE_FREE:
        realtime_status.gripper_state = SimpleGripper::GripperState::kReleased;
        break;
      case intrinsic_proto::icon::GripperState::SENSED_STATE_HOLDING:
        realtime_status.gripper_state = SimpleGripper::GripperState::kGrasped;
        break;
      default:
        // Sentinel do not use value
        return absl::InvalidArgumentError(
            absl::StrCat("Could not convert proto payload for Part to "
                         "RealtimePartStatus. GripperState is: ",
                         sensed_state));
    }
  }
  if (proto_status.has_linear_gripper_state()) {
    realtime_status.linear_gripper_width_sensed =
        proto_status.linear_gripper_state().sensed_width();
  }
  if (proto_status.has_base_t_tip_sensed()) {
    INTR_ASSIGN_OR_RETURN(Pose3d base_t_tip_sensed,
                          FromProto(proto_status.base_t_tip_sensed()));

    if (realtime_status.base_t_tip_sensed.has_value() &&
        !realtime_status.base_t_tip_sensed.value().isApprox(
            base_t_tip_sensed)) {
      return absl::InvalidArgumentError(
          "has_base_t_tip_sensed of L2 PartStatus doesn't match the "
          "RealtimePartStatus");
    } else {
      realtime_status.base_t_tip_sensed = base_t_tip_sensed;
    }
  }

  if (proto_status.has_wrench_at_ft()) {
    realtime_status.wrench_at_ft = FromProto(proto_status.wrench_at_ft());
  }

  if (proto_status.has_wrench_at_tip()) {
    realtime_status.wrench_at_tip = FromProto(proto_status.wrench_at_tip());
  }

  if (proto_status.has_wrench_at_ft_uncompensated()) {
    realtime_status.wrench_at_ft_uncompensated =
        FromProto(proto_status.wrench_at_ft_uncompensated());
  }

  if (proto_status.has_wrench_stability_index()) {
    realtime_status.wrench_stability_index =
        proto_status.wrench_stability_index();
  }

  if (proto_status.has_base_twist_tip_sensed()) {
    realtime_status.base_twist_tip_sensed =
        FromProto(proto_status.base_twist_tip_sensed());
  }

  if (proto_status.has_adio_state()) {
    if (analog_input_block_names != nullptr &&
        analog_output_block_names != nullptr &&
        digital_output_block_names != nullptr &&
        digital_input_block_names != nullptr) {
      auto& adio_in = proto_status.adio_state();
      ADIO::ADIOState adio;
      adio.analog_input_block_names = *analog_input_block_names;
      adio.analog_output_block_names = *analog_output_block_names;
      adio.digital_output_block_names = *digital_output_block_names;
      adio.digital_input_block_names = *digital_input_block_names;

      // Convert the analog inputs from proto.
      for (auto& name : *analog_input_block_names) {
        auto it = adio_in.analog_inputs().find(name);
        if (it == adio_in.analog_inputs().end()) {
          return absl::InvalidArgumentError(absl::StrCat(
              "Could find analog block '", name,
              "'. Names: ", absl::StrJoin(*analog_input_block_names, ", ")));
        }
        INTR_ASSIGN_OR_RETURN(auto rt_block, FromProto(it->second));
        adio.analog_inputs.push_back(rt_block);
      }

      // Convert the analog outputs from proto.
      for (auto& name : *analog_output_block_names) {
        auto it = adio_in.analog_outputs().find(name);
        if (it == adio_in.analog_outputs().end()) {
          return absl::InvalidArgumentError(absl::StrCat(
              "Could find analog block '", name,
              "'. Names: ", absl::StrJoin(*analog_output_block_names, ", ")));
        }
        INTR_ASSIGN_OR_RETURN(auto rt_block, FromProto(it->second));
        adio.analog_outputs.push_back(rt_block);
      }

      // Convert the digital inputs from proto.
      for (auto& name : *digital_input_block_names) {
        auto it = adio_in.digital_inputs().find(name);
        if (it == adio_in.digital_inputs().end()) {
          return absl::InvalidArgumentError(absl::StrCat(
              "Could find digital input block '", name,
              "'. Names: ", absl::StrJoin(*digital_input_block_names, ", ")));
        }
        INTR_ASSIGN_OR_RETURN(auto rt_block, FromProto(it->second));
        adio.digital_inputs.push_back(rt_block);
      }

      // Convert the digital outputs from proto.
      for (auto& name : *digital_output_block_names) {
        auto it = adio_in.digital_outputs().find(name);
        if (it == adio_in.digital_outputs().end()) {
          return absl::InvalidArgumentError(absl::StrCat(
              "Could find digital output block '", name,
              "'. Names: ", absl::StrJoin(*digital_output_block_names, ", ")));
        }
        INTR_ASSIGN_OR_RETURN(auto rt_block, FromProto(it->second));
        adio.digital_outputs.push_back(rt_block);
      }

      realtime_status.adio_state = adio;
    } else {
      LOG_EVERY_N_SEC(WARNING, 10)
          << "Ignoring 'ADIOState' as it's not possible to convert it to "
             "a RealtimeStatus without passing in the adio block names.";
    }
  }
  if (proto_status.has_current_control_mode()) {
    const auto& current_control_mode = proto_status.current_control_mode();
    switch (current_control_mode) {
      case intrinsic_proto::icon::PartControlMode::CONTROL_MODE_UNKNOWN:
        realtime_status.current_control_mode =
            ControlModeExporter::ControlMode::kUnknown;
        break;
      case intrinsic_proto::icon::PartControlMode::CONTROL_MODE_CYCLIC_POSITION:
        realtime_status.current_control_mode =
            ControlModeExporter::ControlMode::kCyclicPosition;
        break;
      case intrinsic_proto::icon::PartControlMode::CONTROL_MODE_CYCLIC_VELOCITY:
        realtime_status.current_control_mode =
            ControlModeExporter::ControlMode::kCyclicVelocity;
        break;
      case intrinsic_proto::icon::PartControlMode::CONTROL_MODE_CYCLIC_TORQUE:
        realtime_status.current_control_mode =
            ControlModeExporter::ControlMode::kCyclicTorque;
        break;
      case intrinsic_proto::icon::PartControlMode::CONTROL_MODE_HAND_GUIDING:
        realtime_status.current_control_mode =
            ControlModeExporter::ControlMode::kHandGuiding;
        break;
      default:
        return absl::InvalidArgumentError(
            absl::StrCat("Could not convert proto payload for Part to "
                         "RealtimePartStatus. PartControlMode is: ",
                         current_control_mode));
    }
  }

  if (proto_status.has_rangefinder_status()) {
    realtime_status.rangefinder_distance =
        proto_status.rangefinder_status().distance();
  }

  if (proto_status.inertial_measurement_unit_status().has_orientation()) {
    realtime_status.imu_sensed_orientation = intrinsic_proto::FromProto(
        proto_status.inertial_measurement_unit_status().orientation());
  }
  if (proto_status.inertial_measurement_unit_status().has_angular_velocity()) {
    realtime_status.imu_sensed_angular_velocity = intrinsic_proto::FromProto(
        proto_status.inertial_measurement_unit_status().angular_velocity());
  }
  if (proto_status.inertial_measurement_unit_status()
          .has_linear_acceleration()) {
    realtime_status.imu_sensed_linear_acceleration = intrinsic_proto::FromProto(
        proto_status.inertial_measurement_unit_status().linear_acceleration());
  }
  if (proto_status.cartesian_position_state().has_sensed_pose()) {
    INTR_ASSIGN_OR_RETURN(
        realtime_status.sensed_pose,
        FromProto(proto_status.cartesian_position_state().sensed_pose()));
  }

  return realtime_status;
}

intrinsic_proto::icon::PartStatus ToProto(
    const RealtimePartStatus& realtime_status) {
  INTRINSIC_ASSERT_NON_REALTIME();
  intrinsic_proto::icon::PartStatus proto_status;
  proto_status.set_timestamp_ns(
      absl::ToInt64Nanoseconds(realtime_status.time_since_timeslicer_epoch));
  proto_status.mutable_operational_status()->set_state(
      ToProto(ToOperationalState(realtime_status.operational_status.state)));
  if (!realtime_status.operational_status.fault_reason.empty()) {
    proto_status.mutable_operational_status()->set_fault_reason(
        realtime_status.operational_status.fault_reason);
  }
  size_t num_pos = 0;
  size_t num_vel = 0;
  size_t num_acc = 0;
  size_t num_torque = 0;
  size_t num_pos_commanded = 0;
  size_t num_vel_commanded = 0;
  size_t num_acc_commanded = 0;
  size_t num_torque_commanded = 0;
  if (realtime_status.sensed_position.has_value()) {
    num_pos = realtime_status.sensed_position->size();
  }
  if (realtime_status.sensed_velocity.has_value()) {
    num_vel = realtime_status.sensed_velocity->size();
  }
  if (realtime_status.sensed_acceleration.has_value()) {
    num_acc = realtime_status.sensed_acceleration->size();
  }
  if (realtime_status.sensed_torque.has_value()) {
    num_torque = realtime_status.sensed_torque->size();
  }
  if (realtime_status.position_commanded_last_cycle.has_value()) {
    num_pos_commanded = realtime_status.position_commanded_last_cycle->size();
  }
  if (realtime_status.velocity_commanded_last_cycle.has_value()) {
    num_vel_commanded = realtime_status.velocity_commanded_last_cycle->size();
  }
  if (realtime_status.acceleration_commanded_last_cycle.has_value()) {
    num_acc_commanded =
        realtime_status.acceleration_commanded_last_cycle->size();
  }
  if (realtime_status.torque_commanded_last_cycle.has_value()) {
    num_torque_commanded = realtime_status.torque_commanded_last_cycle->size();
  }
  // Fill the explicit members of the PartStatus proto
  std::array<size_t, 8> num_count{{
      num_pos,
      num_vel,
      num_acc,
      num_torque,
      num_pos_commanded,
      num_vel_commanded,
      num_acc_commanded,
      num_torque_commanded,
  }};
  size_t num_joints =
      *std::max_element(std::begin(num_count), std::end(num_count));

  proto_status.mutable_joint_states()->Reserve(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    auto* joint_state = proto_status.add_joint_states();
    if (i < num_pos) {  // never true if num_pos == 0
      joint_state->set_position_sensed(
          realtime_status.sensed_position->position[i]);
    }
    if (i < num_vel) {  // never true if num_vel == 0
      joint_state->set_velocity_sensed(
          realtime_status.sensed_velocity->velocity[i]);
    }
    if (i < num_acc) {  // never true if num_acc == 0
      joint_state->set_acceleration_sensed(
          realtime_status.sensed_acceleration->acceleration[i]);
    }
    if (i < num_torque) {  // never true if num_torque == 0
      joint_state->set_torque_sensed(realtime_status.sensed_torque->torque[i]);
    }
    if (i < num_pos_commanded) {  // never true if num_pos_commanded == 0
      joint_state->set_position_commanded_last_cycle(
          realtime_status.position_commanded_last_cycle->position[i]);
    }
    if (i < num_vel_commanded) {  // never true if num_vel_commanded == 0
      joint_state->set_velocity_commanded_last_cycle(
          realtime_status.velocity_commanded_last_cycle->velocity[i]);
    }
    if (i < num_acc_commanded) {  // never true if num_acc_commanded == 0
      joint_state->set_acceleration_commanded_last_cycle(
          realtime_status.acceleration_commanded_last_cycle->acceleration[i]);
    }
    if (i < num_torque_commanded) {  // never true if num_torque_commanded == 0
      joint_state->set_torque_commanded_last_cycle(
          realtime_status.torque_commanded_last_cycle->torque[i]);
    }
  }
  if (realtime_status.base_t_tip_sensed.has_value()) {
    const Pose3d& base_t_tip = realtime_status.base_t_tip_sensed.value();
    *proto_status.mutable_base_t_tip_sensed() = icon::ToProto(base_t_tip);
  }
  if (realtime_status.gripper_state.has_value()) {
    proto_status.mutable_gripper_state()->set_sensed_state(
        ToProto(realtime_status.gripper_state.value()));
  }

  if (realtime_status.linear_gripper_width_sensed.has_value()) {
    proto_status.mutable_linear_gripper_state()->set_sensed_width(
        *realtime_status.linear_gripper_width_sensed);
  }

  if (realtime_status.wrench_at_ft_uncompensated.has_value()) {
    *proto_status.mutable_wrench_at_ft_uncompensated() =
        ToProto(*realtime_status.wrench_at_ft_uncompensated);
  }

  if (realtime_status.wrench_at_ft.has_value()) {
    *proto_status.mutable_wrench_at_ft() =
        ToProto(*realtime_status.wrench_at_ft);
  }

  if (realtime_status.wrench_at_tip.has_value()) {
    *proto_status.mutable_wrench_at_tip() =
        ToProto(*realtime_status.wrench_at_tip);
  }

  if (realtime_status.wrench_stability_index.has_value()) {
    proto_status.set_wrench_stability_index(
        *realtime_status.wrench_stability_index);
  }

  if (realtime_status.base_twist_tip_sensed.has_value()) {
    *proto_status.mutable_base_twist_tip_sensed() =
        icon::ToProto(*realtime_status.base_twist_tip_sensed);
  }

  if (realtime_status.adio_state.has_value()) {
    const ADIO::ADIOState& adio_state = realtime_status.adio_state.value();
    size_t num_analog_inputs = adio_state.analog_input_block_names.size();
    for (size_t i = 0; i < num_analog_inputs; ++i) {
      intrinsic_proto::icon::AnalogBlock block;
      absl::Span<const AnalogBlock::Unit> units =
          adio_state.analog_inputs[i].Units();
      absl::Span<const double> values = adio_state.analog_inputs[i].Values();

      for (size_t k = 0; k < units.size(); ++k) {
        intrinsic_proto::icon::AnalogSignal signal;
        signal.set_value(values[k]);
        signal.set_unit(magic_enum::enum_name(units[k]));
        (*block.mutable_signals())[k] = signal;
      }

      (*proto_status.mutable_adio_state()
            ->mutable_analog_inputs())[adio_state.analog_input_block_names[i]] =
          std::move(block);
    }

    size_t num_analog_outputs = adio_state.analog_output_block_names.size();
    for (size_t i = 0; i < num_analog_outputs; ++i) {
      intrinsic_proto::icon::AnalogBlock block;
      absl::Span<const AnalogBlock::Unit> units =
          adio_state.analog_outputs[i].Units();
      absl::Span<const double> values = adio_state.analog_outputs[i].Values();

      for (size_t k = 0; k < units.size(); ++k) {
        intrinsic_proto::icon::AnalogSignal signal;
        signal.set_value(values[k]);
        signal.set_unit(magic_enum::enum_name(units[k]));
        (*block.mutable_signals())[k] = signal;
      }

      (*proto_status.mutable_adio_state()->mutable_analog_outputs())
          [adio_state.analog_output_block_names[i]] = std::move(block);
    }

    size_t num_digital_inputs = adio_state.digital_input_block_names.size();
    for (size_t i = 0; i < num_digital_inputs; ++i) {
      intrinsic_proto::icon::DioBlock block;
      absl::Span<const bool> values = adio_state.digital_inputs[i].Values();
      for (size_t k = 0; k < values.size(); ++k) {
        intrinsic_proto::icon::DigitalSignal signal;
        signal.set_value(values[k]);
        (*block.mutable_signals())[k] = signal;
      }

      (*proto_status.mutable_adio_state()->mutable_digital_inputs())
          [adio_state.digital_input_block_names[i]] = std::move(block);
    }

    size_t num_digital_outputs = adio_state.digital_output_block_names.size();
    for (size_t i = 0; i < num_digital_outputs; ++i) {
      intrinsic_proto::icon::DioBlock block;
      absl::Span<const bool> values = adio_state.digital_outputs[i].Values();
      for (size_t k = 0; k < values.size(); ++k) {
        intrinsic_proto::icon::DigitalSignal signal;
        signal.set_value(values[k]);
        (*block.mutable_signals())[k] = signal;
      }
      (*proto_status.mutable_adio_state()->mutable_digital_outputs())
          [adio_state.digital_output_block_names[i]] = std::move(block);
    }
  }

  if (realtime_status.current_control_mode.has_value()) {
    proto_status.set_current_control_mode(
        ToProto(*realtime_status.current_control_mode));
  }

  if (realtime_status.rangefinder_distance.has_value()) {
    proto_status.mutable_rangefinder_status()->set_distance(
        *realtime_status.rangefinder_distance);
  }

  if (realtime_status.imu_sensed_orientation.has_value()) {
    *proto_status.mutable_inertial_measurement_unit_status()
         ->mutable_orientation() =
        intrinsic::ToProto(*realtime_status.imu_sensed_orientation);
  }

  if (realtime_status.imu_sensed_linear_acceleration.has_value()) {
    *proto_status.mutable_inertial_measurement_unit_status()
         ->mutable_linear_acceleration() =
        ToVectorProto(*realtime_status.imu_sensed_linear_acceleration);
  }

  if (realtime_status.imu_sensed_angular_velocity.has_value()) {
    *proto_status.mutable_inertial_measurement_unit_status()
         ->mutable_angular_velocity() =
        ToVectorProto(*realtime_status.imu_sensed_angular_velocity);
  }

  if (realtime_status.sensed_pose.has_value()) {
    *proto_status.mutable_cartesian_position_state()->mutable_sensed_pose() =
        intrinsic::ToProto(*realtime_status.sensed_pose);
  }

  return proto_status;
}

::intrinsic_proto::icon::PartControlMode ToProto(
    ControlModeExporter::ControlMode mode) {
  switch (mode) {
    case ControlModeExporter::ControlMode::kUnknown:
      return intrinsic_proto::icon::PartControlMode::CONTROL_MODE_UNKNOWN;
    case ControlModeExporter::ControlMode::kCyclicPosition:
      return intrinsic_proto::icon::PartControlMode::
          CONTROL_MODE_CYCLIC_POSITION;
    case ControlModeExporter::ControlMode::kCyclicVelocity:
      return intrinsic_proto::icon::PartControlMode::
          CONTROL_MODE_CYCLIC_VELOCITY;
    case ControlModeExporter::ControlMode::kCyclicTorque:
      return intrinsic_proto::icon::PartControlMode::CONTROL_MODE_CYCLIC_TORQUE;
    case ControlModeExporter::ControlMode::kHandGuiding:
      return intrinsic_proto::icon::PartControlMode::CONTROL_MODE_HAND_GUIDING;
    default:
      return intrinsic_proto::icon::PartControlMode::CONTROL_MODE_UNKNOWN;
  }
  return intrinsic_proto::icon::PartControlMode::CONTROL_MODE_UNKNOWN;
}

::intrinsic_proto::icon::GripperState_SensedState ToProto(
    SimpleGripper::GripperState gripper_state) {
  switch (gripper_state) {
    case SimpleGripper::GripperState::kUnknown:
      return intrinsic_proto::icon::GripperState::SENSED_STATE_UNKNOWN;
    case SimpleGripper::GripperState::kGrasped:
      return intrinsic_proto::icon::GripperState::SENSED_STATE_HOLDING;
    case SimpleGripper::GripperState::kReleased:
      return intrinsic_proto::icon::GripperState::SENSED_STATE_FREE;
    default:
      return intrinsic_proto::icon::GripperState::SENSED_STATE_UNKNOWN;
  }
  return intrinsic_proto::icon::GripperState::SENSED_STATE_UNKNOWN;
}

absl::StatusOr<AnalogBlock> FromProto(
    const intrinsic_proto::icon::AnalogBlock& proto_block) {
  auto& signals = proto_block.signals();

  // Convert to signal units.
  std::vector<AnalogBlock::Unit> units;
  for (const auto& [index, signal] : signals) {
    if (index >= units.size()) {
      units.resize(index + 1);
    }
    auto enum_value = magic_enum::enum_cast<AnalogBlock::Unit>(signal.unit());
    if (enum_value) {
      units.at(index) = *enum_value;
    } else {
      return absl::InvalidArgumentError(absl::StrCat(
          "Could not convert unit string to enum: ", signal.unit()));
    }
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto rt_block, AnalogBlock::Create(units));

  // Convert to signal values.
  for (const auto& [index, signal] : signals) {
    rt_block.MutableValues().at(index) = signal.value();
  }
  return rt_block;
}

absl::StatusOr<DioBlock> FromProto(
    const intrinsic_proto::icon::DioBlock& proto_block) {
  uint32_t max_index = 0;
  // Go through all elements to find the max_index, which is needed to allocate
  // the correct size in the next step.
  for (const auto& [index, signal] : proto_block.signals()) {
    max_index = std::max(max_index, index);
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto rt_block, DioBlock::Create(max_index + 1));
  for (const auto& [index, signal] : proto_block.signals()) {
    rt_block.MutableValues().at(index) = signal.value();
  }
  return rt_block;
}

}  // namespace intrinsic::icon
