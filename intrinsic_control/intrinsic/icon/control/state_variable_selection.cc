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

#include "intrinsic/icon/control/state_variable_selection.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "gloop/util/gtl/iterator_adaptors.h"
#include "intrinsic/icon/common/state_variable_path_constants.h"
#include "intrinsic/icon/common/state_variable_path_util.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/realtime_part_status.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/state_variable_path_parsing.h"
#include "intrinsic/icon/control/state_variable_selection_types.h"
#include "intrinsic/icon/control/state_variable_selection_util.h"
#include "intrinsic/icon/proto/safety_status_conversion.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/twist.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {
namespace {

// Convenience function to create an out of range error with a consistent
// message for all out-of-range errors in this translation unit in RT functions.
RealtimeStatus RTOutOfRangeErrorBuilder(size_t requested_index, size_t size,
                                        absl::string_view container_name)
    INTRINSIC_CHECK_REALTIME_SAFE {
  return OutOfRangeError(
      RealtimeStatus::StrCat("Index ", requested_index, " < size ", size,
                             " failed for container '", container_name, "'"));
}

// Convenience function to create an out of range error with a consistent
// message for all out-of-range errors in this translation unit.
absl::Status OutOfRangeErrorBuilder(size_t requested_index, size_t size,
                                    absl::string_view container_name) {
  INTRINSIC_ASSERT_NON_REALTIME();
  return absl::OutOfRangeError(
      absl::StrCat("Index ", requested_index, " < size ", size,
                   " failed for container '", container_name, "'"));
}

// Convenience function hiding the boilerplate code for error handling.
absl::StatusOr<size_t> GetNodeIndex(const StateVariablePathNode& node) {
  INTRINSIC_ASSERT_NON_REALTIME();
  if (!node.index.has_value()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "No index found for node '", node.name, "', but it requires one."));
  }
  return *node.index;
}

// Creates a realtime safe std::function object for one of the arm part status
// fields, depending on `part_field_nodes`. The arm part must be at `part_index`
// in the realtime thread. `part_field_nodes` are all nodes without the already
// used fields (the part instance name and the part type name). `part_name` is
// only used for error messages.
absl::StatusOr<StateVariableFieldSelectionFunction> CreateArmPartStatusFunction(
    absl::string_view part_name, size_t part_index,
    absl::Span<StateVariablePathNode> part_field_nodes) {
  const size_t expected_number_of_nodes = 1;
  if (part_field_nodes.size() != expected_number_of_nodes) {
    return absl::InvalidArgumentError(
        "Unexpected number of state variable path nodes!");
  }
  const FixedString<kMaxNodeNameLength> part_name_copy(part_name);
  const auto& node = part_field_nodes.front();
  const absl::string_view field_name = node.name;

  if (field_name == kSensedPositionNodeName) {
    INTR_ASSIGN_OR_RETURN(auto joint_index, GetNodeIndex(node));
    return CreateSelectionFunctionForVector(
        &RealtimePartStatus::sensed_position, &JointStateP::position,
        part_index, joint_index, part_name, kSensedPositionNodeName);
  } else if (field_name == kSensedVelocityNodeName) {
    INTR_ASSIGN_OR_RETURN(auto joint_index, GetNodeIndex(node));
    return CreateSelectionFunctionForVector(
        &RealtimePartStatus::sensed_velocity, &JointStateV::velocity,
        part_index, joint_index, part_name, kSensedVelocityNodeName);
  } else if (field_name == kSensedAccelerationNodeName) {
    INTR_ASSIGN_OR_RETURN(auto joint_index, GetNodeIndex(node));
    return CreateSelectionFunctionForVector(
        &RealtimePartStatus::sensed_acceleration, &JointStateA::acceleration,
        part_index, joint_index, part_name, kSensedAccelerationNodeName);
  } else if (field_name == kSensedTorqueNodeName) {
    INTR_ASSIGN_OR_RETURN(auto joint_index, GetNodeIndex(node));
    return CreateSelectionFunctionForVector(
        &RealtimePartStatus::sensed_torque, &JointStateT::torque, part_index,
        joint_index, part_name, kSensedTorqueNodeName);
  } else if (field_name == kBaseTwistTipSensedNodeNodeName) {
    INTR_ASSIGN_OR_RETURN(auto twist_index, GetNodeIndex(node));
    return [part_index, twist_index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const Twist* twist,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::base_twist_tip_sensed,
                             part_index, absl::string_view(part_name_copy),
                             kBaseTwistTipSensedNodeNodeName));
      if (twist_index >= twist->size()) {
        return RTOutOfRangeErrorBuilder(twist_index, twist->size(), "twist");
      }
      return PartStatusVariant((*twist)(twist_index));
    };
  } else if (field_name == kBaseLinearVelocityTipSensedNodeName) {
    return [part_index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const Twist* twist,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::base_twist_tip_sensed,
                             part_index, absl::string_view(part_name_copy),
                             kBaseLinearVelocityTipSensedNodeName));
      return PartStatusVariant(twist->head<3>().norm());
    };
  } else if (field_name == kBaseAngularVelocityTipSensedNodeName) {
    return [part_index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const Twist* twist,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::base_twist_tip_sensed,
                             part_index, absl::string_view(part_name_copy),
                             kBaseAngularVelocityTipSensedNodeName));
      return PartStatusVariant(twist->tail<3>().norm());
    };
  } else if (field_name == kCurrentControlModeNodeName) {
    return [part_index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const ControlModeExporter::ControlMode* mode,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::current_control_mode,
                             part_index, absl::string_view(part_name_copy),
                             kCurrentControlModeNodeName));
      return PartStatusVariant(static_cast<int64_t>(ToProto(*mode)));
    };
  }
  return absl::InvalidArgumentError(
      "Could not find a suitable match for state_variable_path in arm part.");
}

// Creates a realtime safe std::function object for one of the force torque
// (FT) sensor part status fields, depending on `part_field_nodes` The FT part
// must be at `part_index`. `part_field_nodes` are all nodes without the already
// used fields (the part instance name and the part type name). `part_name` is
// only used for error messages.
absl::StatusOr<StateVariableFieldSelectionFunction>
CreateFTSensorStatusFunction(
    absl::string_view part_name, size_t part_index,
    absl::Span<StateVariablePathNode> part_field_nodes) {
  const size_t expected_number_of_nodes = 1;
  if (part_field_nodes.size() != expected_number_of_nodes) {
    return absl::InvalidArgumentError(
        "Unexpected number of state variable path nodes!");
  }
  const FixedString<kMaxNodeNameLength> part_name_copy(part_name);
  const absl::string_view node_name = part_field_nodes.front().name;
  if (node_name == kWrenchAtTipNodeName) {
    INTR_ASSIGN_OR_RETURN(const size_t index,
                          GetNodeIndex(part_field_nodes.front()));
    return [part_index, index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const Wrench* wrench,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::wrench_at_tip, part_index,
                             absl::string_view(part_name_copy),
                             kWrenchAtTipNodeName));
      if (index >= wrench->size()) {
        return RTOutOfRangeErrorBuilder(index, wrench->size(), "wrench");
      }
      return PartStatusVariant((*wrench)(index));
    };
  } else if (node_name == kForceMagnitudeAtTipNodeName) {
    return [part_index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const Wrench* wrench,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::wrench_at_tip, part_index,
                             absl::string_view(part_name_copy),
                             kForceMagnitudeAtTipNodeName));
      return PartStatusVariant(wrench->head<3>().norm());
    };
  } else if (node_name == kTorqueMagnitudeAtTipNodeName) {
    return [part_index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const Wrench* wrench,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::wrench_at_tip, part_index,
                             absl::string_view(part_name_copy),
                             kTorqueMagnitudeAtTipNodeName));
      return PartStatusVariant(wrench->tail<3>().norm());
    };
  } else if (node_name == kWrenchStabilityIndexNodeName) {
    return [part_index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const double* wrench_stability_index,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::wrench_stability_index,
                             part_index, absl::string_view(part_name_copy),
                             kWrenchStabilityIndexNodeName));
      return PartStatusVariant(*wrench_stability_index);
    };
  }

  return absl::NotFoundError(absl::StrCat(
      "No suitable field found for FT sensor node name '", node_name, "'"));
}

// Creates a realtime safe std::function object to select an ADIO signal from an
// ADIO part. The function works for digital (bool) or analog (double) signals.
// Use this function to avoid repeating boiler plate code for safely accessing
// the value. `field_ptr` is a member pointer to the ADIO block that should be
// selected. `part_index` refers to the index of an ADIO part. `block_name` must
// be the name of the block to select. It must be part of `block_names`.
// `signal_index` refers to index of the signal in the signal block of the ADIO
// part.
// `block_names` must contain the names all blocks available in part
// `part_name`/`part_index`.
// `part_name` is used for error messages only and should be the name of the
// part at `part_index`.
template <typename FieldPtr>
absl::StatusOr<StateVariableFieldSelectionFunction>
CreateADIOSignalSelectionFunction(
    FieldPtr field_ptr, size_t part_index,
    const FixedString<kMaxNodeNameLength>& block_name, size_t signal_index,
    absl::Span<const std::string> block_names, absl::string_view part_name) {
  const FixedString<kMaxNodeNameLength> part_name_copy(part_name);
  // Extract the index of the `block_name` from the given adio block names.
  auto it = absl::c_find(block_names, absl::string_view(block_name));
  if (it == block_names.end()) {
    return absl::NotFoundError(absl::StrCat(
        "Could not find adio block name '", block_name,
        "' in ADIO status of part '", part_name_copy,
        "'. Available blocks: ", absl::StrJoin(block_names, ", ")));
  }
  const size_t block_index = it - block_names.begin();

  return [part_index, block_index, signal_index, part_name_copy, block_name,
          field_ptr](const StateVariableFieldSelectionData& selection_data)
             -> RealtimeStatusOr<PartStatusVariant> {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const ADIO::ADIOState* block,
        SelectCheckedValue(selection_data.robot_status,
                           &RealtimePartStatus::adio_state, part_index,
                           block_name, part_name_copy));
    // Access the chosen (by string) block array for a generic signal type
    // (e.g. digital_input, analog_input, digital_output).
    const auto& blocks = AccessClassMember(*block, field_ptr);
    if (block_index >= blocks.size()) {
      return RTOutOfRangeErrorBuilder(block_index, blocks.size(), block_name);
    }
    const auto& signals = blocks[block_index].Values();
    // Access a single signal from a block.
    if (signal_index >= signals.size()) {
      return RTOutOfRangeErrorBuilder(
          signal_index, signals.size(),
          RealtimeStatus::StrCat(block_name, "_signals"));
    }
    return PartStatusVariant(signals[signal_index]);
  };
}

// Creates a realtime safe std::function object for one of the ADIO part status
// fields, depending on `part_field_nodes`. The ADIO part must be at
// `part_index`. `part_field_nodes` are all nodes without the already used
// fields (the part instance name and the part type name). `part_name` is only
// used for error messages.
absl::StatusOr<StateVariableFieldSelectionFunction> CreateADIOStatusFunction(
    absl::string_view part_name, size_t part_index,
    absl::Span<StateVariablePathNode> part_field_nodes,
    const StateVariableFieldSelectionData& selection_data) {
  const size_t expected_number_of_nodes = 2;
  if (part_field_nodes.size() != expected_number_of_nodes) {
    return absl::InvalidArgumentError(
        "Unexpected number of state variable path nodes!");
  }
  const size_t signal_node_index = 1;
  const FixedString<kMaxNodeNameLength> block_name(
      part_field_nodes[signal_node_index].name);
  const std::optional<size_t> signal_index_opt =
      part_field_nodes[signal_node_index].index;
  if (!signal_index_opt.has_value()) {
    return absl::InvalidArgumentError(
        absl::StrCat("'", part_field_nodes[signal_node_index].name,
                     "' does have have an index field!"));
  }
  size_t signal_index = *signal_index_opt;
  if (part_index >= selection_data.robot_status.part_statuses.size()) {
    return OutOfRangeErrorBuilder(
        part_index, selection_data.robot_status.part_statuses.size(),
        "part_statuses");
  }

  const std::optional<ADIO::ADIOState> adio_optional =
      selection_data.robot_status.part_statuses[part_index].adio_state;
  if (!adio_optional.has_value()) {
    return absl::NotFoundError(
        absl::StrCat("ADIO value not set in RealtimePartStatus for part '",
                     part_name, "'!"));
  }

  // Now, create and return a std::function object for access to the different
  // adio types.
  const absl::string_view node_name = part_field_nodes.front().name;
  if (node_name == kDigitalInputNodeName) {
    return CreateADIOSignalSelectionFunction(
        &ADIO::ADIOState::digital_inputs, part_index, block_name, signal_index,
        adio_optional->digital_input_block_names, part_name);
  } else if (node_name == kDigitalOutputNodeName) {
    return CreateADIOSignalSelectionFunction(
        &ADIO::ADIOState::digital_outputs, part_index, block_name, signal_index,
        adio_optional->digital_output_block_names, part_name);
  } else if (node_name == kAnalogInputNodeName) {
    return CreateADIOSignalSelectionFunction(
        &ADIO::ADIOState::analog_inputs, part_index, block_name, signal_index,
        adio_optional->analog_input_block_names, part_name);
  } else if (node_name == kAnalogOutputNodeName) {
    return CreateADIOSignalSelectionFunction(
        &ADIO::ADIOState::analog_outputs, part_index, block_name, signal_index,
        adio_optional->analog_output_block_names, part_name);
  }

  return absl::NotFoundError(absl::StrCat(
      "No suitable field found in ADIO part for node name '", node_name, "'"));
}

// Creates a realtime safe std::function object for one of the Gripper part
// status fields, depending on `part_field_nodes`. The Gripper part must be at
// `part_index`. `part_field_nodes` are all nodes without the already used
// fields (the part instance name and the part type name). `part_name` is only
// used for error messages.
absl::StatusOr<StateVariableFieldSelectionFunction> CreateGripperStatusFunction(
    absl::string_view part_name, size_t part_index,
    absl::Span<StateVariablePathNode> part_field_nodes) {
  const size_t expected_number_of_nodes = 1;
  if (part_field_nodes.size() != expected_number_of_nodes) {
    return absl::InvalidArgumentError(
        "Unexpected number of state variable path nodes!");
  }
  const FixedString<kMaxNodeNameLength> part_name_copy(part_name);
  const absl::string_view node_name = part_field_nodes.front().name;
  if (node_name == kGripperSensedStateNodeName) {
    return [part_index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const SimpleGripper::GripperState* status,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::gripper_state, part_index,
                             absl::string_view(part_name_copy),
                             kGripperSensedStateNodeName));
      return PartStatusVariant(static_cast<int64_t>(ToProto(*status)));
    };
  } else if (node_name == kGripperOpeningWidthNodeName) {
    return [part_index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const double* status,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::linear_gripper_width_sensed,
                             part_index, absl::string_view(part_name_copy),
                             kGripperOpeningWidthNodeName));
      return PartStatusVariant(*status);
    };
  }

  return absl::NotFoundError("No suitable field found in gripper part.");
}

// Creates a realtime safe std::function object for one of the Rangefinder part
// status fields, depending on `part_field_nodes`. The Rangefinder part must be
// at `part_index`. `part_field_nodes` are all nodes without the already used
// fields (the part instance name and the part type name). `part_name` is only
// used for error messages.
absl::StatusOr<StateVariableFieldSelectionFunction> CreateRangeFinderFunction(
    absl::string_view part_name, size_t part_index,
    absl::Span<StateVariablePathNode> part_field_nodes) {
  const size_t expected_number_of_nodes = 1;
  if (part_field_nodes.size() != expected_number_of_nodes) {
    return absl::InvalidArgumentError(
        "Unexpected number of state variable path nodes!");
  }
  const FixedString<kMaxNodeNameLength> part_name_copy(part_name);
  const absl::string_view node_name = part_field_nodes.front().name;
  if (node_name == kRangefinderDistanceNodeName) {
    return [part_index, part_name_copy](
               const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const double* distance,
          SelectCheckedValue(selection_data.robot_status,
                             &RealtimePartStatus::rangefinder_distance,
                             part_index, absl::string_view(part_name_copy),
                             kRangefinderDistanceNodeName));
      return PartStatusVariant(*distance);
    };
  }

  return absl::NotFoundError("No suitable field found in rangefinder part.");
}

// Creates a realtime safe std::function object for one of the safety status
// fields, depending on `part_field_nodes`. `part_field_nodes` are all nodes
// without the already used fields (the part type name).
absl::StatusOr<StateVariableFieldSelectionFunction> CreateSafetyStatusFunction(
    absl::Span<StateVariablePathNode> part_field_nodes) {
  const size_t expected_number_of_nodes = 1;
  if (part_field_nodes.size() != expected_number_of_nodes) {
    return absl::InvalidArgumentError(
        "Unexpected number of state variable path nodes!");
  }
  if (part_field_nodes.front().name == kEnableButtonStatusNodeName) {
    return [](const StateVariableFieldSelectionData& selection_data)
               -> RealtimeStatusOr<PartStatusVariant> {
      return PartStatusVariant(static_cast<int64_t>(ToProto(
          selection_data.robot_status.safety_status.enable_button_status)));
    };
  }
  return absl::NotFoundError("No suitable field found in safety status.");
}

}  // namespace

// Redirects the call to part specific functions. Also checks the `part_name`
// actually exists in the current session.
absl::StatusOr<StateVariableFieldSelectionFunction>
RobotSystemStateFieldSelector::CreateFunctionForParts(
    absl::string_view part_name, absl::string_view part_type_name,
    size_t part_index, absl::Span<StateVariablePathNode> part_field_nodes) {
  if (part_type_name == kArmTypeNodeName) {
    return CreateArmPartStatusFunction(part_name, part_index, part_field_nodes);
  } else if (part_type_name == kFTTypeNodeName) {
    return CreateFTSensorStatusFunction(part_name, part_index,
                                        part_field_nodes);
  } else if (part_type_name == kADIOTypeNodeName) {
    return CreateADIOStatusFunction(part_name, part_index, part_field_nodes,
                                    selection_data_);
  } else if (part_type_name == kGripperTypeNodeName) {
    return CreateGripperStatusFunction(part_name, part_index, part_field_nodes);
  } else if (part_type_name == kRangefinderNodeName) {
    return CreateRangeFinderFunction(part_name, part_index, part_field_nodes);
  }
  return absl::NotFoundError(
      absl::StrCat("Did not find any known part type for '", part_type_name,
                   "' with name '", part_name, "'."));
}

absl::StatusOr<StateVariableFieldSelectionFunction>
RobotSystemStateFieldSelector::CreateRobotSystemStateFieldSelector(
    absl::string_view state_variable_path,
    std::optional<size_t>& required_part_index) {
  if (state_variable_path.empty()) {
    return InvalidArgumentError("State variable path must not be empty!");
  }

  INTR_ASSIGN_OR_RETURN(
      std::vector<StateVariablePathNode> state_variable_path_nodes,
      PathToNodeVec(state_variable_path));
  const size_t minimum_number_of_path_nodes = 2;
  if (state_variable_path_nodes.size() < minimum_number_of_path_nodes) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Could not find at least ", minimum_number_of_path_nodes,
        " nodes in path '", state_variable_path,
        "'. Expected format is <part_name>[.<part_type>].<nodes...>"));
  }
  absl::StatusOr<StateVariableFieldSelectionFunction> function_or_status;
  const absl::string_view first_node = state_variable_path_nodes[0].name;
  if (first_node == kSafetyTypeNodeName) {
    required_part_index = std::nullopt;
    function_or_status = CreateSafetyStatusFunction(
        absl::MakeSpan(state_variable_path_nodes).subspan(1));
  } else {  // All fields that belong to a part status are parsed here
    const absl::string_view part_name = state_variable_path_nodes[0].name;
    const absl::string_view part_type_name = state_variable_path_nodes[1].name;
    const size_t minimum_number_of_part_path_nodes = 3;
    if (state_variable_path_nodes.size() < minimum_number_of_part_path_nodes) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Could not find at least ", minimum_number_of_part_path_nodes,
          " nodes in path '", state_variable_path,
          "'. Expected format is <part_name>.<part_type>.<fields>"));
    }
    // Every part status field needs to have a correct part name that maps to a
    // part index.
    const auto it = part_name_to_index_map_.find(part_name);
    if (it == part_name_to_index_map_.end()) {
      return absl::NotFoundError(absl::StrCat(
          "Could not find part with name '", part_name, "'!\nAvailable parts: ",
          absl::StrJoin(gtl::key_view(part_name_to_index_map_), ", ")));
    }
    const size_t part_index = it->second;
    required_part_index = part_index;
    function_or_status = CreateFunctionForParts(
        part_name, part_type_name, part_index,
        absl::MakeSpan(state_variable_path_nodes).subspan(2));
  }

  if (!function_or_status.ok()) {
    return function_or_status;
  }
  RealtimeStatusOr<PartStatusVariant> part_status_result =
      (*function_or_status)(selection_data_);
  if (!part_status_result.ok()) {
    return absl::Status(
        part_status_result.status().code(),
        absl::StrCat(
            "Execution of state variable selection function failed with "
            "error: ",
            part_status_result.status().ToString()));
  }
  return function_or_status;
}

}  // namespace intrinsic::icon
