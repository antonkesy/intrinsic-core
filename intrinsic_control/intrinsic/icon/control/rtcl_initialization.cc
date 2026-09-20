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

#include "intrinsic/icon/control/rtcl_initialization.h"

#include <stddef.h>

#include <cstddef>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/fixed_array.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/state_variable_path_constants.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/behavior_override.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/parts/part_property_registry.h"
#include "intrinsic/icon/control/parts/proto/v1/realtime_part_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_part.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/control/parts/realtime_part_from_proto_factory_registry.h"
#include "intrinsic/icon/control/realtime_signal_storage.h"
#include "intrinsic/icon/control/realtime_signal_types.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/rtcl_action_factory_registry.h"
#include "intrinsic/icon/control/rtcl_action_instance.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/config/icon_main_config.pb.h"
#include "intrinsic/icon/server/signature_utils.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

// Looks up the ActionSignature for `action_type_name` and verifies
// slot-compatibility with a given part.
// Also ensures that the signature contains a boolean state variable with the
// name `intrinsic::kIsDone`.
absl::StatusOr<intrinsic_proto::icon::v1::ActionSignature>
LookupAndValidateSafetyActionSignature(
    absl::string_view action_type_name, absl::string_view part_name,
    const intrinsic_proto::icon::v1::PartConfig& part_config) {
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::icon::v1::ActionSignature action_signature,
      GetGlobalRtclActionFactoryRegistry().GetSignature(action_type_name));

  if (action_signature.part_slot_infos().size() != 1) {
    return absl::InvalidArgumentError(absl::Substitute(
        "Action type '$0' for Part '$1' must support exactly one slot, but "
        "wants: [$2]",
        action_signature.action_type_name(), part_name,
        absl::StrJoin(action_signature.part_slot_infos(), ", ",
                      absl::PairFormatter(
                          absl::AlphaNumFormatter(), "=",
                          [](std::string* out,
                             const intrinsic_proto::icon::v1::ActionSignature::
                                 PartSlotInfo& slot_info) {
                            absl::StrAppend(out, "{", slot_info, "}");
                          }))));
  }

  auto done_info = absl::c_find_if(
      action_signature.state_variable_infos(),
      [](const ::intrinsic_proto::icon::v1::ActionSignature::StateVariableInfo&
             state_variable) {
        return state_variable.state_variable_name() == kIsDone;
      });
  if (done_info == action_signature.state_variable_infos().end()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Action type '", action_type_name,
                     "' does not have the state variable '", kIsDone,
                     "', which is required for safety actions"));
  }
  if (done_info->type() != ::intrinsic_proto::icon::v1::ActionSignature::
                               StateVariableInfo::TYPE_BOOL) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Action type '", action_type_name, "' has the state variable '",
        kIsDone, "', but with the wrong type (should be ",
        ::intrinsic_proto::icon::v1::ActionSignature::StateVariableInfo::
            Type_Name(::intrinsic_proto::icon::v1::ActionSignature::
                          StateVariableInfo::TYPE_BOOL),
        ", but is ",
        ::intrinsic_proto::icon::v1::ActionSignature::StateVariableInfo::
            Type_Name(done_info->type()),
        ")"));
  }

  // Find all state variable names that use the prefix that is reserved to
  // identify state variable paths. Then, report all invalid names.
  std::vector<::intrinsic_proto::icon::v1::ActionSignature_StateVariableInfo>
      violating_action_variable_infos;
  absl::c_copy_if(
      action_signature.state_variable_infos(),
      std::back_inserter(violating_action_variable_infos),
      [](const ::intrinsic_proto::icon::v1::ActionSignature::StateVariableInfo&
             state_variable) {
        return absl::StartsWith(state_variable.state_variable_name(),
                                kStateVariablePathPrefix);
      });
  if (!violating_action_variable_infos.empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Action '", action_signature.action_type_name(),
                     "' uses the reserved state variable path prefix '",
                     kStateVariablePathPrefix,
                     "' in the beginning of the action state variable name(s) ",
                     absl::StrJoin(violating_action_variable_infos, ", ",
                                   [](std::string* output, auto variable_info) {
                                     absl::SubstituteAndAppend(
                                         output, "'$0'",
                                         variable_info.state_variable_name());
                                   }),
                     "."));
  }

  if (const auto result = PartCompatibleWithSlot(
          part_config, action_signature.part_slot_infos().begin()->second);
      !result.Compatible()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Action type '", action_signature.action_type_name(),
                     "' is not compatible with Part '", part_name,
                     "'. Explanation: ", result.Explain()));
  }
  return action_signature;
}

absl::StatusOr<std::unique_ptr<StreamingIoStorage>> MakeStreamingIoStorage(
    ActionInstanceId id,
    intrinsic_proto::icon::v1::ActionSignature action_signature) {
  return std::make_unique<StreamingIoStorage>(id, action_signature);
}

// Creates an `ActionFactoryContext` for the provided `ActionSignature`.
ActionFactoryContext MakeActionFactoryContext(
    ActionInstanceId id,
    intrinsic_proto::icon::v1::ActionSignature action_signature,
    const ::intrinsic_proto::icon::v1::ServerConfig& server_config,
    size_t part_index, const intrinsic_proto::icon::v1::PartConfig& part_config,
    const absl::flat_hash_map<std::string, RealtimeSignalId>&
        realtime_signal_name_to_id,
    StreamingIoStorage& streaming_io_storage) {
  absl::flat_hash_map<std::string, SlotInfo> slot_info_map{
      {action_signature.part_slot_infos().begin()->first,
       SlotInfo{
           .config = part_config,
           .slot_id = RealtimeSlotId(part_index),
       }}};

  // We discard this map, because there is no way for a user to get a handle to
  // safety actions, so it's impossible to access anyway.
  absl::flat_hash_map<ActionInstanceId, JointTrajectoryPVA> trajectory_map;
  return ActionFactoryContext{server_config,        action_signature,
                              slot_info_map,        realtime_signal_name_to_id,
                              streaming_io_storage, id,
                              trajectory_map};
}

// Creates an `RtclActionInstance` and validates the `ActionFactoryContext`.
absl::StatusOr<RtclActionInstance> MakeRtclActionInstance(
    ActionInstanceId id, ActionFactoryContext context,
    absl::string_view action_type_name, size_t num_parts, size_t part_index,
    const intrinsic_proto::icon::v1::PartConfig& part_config,
    const RealtimeSignalStorage signal_storage,
    StreamingIoStorage& streaming_io_storage,
    const intrinsic_proto::icon::v1::ActionSignature& action_signature) {
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<RtclActionInterface> action_instance,
      GetGlobalRtclActionFactoryRegistry().CallFactory(
          action_type_name, google::protobuf::Any(), context));

  INTR_RETURN_IF_ERROR(context.Validate());

  auto realtime_streaming_io_storage =
      std::make_unique<RealtimeStreamingIoStorage>(
          FromStreamingIoStorage(streaming_io_storage));

  // Construct a vector of mutable bools that we then move into the FixedArray
  // we need to create an RtclActionInstance below.
  // This intermediate step is necessary because RtclActionInstance has a
  // FixedArray of *const* bool, but we need to change the visibility for the
  // Part the Action uses before assigning the visibility vector.
  std::vector<bool> visibility_by_slot_id_vector(num_parts, false);
  visibility_by_slot_id_vector[part_index] = true;
  auto visibility_by_slot_id = std::make_unique<absl::FixedArray<bool>>(
      std::make_move_iterator(visibility_by_slot_id_vector.begin()),
      std::make_move_iterator(visibility_by_slot_id_vector.end()));

  INTR_ASSIGN_OR_RETURN(const auto behavior_override_capabilities_map,
                        MakeBehaviorOverrideSupportArray(action_signature));

  return RtclActionInstance{
      .streaming_io_storage = std::move(realtime_streaming_io_storage),
      .realtime_signal_storage =
          std::make_unique<RealtimeSignalStorage>(signal_storage),
      .action = std::move(action_instance),
      .visibility_by_slot_id = std::move(visibility_by_slot_id),
      .id = id,
      .supported_behavior_overrides_by_enum_value =
          std::make_unique<absl::FixedArray<bool>>(
              behavior_override_capabilities_map),
  };
}

// Builds a PartAndSafetyAction struct for `part_name`, which contains an
// instance of `safety_action_type_name`.
// `num_parts` and `part_index` are used to fill a SlotInfo object and slot
// visibility array.
// NOTE: While a RealtimeSlotId is opaque to the Action instance itself, this
// function is aware of (establishes, in fact) the internal implementation
// detail that the values of RealtimeSlotIds are exactly indices into an array
// of Parts.
absl::StatusOr<PartAndSafetyAction> MakeSafetyAction(
    const ::intrinsic_proto::icon::v1::ServerConfig& server_config,
    absl::string_view part_name, absl::string_view safety_action_type_name,
    size_t num_parts, size_t part_index,
    const intrinsic_proto::icon::v1::PartConfig& part_config,
    RealtimePart part) {
  // Create RtclActionInstance for safety action
  ActionInstanceId safety_action_instance_id(1);
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::icon::v1::ActionSignature safety_action_signature,
      LookupAndValidateSafetyActionSignature(safety_action_type_name, part_name,
                                             part_config));
  INTR_ASSIGN_OR_RETURN(auto safety_streaming_io_storage,
                        MakeStreamingIoStorage(safety_action_instance_id,
                                               safety_action_signature));
  INTR_ASSIGN_OR_RETURN(SignalStorageAndIdMap signal_storage_and_id_map,
                        CreateRealtimeSignalStorage(safety_action_signature));
  auto safety_action_context = MakeActionFactoryContext(
      safety_action_instance_id, safety_action_signature, server_config,
      part_index, part_config, signal_storage_and_id_map.signal_id_map,
      *safety_streaming_io_storage);
  INTR_ASSIGN_OR_RETURN(
      RtclActionInstance safety_action_instance,
      MakeRtclActionInstance(
          safety_action_instance_id, safety_action_context,
          safety_action_type_name, num_parts, part_index, part_config,
          signal_storage_and_id_map.signal_storage,
          *safety_streaming_io_storage, safety_action_signature));

  return PartAndSafetyAction{
      .part = std::move(part),
      .safety_action_io_storage = std::move(safety_streaming_io_storage),
      .safety_action = std::move(safety_action_instance),
  };
}

}  // namespace

// Proto version.
absl::StatusOr<RtclPartsAndConfigs> InitializePartsFromProto(
    const ::intrinsic_proto::icon::v1::ServerConfig& server_config,
    const absl::flat_hash_map<std::string,
                              intrinsic_proto::v1::RealtimePartConfig>&
        realtime_part_configs_by_part_name,
    const Context& context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  RtclPartsAndConfigs parts_and_configs;
  for (const auto& [part_name, part_config] :
       realtime_part_configs_by_part_name) {
    auto part_factory = GetGlobalRealtimePartFromProtoFactoryRegistry().Get(
        part_config.part_type_name());

    if (!part_factory) {
      return absl::NotFoundError(absl::StrCat(
          "No factory for Part type '", part_config.part_type_name(),
          "', did you forget to register it? (If using a _register library, "
          "make sure it has alwayslink=True set.)"));
    }

    std::vector<PartPropertyInitialData> property_data;
    PartPropertyRegistry part_property_registry(property_data);
    std::string hardware_resource_name = part_config.hardware_resource_name();
    if (hardware_resource_name.empty()) {
      hardware_resource_name = context.GetRuntimeOptions().resource_id;
    }
    INTR_ASSIGN_OR_RETURN(
        PartPtrAndPartConfig part_and_config,
        part_factory(
            {
                .part_name = part_name,
                .context = context,
                .property_registry = part_property_registry,
                .control_frequency_hz = server_config.frequency_hz(),
                .hardware_resource_name = hardware_resource_name,
            },
            part_config.config()),
        _ << "Error building Part '" << part_name << "'.");
    parts_and_configs.part_property_initial_data_in_index_order.emplace_back(
        std::move(property_data));
    INTR_ASSIGN_OR_RETURN(
        PartAndSafetyAction part_and_safety_action,
        MakeSafetyAction(
            server_config, part_name,
            /*safety_action_type_name=*/part_config.safety_action_type_name(),
            /*num_parts=*/realtime_part_configs_by_part_name.size(),
            /*part_index=*/parts_and_configs.parts_with_safety_actions.size(),
            part_and_config.config,
            RealtimePart(/*name=*/part_name,
                         /*part=*/std::move(part_and_config.part_ptr))));

    parts_and_configs.parts_with_safety_actions.emplace_back(
        std::move(part_and_safety_action));
    parts_and_configs.part_configs.emplace_back(
        std::move(part_and_config.config));
  }

  return parts_and_configs;
}

}  // namespace intrinsic::icon
