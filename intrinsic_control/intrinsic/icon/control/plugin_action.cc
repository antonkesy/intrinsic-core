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

#include "intrinsic/icon/control/plugin_action.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "absl/functional/bind_front.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/c_api/c_plugin_api.h"
#include "intrinsic/icon/control/c_api/c_realtime_status.h"
#include "intrinsic/icon/control/c_api/c_rtcl_action.h"
#include "intrinsic/icon/control/c_api/c_types.h"
#include "intrinsic/icon/control/c_api/convert_c_realtime_status.h"
#include "intrinsic/icon/control/c_api/wrappers/action_factory_context_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/feature_interface_wrappers.h"
#include "intrinsic/icon/control/c_api/wrappers/realtime_signal_access_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/realtime_slot_map_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/streaming_io_realtime_access_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/string_wrapper.h"
#include "intrinsic/icon/control/rtcl_action_factory_registry.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

IntrinsicIconRealtimeStatus RegisterActionFromPlugin(
    int64_t icon_api_version, IntrinsicIconStringView action_type_name,
    IntrinsicIconStringView action_signature_proto,
    IntrinsicIconRtclActionVtable function_table) {
  absl::string_view action_type_name_view(action_type_name.data,
                                          action_type_name.size);
  intrinsic_proto::icon::v1::ActionSignature signature;
  absl::string_view action_signature_proto_string(action_signature_proto.data,
                                                  action_signature_proto.size);
  if (!signature.ParseFromString(action_signature_proto_string)) {
    return FromAbslStatus(absl::InvalidArgumentError(
        absl::StrCat("Failed to parse Action Signature proto for Action type '",
                     action_type_name_view, "'.")));
  }
  if (!GetGlobalRtclActionFactoryRegistry().RegisterGeneric(
          action_type_name_view,
          absl::bind_front(&PluginAction::Create, std::move(function_table)),
          std::move(signature))) {
    return FromAbslStatus(absl::AlreadyExistsError(absl::StrCat(
        "Failed to register factory for Action type'", action_type_name_view,
        "'. This is most likely because another Action type of the same name "
        "has already been registered.")));
  }
  return FromAbslStatus(absl::OkStatus());
}

absl::Status RegisterPluginActions(
    IntrinsicIconRegisterActionTypes register_actions_fn) {
  return ToAbslStatus(register_actions_fn(&RegisterActionFromPlugin));
}

// static
absl::StatusOr<std::unique_ptr<PluginAction>> PluginAction::Create(
    IntrinsicIconRtclActionVtable plugin_function_table,
    const google::protobuf::Any& params, ActionFactoryContext& context) {
  IntrinsicIconServerFunctions server_functions{
      .action_factory_context = GetActionFactoryContextVtable(),
      .realtime_slot_map = GetRealtimeSlotMapVtable(),
      .feature_interfaces = GetFeatureInterfaceVtable(),
      .streaming_io_access = GetStreamingIoRealtimeAccessVtable(),
  };
  IntrinsicIconRtclAction* action_instance = nullptr;
  std::string params_string = params.SerializeAsString();
  INTR_RETURN_IF_ERROR(ToAbslStatus(plugin_function_table.create(
      /*server_functions=*/server_functions,
      /*params_any_proto=*/WrapView(params_string),
      /*action_factory_context=*/Wrap(&context),
      /*action_ptr_out=*/&action_instance)));
  if (action_instance == nullptr) {
    return absl::InternalError(
        "Plugin Action factory returned nullptr, but did not populate status!");
  }
  return absl::WrapUnique(
      new PluginAction(plugin_function_table, action_instance));
}

PluginAction::PluginAction(IntrinsicIconRtclActionVtable plugin_function_table,
                           IntrinsicIconRtclAction* action_instance)
    : plugin_function_table_(plugin_function_table),
      action_instance_(action_instance,
                       DestroyPluginActionInstance{
                           .destroy = plugin_function_table_.destroy}) {}

RealtimeStatus PluginAction::OnEnter(OnEnterParameters params) {
  return ToRealtimeStatus(plugin_function_table_.on_enter(
      action_instance_.get(), Wrap(&params.slot_map)));
}

RealtimeStatus PluginAction::Sense(SenseParameters params) {
  return ToRealtimeStatus(plugin_function_table_.sense(
      action_instance_.get(), Wrap(&params.slot_map),
      Wrap(&params.streaming_io_access), Wrap(&params.signal_access)));
}

RealtimeStatus PluginAction::Control(ControlParameters params) {
  return ToRealtimeStatus(plugin_function_table_.control(
      action_instance_.get(), Wrap(&params.slot_map)));
}

RealtimeStatusOr<StateVariableValue> PluginAction::GetStateVariable(
    absl::string_view name) const {
  IntrinsicIconStateVariableValue state_variable;
  INTRINSIC_RT_RETURN_IF_ERROR(
      ToRealtimeStatus(plugin_function_table_.get_state_variable(
          action_instance_.get(), name.data(), name.size(), &state_variable)));
  switch (state_variable.type) {
    case IntrinsicIconStateVariableValue::kBool:
      return StateVariableValue(state_variable.value.bool_value);
    case IntrinsicIconStateVariableValue::kDouble:
      return StateVariableValue(state_variable.value.double_value);
    case IntrinsicIconStateVariableValue::kInt64:
      return StateVariableValue(state_variable.value.int64_value);
    case IntrinsicIconStateVariableValue::kNone:
      return icon::InternalError(
          RealtimeStatus::StrCat("State variable '", name, "' is not set."));
  }
  return icon::InternalError(
      RealtimeStatus::StrCat("State variable '", name, "' has invalid type."));
}

}  // namespace intrinsic::icon
