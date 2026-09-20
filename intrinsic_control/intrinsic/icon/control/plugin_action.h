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

#ifndef INTRINSIC_ICON_CONTROL_PLUGIN_ACTION_H_
#define INTRINSIC_ICON_CONTROL_PLUGIN_ACTION_H_

#include <cstdint>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/any.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/c_api/c_plugin_api.h"
#include "intrinsic/icon/control/c_api/c_realtime_status.h"
#include "intrinsic/icon/control/c_api/c_rtcl_action.h"
#include "intrinsic/icon/control/c_api/c_types.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

IntrinsicIconRealtimeStatus RegisterActionFromPlugin(
    int64_t icon_api_version, IntrinsicIconStringView action_type_name,
    IntrinsicIconStringView action_signature_proto,
    IntrinsicIconRtclActionVtable function_table);

// Invokes `register_actions_fn`, a function pointer loaded from an Action
// plugin, in order to register the Actions contained in that plugin.
absl::Status RegisterPluginActions(
    IntrinsicIconRegisterActionTypes register_actions_fn);

class PluginAction final : public RtclActionInterface {
 public:
  // `plugin_function_table` is passed by value because it is bound to this for
  // later use via absl::bind_front. If it were a reference, that reference
  // might go out of scope before the bound version of this function is actually
  // called.
  static absl::StatusOr<std::unique_ptr<PluginAction>> Create(
      IntrinsicIconRtclActionVtable plugin_function_table,
      const google::protobuf::Any& params, ActionFactoryContext& context);

  RealtimeStatus OnEnter(OnEnterParameters params) override;

  RealtimeStatus Sense(SenseParameters params) override;

  RealtimeStatus Control(ControlParameters params) override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const override;

 private:
  struct DestroyPluginActionInstance {
    void operator()(IntrinsicIconRtclAction* self) { destroy(self); }
    void (*destroy)(IntrinsicIconRtclAction* self);
  };

  PluginAction(IntrinsicIconRtclActionVtable plugin_function_table,
               IntrinsicIconRtclAction* action_instance);

  IntrinsicIconRtclActionVtable plugin_function_table_;
  std::unique_ptr<IntrinsicIconRtclAction, DestroyPluginActionInstance>
      action_instance_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PLUGIN_ACTION_H_
