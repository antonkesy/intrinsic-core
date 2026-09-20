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

#include "intrinsic/icon/control/c_api/wrappers/joint_position_wrapper.h"

#include "intrinsic/icon/control/c_api/c_feature_interfaces.h"
#include "intrinsic/icon/control/c_api/c_realtime_status.h"
#include "intrinsic/icon/control/c_api/c_types.h"
#include "intrinsic/icon/control/c_api/convert_c_realtime_status.h"
#include "intrinsic/icon/control/c_api/convert_c_types.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {
namespace {

JointPosition* Unwrap(
    IntrinsicIconFeatureInterfaceJointPositionCommandInterface*
        joint_position) {
  return reinterpret_cast<JointPosition*>(joint_position);
}

const JointPosition* Unwrap(
    const IntrinsicIconFeatureInterfaceJointPositionCommandInterface*
        joint_position) {
  return reinterpret_cast<const JointPosition*>(joint_position);
}

IntrinsicIconRealtimeStatus SetPositionSetpoints(
    IntrinsicIconFeatureInterfaceJointPositionCommandInterface* self,
    const IntrinsicIconJointPositionCommand* const setpoints) {
  RealtimeStatusOr<JointPositionCommand> maybe_setpoints = Convert(*setpoints);
  if (!maybe_setpoints.ok()) {
    return FromRealtimeStatus(maybe_setpoints.status());
  }
  return FromRealtimeStatus(
      Unwrap(self)->SetPositionSetpoints(maybe_setpoints.value()));
}

IntrinsicIconJointPositionCommand PreviousPositionSetpoints(
    const IntrinsicIconFeatureInterfaceJointPositionCommandInterface* self) {
  return Convert(Unwrap(self)->PreviousPositionSetpoints());
}

}  // namespace

IntrinsicIconFeatureInterfaceJointPositionCommandInterface* Wrap(
    JointPosition* joint_position) {
  return reinterpret_cast<
      IntrinsicIconFeatureInterfaceJointPositionCommandInterface*>(
      joint_position);
}

const IntrinsicIconFeatureInterfaceJointPositionCommandInterface* Wrap(
    const JointPosition* joint_position) {
  return reinterpret_cast<
      const IntrinsicIconFeatureInterfaceJointPositionCommandInterface*>(
      joint_position);
}

IntrinsicIconFeatureInterfaceJointPositionCommandInterfaceVtable
GetJointPositionCommandInterfaceVtable() {
  return {.set_position_setpoints = &SetPositionSetpoints,
          .previous_position_setpoints = &PreviousPositionSetpoints};
}

}  // namespace intrinsic::icon
