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

#include "intrinsic/icon/control/c_api/wrappers/force_torque_sensor_wrapper.h"

#include "intrinsic/icon/control/c_api/c_feature_interfaces.h"
#include "intrinsic/icon/control/c_api/c_realtime_status.h"
#include "intrinsic/icon/control/c_api/c_types.h"
#include "intrinsic/icon/control/c_api/convert_c_realtime_status.h"
#include "intrinsic/icon/control/c_api/convert_c_types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"

namespace intrinsic::icon {
namespace {

IntrinsicIconWrench WrenchAtTip(
    const IntrinsicIconFeatureInterfaceForceTorqueSensor* self) {
  return Convert(
      reinterpret_cast<const ForceTorqueSensor*>(self)->WrenchAtTip());
}

// TODO(b/285873767): implement "num_taring_samples" parameter here.
IntrinsicIconRealtimeStatus Tare(
    IntrinsicIconFeatureInterfaceForceTorqueSensor* self) {
  return FromRealtimeStatus(reinterpret_cast<ForceTorqueSensor*>(self)->Tare(
      /*num_taring_cycles=*/1));
}

}  // namespace

IntrinsicIconFeatureInterfaceForceTorqueSensor* Wrap(
    ForceTorqueSensor* force_torque_sensor) {
  return reinterpret_cast<IntrinsicIconFeatureInterfaceForceTorqueSensor*>(
      force_torque_sensor);
}

const IntrinsicIconFeatureInterfaceForceTorqueSensor* Wrap(
    const ForceTorqueSensor* force_torque_sensor) {
  return reinterpret_cast<
      const IntrinsicIconFeatureInterfaceForceTorqueSensor*>(
      force_torque_sensor);
}

IntrinsicIconFeatureInterfaceForceTorqueSensorVtable
GetForceTorqueSensorVtable() {
  return {
      .wrench_at_tip = WrenchAtTip,
      .tare = Tare,
  };
}

}  // namespace intrinsic::icon
