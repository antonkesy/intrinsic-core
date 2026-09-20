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

#include "intrinsic/icon/control/c_api/wrappers/joint_position_sensor_wrapper.h"

#include "intrinsic/icon/control/c_api/c_feature_interfaces.h"
#include "intrinsic/icon/control/c_api/c_types.h"
#include "intrinsic/icon/control/c_api/convert_c_types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"

namespace intrinsic::icon {
namespace {

const JointPositionSensor* Unwrap(
    const IntrinsicIconFeatureInterfaceJointPositionSensor*
        joint_position_sensor) {
  return reinterpret_cast<const JointPositionSensor*>(joint_position_sensor);
}

IntrinsicIconJointStateP GetSensedPosition(
    const IntrinsicIconFeatureInterfaceJointPositionSensor* self) {
  return Convert(Unwrap(self)->GetSensedPosition());
}

}  // namespace

IntrinsicIconFeatureInterfaceJointPositionSensor* Wrap(
    JointPositionSensor* joint_position_sensor) {
  return reinterpret_cast<IntrinsicIconFeatureInterfaceJointPositionSensor*>(
      joint_position_sensor);
}

const IntrinsicIconFeatureInterfaceJointPositionSensor* Wrap(
    const JointPositionSensor* joint_position_sensor) {
  return reinterpret_cast<
      const IntrinsicIconFeatureInterfaceJointPositionSensor*>(
      joint_position_sensor);
}

IntrinsicIconFeatureInterfaceJointPositionSensorVtable
GetJointPositionSensorVtable() {
  return {.get_sensed_position = &GetSensedPosition};
}

}  // namespace intrinsic::icon
