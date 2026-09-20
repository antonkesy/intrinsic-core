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

#include "intrinsic/icon/control/c_api/wrappers/feature_interface_wrappers.h"

#include "intrinsic/icon/control/c_api/c_feature_interfaces.h"
#include "intrinsic/icon/control/c_api/wrappers/force_torque_sensor_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/joint_limits_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/joint_position_sensor_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/joint_position_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/joint_velocity_estimator_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/manipulator_kinematics_wrapper.h"

namespace intrinsic::icon {

IntrinsicIconFeatureInterfaceVtable GetFeatureInterfaceVtable() {
  return {
      .joint_position = GetJointPositionCommandInterfaceVtable(),
      .joint_position_sensor = GetJointPositionSensorVtable(),
      .joint_velocity_estimator = GetJointVelocityEstimatorVtable(),
      .joint_limits = GetJointLimitsFeatureInterfaceVtable(),
      .manipulator_kinematics = GetManipulatorKinematicsVtable(),
      .force_torque_sensor = GetForceTorqueSensorVtable(),
  };
}

}  // namespace intrinsic::icon
