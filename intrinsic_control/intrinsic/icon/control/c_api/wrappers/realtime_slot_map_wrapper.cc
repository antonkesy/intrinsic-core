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

#include "intrinsic/icon/control/c_api/wrappers/realtime_slot_map_wrapper.h"

#include <cstdint>

#include "intrinsic/icon/control/c_api/c_feature_interfaces.h"
#include "intrinsic/icon/control/c_api/c_realtime_slot_map.h"
#include "intrinsic/icon/control/c_api/wrappers/force_torque_sensor_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/joint_limits_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/joint_position_sensor_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/joint_position_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/joint_velocity_estimator_wrapper.h"
#include "intrinsic/icon/control/c_api/wrappers/manipulator_kinematics_wrapper.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
#include "intrinsic/icon/control/slot_types.h"

namespace intrinsic::icon {
namespace {

IntrinsicIconFeatureInterfacesForSlot GetMutableFeatureInterfacesForSlot(
    IntrinsicIconRealtimeSlotMap* self, uint64_t slot_id) {
  RealtimeSlotMap* realtime_slot_map = reinterpret_cast<RealtimeSlotMap*>(self);
  RealtimeSlotId icon_slot_id(slot_id);
  return IntrinsicIconFeatureInterfacesForSlot{
      .joint_position =
          Wrap(realtime_slot_map->GetMutableInterfaceForSlot<JointPosition>(
              icon_slot_id)),
      .joint_position_sensor = Wrap(
          realtime_slot_map->GetMutableInterfaceForSlot<JointPositionSensor>(
              icon_slot_id)),
      .joint_velocity_estimator = Wrap(
          realtime_slot_map->GetMutableInterfaceForSlot<JointVelocityEstimator>(
              icon_slot_id)),
      .joint_limits = Wrap(
          realtime_slot_map->GetMutableInterfaceForSlot<JointLimitsInterface>(
              icon_slot_id)),
      .manipulator_kinematics = Wrap(
          realtime_slot_map->GetMutableInterfaceForSlot<ManipulatorKinematics>(
              icon_slot_id)),
      .force_torque_sensor =
          Wrap(realtime_slot_map->GetMutableInterfaceForSlot<ForceTorqueSensor>(
              icon_slot_id)),
  };
}

IntrinsicIconConstFeatureInterfacesForSlot GetFeatureInterfacesForSlot(
    const IntrinsicIconRealtimeSlotMap* self, uint64_t slot_id) {
  const RealtimeSlotMap* realtime_slot_map =
      reinterpret_cast<const RealtimeSlotMap*>(self);
  RealtimeSlotId icon_slot_id(slot_id);
  return IntrinsicIconConstFeatureInterfacesForSlot{
      .joint_position = Wrap(
          realtime_slot_map->GetInterfaceForSlot<JointPosition>(icon_slot_id)),
      .joint_position_sensor =
          Wrap(realtime_slot_map->GetInterfaceForSlot<JointPositionSensor>(
              icon_slot_id)),
      .joint_velocity_estimator =
          Wrap(realtime_slot_map->GetInterfaceForSlot<JointVelocityEstimator>(
              icon_slot_id)),
      .joint_limits =
          Wrap(realtime_slot_map->GetInterfaceForSlot<JointLimitsInterface>(
              icon_slot_id)),
      .manipulator_kinematics =
          Wrap(realtime_slot_map->GetInterfaceForSlot<ManipulatorKinematics>(
              icon_slot_id)),
      .force_torque_sensor =
          Wrap(realtime_slot_map->GetInterfaceForSlot<ForceTorqueSensor>(
              icon_slot_id)),
  };
}

}  // namespace

IntrinsicIconRealtimeSlotMap* Wrap(RealtimeSlotMap* realtime_slot_map) {
  return reinterpret_cast<IntrinsicIconRealtimeSlotMap*>(realtime_slot_map);
}

const IntrinsicIconRealtimeSlotMap* Wrap(
    const RealtimeSlotMap* realtime_slot_map) {
  return reinterpret_cast<const IntrinsicIconRealtimeSlotMap*>(
      realtime_slot_map);
}

IntrinsicIconRealtimeSlotMapVtable GetRealtimeSlotMapVtable() {
  return {
      .get_mutable_feature_interfaces_for_slot =
          &GetMutableFeatureInterfacesForSlot,
      .get_feature_interfaces_for_slot = &GetFeatureInterfacesForSlot,
  };
}

}  // namespace intrinsic::icon
