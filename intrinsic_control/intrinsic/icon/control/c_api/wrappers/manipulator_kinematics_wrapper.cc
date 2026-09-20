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

#include "intrinsic/icon/control/c_api/wrappers/manipulator_kinematics_wrapper.h"

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/c_api/c_feature_interfaces.h"
#include "intrinsic/icon/control/c_api/c_realtime_status.h"
#include "intrinsic/icon/control/c_api/c_types.h"
#include "intrinsic/icon/control/c_api/convert_c_realtime_status.h"
#include "intrinsic/icon/control/c_api/convert_c_types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {
namespace {
const ManipulatorKinematics* Unwrap(
    const IntrinsicIconFeatureInterfaceManipulatorKinematics*
        manipulator_kinematics) {
  return reinterpret_cast<const ManipulatorKinematics*>(manipulator_kinematics);
}

IntrinsicIconRealtimeStatus ComputeChainJacobian(
    const IntrinsicIconFeatureInterfaceManipulatorKinematics* self,
    const IntrinsicIconJointStateP* dof_positions,
    IntrinsicIconMatrix6Nd* jacobian_out) {
  const ManipulatorKinematics* manipulator_kinematics = Unwrap(self);
  intrinsic::icon::RealtimeStatusOr<eigenmath::Matrix6Nd> jacobian =
      manipulator_kinematics->ComputeChainJacobian(Convert(*dof_positions));
  if (!jacobian.ok()) {
    return FromRealtimeStatus(jacobian.status());
  }
  *jacobian_out = Convert(jacobian.value());

  return FromRealtimeStatus(OkStatus());
}

IntrinsicIconRealtimeStatus ComputeChainFK(
    const IntrinsicIconFeatureInterfaceManipulatorKinematics* self,
    const IntrinsicIconJointStateP* dof_positions,
    IntrinsicIconPose3d* pose_out) {
  const ManipulatorKinematics* manipulator_kinematics = Unwrap(self);
  intrinsic::icon::RealtimeStatusOr<Pose3d> fk_pose =
      manipulator_kinematics->ComputeChainFK(Convert(*dof_positions));
  if (!fk_pose.ok()) {
    return FromRealtimeStatus(fk_pose.status());
  }
  *pose_out = Convert(fk_pose.value());

  return FromRealtimeStatus(OkStatus());
}
}  // namespace

IntrinsicIconFeatureInterfaceManipulatorKinematics* Wrap(
    ManipulatorKinematics* manipulator_kinematics) {
  return reinterpret_cast<IntrinsicIconFeatureInterfaceManipulatorKinematics*>(
      manipulator_kinematics);
}

const IntrinsicIconFeatureInterfaceManipulatorKinematics* Wrap(
    const ManipulatorKinematics* manipulator_kinematics) {
  return reinterpret_cast<
      const IntrinsicIconFeatureInterfaceManipulatorKinematics*>(
      manipulator_kinematics);
}

IntrinsicIconFeatureInterfaceManipulatorKinematicsVtable
GetManipulatorKinematicsVtable() {
  return {
      .compute_chain_jacobian = ComputeChainJacobian,
      .compute_chain_fk = ComputeChainFK,
  };
}

}  // namespace intrinsic::icon
