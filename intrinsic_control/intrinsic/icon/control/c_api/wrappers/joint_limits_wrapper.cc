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

#include "intrinsic/icon/control/c_api/wrappers/joint_limits_wrapper.h"

#include "intrinsic/icon/control/c_api/c_feature_interfaces.h"
#include "intrinsic/icon/control/c_api/c_types.h"
#include "intrinsic/icon/control/c_api/convert_c_types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"

namespace intrinsic::icon {
namespace {

const JointLimitsInterface* Unwrap(
    const IntrinsicIconFeatureInterfaceJointLimits* joint_limits) {
  return reinterpret_cast<const JointLimitsInterface*>(joint_limits);
}

IntrinsicIconJointLimits GetApplicationLimits(
    const IntrinsicIconFeatureInterfaceJointLimits* self) {
  return Convert(Unwrap(self)->GetApplicationLimits());
}

IntrinsicIconJointLimits GetSystemLimits(
    const IntrinsicIconFeatureInterfaceJointLimits* self) {
  return Convert(Unwrap(self)->GetSystemLimits());
}

}  // namespace

IntrinsicIconFeatureInterfaceJointLimits* Wrap(
    JointLimitsInterface* joint_limits) {
  return reinterpret_cast<IntrinsicIconFeatureInterfaceJointLimits*>(
      joint_limits);
}

const IntrinsicIconFeatureInterfaceJointLimits* Wrap(
    const JointLimitsInterface* joint_limits) {
  return reinterpret_cast<const IntrinsicIconFeatureInterfaceJointLimits*>(
      joint_limits);
}

IntrinsicIconFeatureInterfaceJointLimitsVtable
GetJointLimitsFeatureInterfaceVtable() {
  return {.get_application_limits = &GetApplicationLimits,
          .get_system_limits = &GetSystemLimits};
}
}  // namespace intrinsic::icon
