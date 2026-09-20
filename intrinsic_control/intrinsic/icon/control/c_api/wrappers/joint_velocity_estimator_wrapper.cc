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

#include "intrinsic/icon/control/c_api/wrappers/joint_velocity_estimator_wrapper.h"

#include "intrinsic/icon/control/c_api/c_feature_interfaces.h"
#include "intrinsic/icon/control/c_api/c_types.h"
#include "intrinsic/icon/control/c_api/convert_c_types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"

namespace intrinsic::icon {
namespace {

const JointVelocityEstimator* Unwrap(
    const IntrinsicIconFeatureInterfaceJointVelocityEstimator*
        joint_velocity_estimator) {
  return reinterpret_cast<const JointVelocityEstimator*>(
      joint_velocity_estimator);
}

IntrinsicIconJointStateV GetVelocityEstimate(
    const IntrinsicIconFeatureInterfaceJointVelocityEstimator* self) {
  return Convert(Unwrap(self)->GetVelocityEstimate());
}

}  // namespace

IntrinsicIconFeatureInterfaceJointVelocityEstimator* Wrap(
    JointVelocityEstimator* joint_velocity_estimator) {
  return reinterpret_cast<IntrinsicIconFeatureInterfaceJointVelocityEstimator*>(
      joint_velocity_estimator);
}

const IntrinsicIconFeatureInterfaceJointVelocityEstimator* Wrap(
    const JointVelocityEstimator* joint_velocity_estimator) {
  return reinterpret_cast<
      const IntrinsicIconFeatureInterfaceJointVelocityEstimator*>(
      joint_velocity_estimator);
}

IntrinsicIconFeatureInterfaceJointVelocityEstimatorVtable
GetJointVelocityEstimatorVtable() {
  return {.get_velocity_estimate = &GetVelocityEstimate};
}

}  // namespace intrinsic::icon
