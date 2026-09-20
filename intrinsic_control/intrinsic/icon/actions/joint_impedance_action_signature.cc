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

#include "intrinsic/icon/actions/joint_impedance_action_signature.h"

#include "absl/log/check.h"
#include "intrinsic/icon/actions/action_utils.h"
#include "intrinsic/icon/actions/joint_impedance_action_info.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic::icon {

::intrinsic_proto::icon::v1::ActionSignature GetJointImpedanceSignature() {
  ActionSignatureBuilder builder(JointImpedanceInfo::kActionTypeName,
                                 JointImpedanceInfo::kActionDescription);
  CHECK_OK(builder.AddPartSlot(
      JointImpedanceInfo::kSlotName, JointImpedanceInfo::kSlotDescription,
      /*required_feature_interfaces=*/
      {
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_JOINT_TORQUE,
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_JOINT_POSITION_SENSOR,
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_JOINT_VELOCITY_ESTIMATOR,
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_JOINT_LIMITS,
      },
      /*optional_feature_interfaces=*/
      {
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_JOINT_ACCELERATION_ESTIMATOR,
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_DYNAMICS,
      }));
  CHECK_OK(builder.SetFixedParametersType<JointImpedanceInfo::FixedParams>());

  CHECK_OK(builder.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                        StateVariableInfo::TYPE_BOOL>(
      kIsDone, kIsDoneDescription));

  CHECK_OK(builder.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                        StateVariableInfo::TYPE_BOOL>(
      JointImpedanceInfo::kIsSettled,
      JointImpedanceInfo::kIsSettledDescription));

  CHECK_OK(builder.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                        StateVariableInfo::TYPE_DOUBLE>(
      JointImpedanceInfo::kDistanceToSensed,
      JointImpedanceInfo::kDistanceToSensedDescription));

  CHECK_OK(builder.AddStreamingInput<JointImpedanceInfo::FixedParams>(
      JointImpedanceInfo::kStreamingInputName));

  return builder.Finish();
}

}  // namespace intrinsic::icon
