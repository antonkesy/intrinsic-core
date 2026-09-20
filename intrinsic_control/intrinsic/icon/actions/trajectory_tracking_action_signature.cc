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

#include "intrinsic/icon/actions/trajectory_tracking_action_signature.h"

#include "absl/log/check.h"
#include "intrinsic/icon/actions/action_utils.h"
#include "intrinsic/icon/actions/trajectory_tracking_action_info.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic::icon {

intrinsic_proto::icon::v1::ActionSignature
GetTrajectoryTrackingActionSignature() {
  ActionSignatureBuilder b(TrajectoryTrackingActionInfo::kActionTypeName,
                           TrajectoryTrackingActionInfo::kActionDescription);
  CHECK_OK(b.AddPartSlot(TrajectoryTrackingActionInfo::kSlotName,
                         TrajectoryTrackingActionInfo::kSlotDescription,
                         /*required_feature_interfaces=*/
                         {
                             intrinsic_proto::icon::v1::FeatureInterfaceTypes::
                                 FEATURE_INTERFACE_JOINT_POSITION,
                             intrinsic_proto::icon::v1::FeatureInterfaceTypes::
                                 FEATURE_INTERFACE_JOINT_VELOCITY_ESTIMATOR,
                             intrinsic_proto::icon::v1::FeatureInterfaceTypes::
                                 FEATURE_INTERFACE_JOINT_LIMITS,
                         }));
  CHECK_OK(
      b.SetFixedParametersType<TrajectoryTrackingActionInfo::FixedParams>());
  CHECK_OK(b.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                  StateVariableInfo::TYPE_BOOL>(
      kIsDone, kIsDoneDescription));
  CHECK_OK(b.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                  StateVariableInfo::TYPE_BOOL>(
      TrajectoryTrackingActionInfo::kIsSettled,
      TrajectoryTrackingActionInfo::kIsSettledDescription));
  CHECK_OK(b.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                  StateVariableInfo::TYPE_DOUBLE>(
      TrajectoryTrackingActionInfo::kIsSettledUncertainty,
      TrajectoryTrackingActionInfo::kIsSettledUncertaintyDescription));
  CHECK_OK(b.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                  StateVariableInfo::TYPE_DOUBLE>(
      TrajectoryTrackingActionInfo::kTrajectoryProgress,
      TrajectoryTrackingActionInfo::kTrajectoryProgressDescription));
  CHECK_OK(b.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                  StateVariableInfo::TYPE_DOUBLE>(
      TrajectoryTrackingActionInfo::kTimeSinceTrajectoryStartSeconds,
      TrajectoryTrackingActionInfo::
          kTimeSinceTrajectoryStartSecondsDescription));
  CHECK_OK(b.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                  StateVariableInfo::TYPE_DOUBLE>(
      TrajectoryTrackingActionInfo::kCartesianArcLengthAlongTrajectoryMeters,
      TrajectoryTrackingActionInfo::
          kCartesianArcLengthAlongTrajectoryMetersDescription));
  CHECK_OK(b.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                  StateVariableInfo::TYPE_DOUBLE>(
      TrajectoryTrackingActionInfo::kDistanceToFinalSetpoint,
      TrajectoryTrackingActionInfo::kDistanceToFinalSetpointDescription));
  CHECK_OK(b.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                  StateVariableInfo::TYPE_DOUBLE>(
      TrajectoryTrackingActionInfo::kTrajectoryDoneForSeconds,
      TrajectoryTrackingActionInfo::kTrajectoryDoneForSecondsDescription));

  CHECK_OK(b.AddRealtimeSignal(
      TrajectoryTrackingActionInfo::kSignalPathAccurateStop,
      TrajectoryTrackingActionInfo::kSignalPathAccurateStopDescription));

  CHECK_OK(b.AddSupportedBehaviorOverride(
      intrinsic_proto::icon::v1::BehaviorOverrideRequest::
          BEHAVIOR_OVERRIDE_REQUEST_PAUSE,
      "Decelerates smoothly to a stop on the trajectory path. Resumes motion "
      "when requested."));

  return b.Finish();
}

}  // namespace intrinsic::icon
