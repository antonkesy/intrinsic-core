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

#ifndef INTRINSIC_ICON_PROTO_JOINT_TRAJECTORY_CONVERSION_H_
#define INTRINSIC_ICON_PROTO_JOINT_TRAJECTORY_CONVERSION_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"

namespace intrinsic {

intrinsic_proto::icon::JointTrajectoryInterpolationType ToProto(
    const JointTrajectoryInterpolationType& param);

absl::StatusOr<JointTrajectoryInterpolationType> FromProto(
    const intrinsic_proto::icon::JointTrajectoryInterpolationType& proto);

absl::StatusOr<intrinsic_proto::icon::JointTrajectoryPVA> ToProto(
    const JointTrajectoryPVA& joint_trajectory);

absl::StatusOr<JointTrajectoryPVA> FromProto(
    const intrinsic_proto::icon::JointTrajectoryPVA& proto);

}  // namespace intrinsic

#endif  // INTRINSIC_ICON_PROTO_JOINT_TRAJECTORY_CONVERSION_H_
