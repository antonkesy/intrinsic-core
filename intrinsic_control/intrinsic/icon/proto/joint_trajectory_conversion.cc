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

#include "intrinsic/icon/proto/joint_trajectory_conversion.h"

#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "google/protobuf/duration.pb.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/icon/proto/joint_state_conversion.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/dynamic_limits_check_mode.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/util/proto_time.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

intrinsic_proto::icon::JointTrajectoryInterpolationType ToProto(
    const JointTrajectoryInterpolationType& param) {
  intrinsic_proto::icon::JointTrajectoryInterpolationType proto;
  switch (param) {
    case JointTrajectoryInterpolationType::kCubicPolynomial: {
      proto = intrinsic_proto::icon::JointTrajectoryInterpolationType::
          INTERPOLATION_TYPE_CUBIC_POLYNOMIAL;
      break;
    }
    case JointTrajectoryInterpolationType::kQuinticPolynomial: {
      proto = intrinsic_proto::icon::JointTrajectoryInterpolationType::
          INTERPOLATION_TYPE_QUINTIC_POLYNOMIAL;
      break;
    }
    default: {
      proto = intrinsic_proto::icon::JointTrajectoryInterpolationType::
          INTERPOLATION_TYPE_UNSPECIFIED;
      break;
    }
  }
  return proto;
}

absl::StatusOr<JointTrajectoryInterpolationType> FromProto(
    const intrinsic_proto::icon::JointTrajectoryInterpolationType& proto) {
  switch (proto) {
    case intrinsic_proto::icon::JointTrajectoryInterpolationType::
        INTERPOLATION_TYPE_UNSPECIFIED:
      return JointTrajectoryInterpolationType::kUnspecified;
    case intrinsic_proto::icon::JointTrajectoryInterpolationType::
        INTERPOLATION_TYPE_CUBIC_POLYNOMIAL:
      return JointTrajectoryInterpolationType::kCubicPolynomial;
    case intrinsic_proto::icon::JointTrajectoryInterpolationType::
        INTERPOLATION_TYPE_QUINTIC_POLYNOMIAL:
      return JointTrajectoryInterpolationType::kQuinticPolynomial;
    default:
      return absl::InvalidArgumentError("Invalid interpolation type.");
  }
}

absl::StatusOr<intrinsic_proto::icon::JointTrajectoryPVA> ToProto(
    const JointTrajectoryPVA& joint_trajectory) {
  intrinsic_proto::icon::JointTrajectoryPVA trajectory_proto;

  for (int i = 0; i < joint_trajectory.size(); i++) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(absl::Duration time_since_start,
                                  joint_trajectory.TimeAt(i));
    INTR_RETURN_IF_ERROR(FromAbslDuration(
        time_since_start, trajectory_proto.add_time_since_start()));

    INTRINSIC_RT_ASSIGN_OR_RETURN(JointStatePVA joint_state,
                                  joint_trajectory.DataAt(i));
    *trajectory_proto.add_state() = ToProto(joint_state);
    if (joint_trajectory.HasCartesianArcLength()) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(double cartesian_arc_length,
                                    joint_trajectory.CartesianArcLengthAt(i));
      trajectory_proto.add_cartesian_arc_length_meters(cartesian_arc_length);
    }
  }
  trajectory_proto.set_joint_dynamic_limits_check_mode(
      ToProto(joint_trajectory.joint_dynamic_limits_check_mode()));

  trajectory_proto.set_interpolation_type(
      ToProto(joint_trajectory.interpolation_type()));

  return trajectory_proto;
}

absl::StatusOr<JointTrajectoryPVA> FromProto(
    const intrinsic_proto::icon::JointTrajectoryPVA& proto) {
  if (proto.time_since_start_size() < 1) {
    return absl::InvalidArgumentError(
        "JointTrajectoryPVA time_since_start must not be empty.");
  }

  if (proto.state_size() != proto.time_since_start_size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("JointTrajectoryPVA sizes do not match: states is size ",
                     proto.state_size(), " but time_since_start is size ",
                     proto.time_since_start_size()));
  }

  if (proto.cartesian_arc_length_meters_size() > 0 &&
      proto.cartesian_arc_length_meters_size() !=
          proto.time_since_start_size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Cartesian arc length size does not match: its size ",
        proto.cartesian_arc_length_meters_size(),
        " but time_since_start is size ", proto.time_since_start_size()));
  }

  std::vector<absl::Duration> time_stamps;
  time_stamps.reserve(proto.time_since_start_size());
  std::vector<JointStatePVA> joint_states;
  joint_states.reserve(proto.time_since_start_size());

  for (int i = 0; i < proto.time_since_start_size(); ++i) {
    time_stamps.push_back(
        ToAbslDurationNoValidation(proto.time_since_start(i)));
    INTR_ASSIGN_OR_RETURN(auto joint_state, FromProto(proto.state(i)));
    joint_states.push_back(std::move(joint_state));
  }

  std::optional<std::vector<double>> cartesian_arc_lengths;
  if (proto.cartesian_arc_length_meters_size() > 0) {
    cartesian_arc_lengths =
        std::vector<double>(proto.cartesian_arc_length_meters().begin(),
                            proto.cartesian_arc_length_meters().end());
  }

  INTR_ASSIGN_OR_RETURN(DynamicLimitsCheckMode joint_dynamic_limits_check_mode,
                        FromProto(proto.joint_dynamic_limits_check_mode()));

  INTR_ASSIGN_OR_RETURN(JointTrajectoryInterpolationType interpolation_type,
                        FromProto(proto.interpolation_type()));

  return JointTrajectoryPVA::Create(
      std::move(joint_states), std::move(time_stamps),
      joint_dynamic_limits_check_mode, interpolation_type,
      std::move(cartesian_arc_lengths));
}

}  // namespace intrinsic
