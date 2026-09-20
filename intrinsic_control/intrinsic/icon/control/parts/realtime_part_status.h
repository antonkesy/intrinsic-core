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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_STATUS_H_
#define INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_STATUS_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/io_block.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/proto/io_block.pb.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

// LINT.IfChange

// Exposes some of the state of a RealtimePart.
// Types and names should be consistent with those in part_interfaces.h.
//
// Copying this is realtime safe.
struct RealtimePartStatus {
  absl::Duration time_since_timeslicer_epoch;
  RealtimeOperationalStatus operational_status;
  std::optional<JointStateP> sensed_position;
  std::optional<JointStateV> sensed_velocity;
  std::optional<JointStateA> sensed_acceleration;
  std::optional<JointStateT> sensed_torque;
  std::optional<JointStateP> position_commanded_last_cycle;
  std::optional<JointStateV> velocity_commanded_last_cycle;
  std::optional<JointStateA> acceleration_commanded_last_cycle;
  std::optional<JointStateT> torque_commanded_last_cycle;
  std::optional<Pose3d> base_t_tip_sensed;
  std::optional<SimpleGripper::GripperState> gripper_state;
  std::optional<double> linear_gripper_width_sensed;
  std::optional<Wrench> wrench_at_ft_uncompensated;
  std::optional<Wrench> wrench_at_ft;
  std::optional<Wrench> wrench_at_tip;
  std::optional<double> wrench_stability_index;
  std::optional<Twist> base_twist_tip_sensed;
  std::optional<ADIO::ADIOState> adio_state;
  std::optional<ControlModeExporter::ControlMode> current_control_mode;
  std::optional<double> rangefinder_distance;
  std::optional<eigenmath::Vector3d> imu_sensed_linear_acceleration;
  std::optional<eigenmath::Vector3d> imu_sensed_angular_velocity;
  std::optional<eigenmath::Quaterniond> imu_sensed_orientation;
  std::optional<Pose3d> sensed_pose;
};
// LINT.ThenChange(
// //intrinsic_control/intrinsic/icon/cc_client/state_variable_path.h,
// //intrinsic_control/intrinsic/icon/python/state_variable_path.py)

// Attempts to convert `proto` into a RealtimePartStatus. This will only succeed
// if `proto` contains information that can be encoded in RealtimePartStatus.
//
// If the optional `*block_names` parameters are set, the ADIO part will also be
// extracted. The `*block_names` vectors must outlive the returned
// RealtimePartStatus!
//
// Not realtime safe.
absl::StatusOr<RealtimePartStatus> FromProto(
    const intrinsic_proto::icon::PartStatus& proto,
    const std::vector<std::string>* analog_input_block_names
        ABSL_ATTRIBUTE_LIFETIME_BOUND = nullptr,
    const std::vector<std::string>* analog_output_block_names
        ABSL_ATTRIBUTE_LIFETIME_BOUND = nullptr,
    const std::vector<std::string>* digital_input_block_names
        ABSL_ATTRIBUTE_LIFETIME_BOUND = nullptr,
    const std::vector<std::string>* digital_output_block_names
        ABSL_ATTRIBUTE_LIFETIME_BOUND = nullptr);

// Converts `realtime_status` into a proto. Unlike FromProto(), this cannot
// fail.
//
// Not realtime safe.
intrinsic_proto::icon::PartStatus ToProto(
    const RealtimePartStatus& realtime_status);

// Converts `gripper_state` to a proto gripper state.
//
// Realtime safe.
::intrinsic_proto::icon::GripperState_SensedState ToProto(
    SimpleGripper::GripperState gripper_state);

// Converts `mode` to a proto control mode.
//
// Realtime safe.
::intrinsic_proto::icon::PartControlMode ToProto(
    ControlModeExporter::ControlMode mode);

// Not realtime safe.
absl::StatusOr<AnalogBlock> FromProto(
    const intrinsic_proto::icon::AnalogBlock& proto_block);

// Not realtime safe.
absl::StatusOr<DioBlock> FromProto(
    const intrinsic_proto::icon::DioBlock& proto_block);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_STATUS_H_
