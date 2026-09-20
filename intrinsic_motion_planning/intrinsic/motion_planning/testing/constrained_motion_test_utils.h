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

#ifndef INTRINSIC_MOTION_PLANNING_TESTING_CONSTRAINED_MOTION_TEST_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_TESTING_CONSTRAINED_MOTION_TEST_UTILS_H_

#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/proto/motion_target.pb.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/skills/proto/motion_targets.pb.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"

namespace intrinsic {

using ::intrinsic_proto::world::TransformNodeReference;

// Creates a geometric constraint of type PositionEquality that requires the
// origin of the 'tool_name' frame to have the same position as
// `world_p_moving`, regardless of the orientation of the frames.
intrinsic_proto::motion_planning::v1::PositionEquality
CreatePositionEqualityGeometricConstraint(
    const WorldObjectName& tool_object_name, const FrameName& tool_name,
    const eigenmath::Vector3d& world_p_moving);

// Creates a geometric constraint of type PositionEquality that requires the
// origin of the 'tool_name' frame of `tool_object_name` to have the same
// position as `world_t_reference * reference_p_moving`.
intrinsic_proto::motion_planning::v1::PositionEquality
CreatePositionEqualityGeometricConstraint(
    const WorldObjectName& reference_object_name,
    const FrameName& reference_name, const WorldObjectName& tool_object_name,
    const FrameName& tool_name, const eigenmath::Vector3d& reference_p_moving);

// Creates a geometric constraint of type `PointAt` with the `tool_name` frame
// of `tool_object_name` as the `moving_frame` and the `reference_name` frame of
// `reference_object_name` as the `target_frame`. All other fields are set to
// default values (please refer to the proto definition for more details).
intrinsic_proto::motion_planning::v1::PointAt CreatePointAtGeometricConstraint(
    const WorldObjectName& reference_object_name,
    const FrameName& reference_name, const WorldObjectName& tool_object_name,
    const FrameName& tool_name);

// Creates a geometric constraint of type RotationCone that requires the z-axis
// in the `target` frame to be rotated within a maximum angle of
// `cone_half_angle` from the equivalent axis in the world frame. Does not
// constrain the positions of the frames.
intrinsic_proto::motion_planning::v1::RotationCone
CreateRotationConeGeometricConstraint(absl::string_view tool_object_name,
                                      absl::string_view tool_name,
                                      double cone_half_angle,
                                      const Pose3d& world_t_target);

// Creates a geometric constraint of type RotationCone that requires the z-axis
// in the `target` frame to be rotated within a maximum angle of
// `cone_half_angle` from the equivalent axis in the world frame. Does not
// constrain the positions of the frames.
intrinsic_proto::motion_planning::v1::RotationCone
CreateRotationConeGeometricConstraint(const WorldObjectName& tool_object_name,
                                      const FrameName& tool_name,
                                      double cone_half_angle,
                                      const Pose3d& world_t_target);

// Creates a geometric constraint of type RotationEquality that requires the
// `tool_name` frame of `tool_object_name` to have the same fixed relative
// rotation as the `world_t_target`.
intrinsic_proto::motion_planning::v1::RotationEquality
CreateRotationEqualityGeometricConstraint(
    const WorldObjectName& tool_object_name, const FrameName& tool_name,
    const Pose3d& world_t_target);

// Creates a geometric constraint of type RotationEquality that requires the
// `tool_name` frame of `tool_object_name` to have the same fixed relative
// rotation as the `world_t_reference * reference_t_moving`.
intrinsic_proto::motion_planning::v1::RotationEquality
CreateRotationEqualityGeometricConstraint(
    const WorldObjectName& reference_object_name,
    const FrameName& reference_name, const WorldObjectName& tool_object_name,
    const FrameName& tool_name, const Pose3d& reference_t_moving);

// Creates a geometric constraint of type PoseEquality that requires the
// `tool_name` frame of `tool_object_name` to have the same pose as
// `world_t_target`.
intrinsic_proto::motion_planning::v1::PoseEquality
CreatePoseEqualityGeometricConstraint(const WorldObjectName& tool_object_name,
                                      const FrameName& tool_name,
                                      const Pose3d& world_t_target);

// Creates a geometric constraint of type PoseEquality that requires the
// `tool_name` frame of `tool_object_name` to have the same pose as
// `world_t_reference * reference_t_moving`.
intrinsic_proto::motion_planning::v1::PoseEquality
CreatePoseEqualityGeometricConstraint(
    const WorldObjectName& reference_object_name,
    const FrameName& reference_name, const WorldObjectName& tool_object_name,
    const FrameName& tool_name, const Pose3d& reference_t_moving);

// Creates a geometric constraint of type JointPositionLimits that defines the
// lower and upper limits for the robot defined by `robot_name`.
intrinsic_proto::motion_planning::v1::JointPositionLimits
CreateJointLimitGeometricConstraint(absl::string_view robot_name,
                                    const eigenmath::VectorNd& min_position,
                                    const eigenmath::VectorNd& max_position);

// Creates a geometric constraint of type JointPositionEquality that defines the
// joint configuration for the robot defined by robot_name.
intrinsic_proto::motion_planning::v1::JointPositionEquality
CreateJointPositionEqualityGeometricConstraint(
    absl::string_view robot_name, const eigenmath::VectorNd& config);

// Creates a geometric constraint of type RelativePositionEquality that the
// `tool_name` frame of `tool_object_name` to move by an offset
// `offset_t_reference`, where reference is a reference frame for the relative
// motion.
intrinsic_proto::motion_planning::v1::RelativePoseEquality
CreateRelativePoseEqualityGeometricConstraint(
    const WorldObjectName& tool_object_name, const FrameName& tool_name,
    const WorldObjectName& reference_object_name, const Pose3d& relative_pose);

intrinsic_proto::motion_planning::v1::RelativePositionEquality
CreateRelativePositionEqualityGeometricConstraint(
    const WorldObjectName& tool_object_name, const FrameName& tool_name,
    const WorldObjectName& reference_object_name,
    const eigenmath::Vector3d& relative_position,
    const eigenmath::Vector3d& moving_frame_offset);

intrinsic_proto::motion_planning::v1::RelativeRotationEquality
CreateRelativeRotationEqualityGeometricConstraint(
    const WorldObjectName& tool_object_name, const FrameName& tool_name,
    const WorldObjectName& reference_object_name,
    const eigenmath::Quaterniond& relative_rotation);

intrinsic_proto::Point ToPoint(const std::vector<double>& v);

// Set frame to root, flange or gripper.
void SetFrame(TransformNodeReference* ref, const std::string& frame_label);

// Creates a geometric constraint of type PositionBoundingBox that defines the
// bounding box lower and upper bounds.
intrinsic_proto::motion_planning::v1::PositionBoundingBox
CreatePositionBoundingBoxProto(const std::vector<double>& lower_bounds,
                               const std::vector<double>& upper_bounds,
                               const std::string& target_frame,
                               const std::string& moving_frame);

// Creates a geometric constraint of type PositionBoundingBox that defines the
// bounding box lower and upper bounds.
intrinsic_proto::motion_planning::v1::PositionBoundingBox
CreatePositionBoundingBoxProto(const eigenmath::Vector3d& lower_bounds,
                               const eigenmath::Vector3d& upper_bounds,
                               const std::string& target_frame,
                               const std::string& moving_frame,
                               const Pose3d& target_t_bbox);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_TESTING_CONSTRAINED_MOTION_TEST_UTILS_H_
