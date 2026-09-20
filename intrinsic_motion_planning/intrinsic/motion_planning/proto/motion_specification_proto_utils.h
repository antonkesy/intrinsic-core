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

#ifndef INTRINSIC_MOTION_PLANNING_PROTO_MOTION_SPECIFICATION_PROTO_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_PROTO_MOTION_SPECIFICATION_PROTO_UTILS_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_specification.pb.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {
// Returns the CartesianLimits defined in the dynamic cartesian limits proto. It
// is initialized with the `default_limits` and only overrides those defined in
// the proto.
CartesianLimits ParseCartesianLimitsFromProto(
    const intrinsic_proto::motion_planning::v1::DynamicCartesianLimits&
        cartesian_limits,
    const CartesianLimits& default_limits);

// Returns the CartesianLimits defined in the motion_segment's dynamic limits.
// It is initialized with the `default_limits` and only overrides those defined
// in the proto.
CartesianLimits ParseCartesianLimitsFromProto(
    const intrinsic_proto::motion_planning::v1::MotionSegment& motion_segment,
    const CartesianLimits& default_limits);

// Reads the joint position limits from a JointPositionLimitConstraint and
// updates `joint_limits` if set. Returns InvalidArgumentError if the robot
// is not compatible with the provided `robot` or if it does not exist in the
// world.
absl::Status SetJointPositionLimitsFromProto(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::JointPositionLimits&
        joint_position_proto,
    JointLimits& joint_limits);

// Parses the Joint Limits from a motion segment and returns the parsed joint
// limits. The limits will be initialized with the provided default values, and
// only set values will be overridden. Otherwise the default value will be used.
// If no joint limit values were set, the default limits are returned.
absl::StatusOr<JointLimits> ParseJointLimitsFromProto(
    const intrinsic_proto::motion_planning::v1::MotionSegment& motion_segment,
    const JointLimits& default_limits);

// Converts JointLimitsUpdate proto to the joint limits.
// The fields that are not set in the proto will be set to infinite.
absl::StatusOr<JointLimits> ToJointLimits(
    const intrinsic_proto::motion_planning::v1::JointLimitsUpdate&
        joint_limits_proto);

// Converts joint limits to the JointLimitsUpdate proto.
// Only sets the fields that are not infinite.
intrinsic_proto::motion_planning::v1::JointLimitsUpdate
ToJointLimitsUpdateProto(const JointLimits& joint_limits);

// Converts cartesian limits to the DynamicCartesianLimits proto. The max
// translational velocity and acceleration for the Dynamic Cartesian Limit is
// set to the max coefficient of the Cartesian Limits. Does not check if the
// content of CartesianLimits is valid, i.e., velocity and acceleration are
// positive values. Does not set individual fields if value is set infinite.
intrinsic_proto::motion_planning::v1::DynamicCartesianLimits
ToDynamicCartesianLimitsProto(const CartesianLimits& cartesian_limits);

// Converts a UniformGeometricConstraint proto to a UniformGeometricConstraint.
absl::StatusOr<std::vector<std::unique_ptr<ConstraintInterface>>>
GetUniformGeometricConstraintsFromProto(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::UniformGeometricConstraint&
        geometric_constraint_proto);

absl::StatusOr<
    std::vector<intrinsic_proto::motion_planning::v1::GeometricConstraint>>
GetGeometricConstraintFrom(
    const intrinsic_proto::motion_planning::v1::UniformGeometricConstraint&
        uniform_geometric_constraints);

absl::StatusOr<std::vector<std::unique_ptr<ConstraintInterface>>>
GetConstraintsFromProto(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        geometric_constraint);

// Converts a RelativePositionEquality constraint into an equivalent
// PositionEquality constraint.
absl::StatusOr<intrinsic_proto::motion_planning::v1::PositionEquality>
ConvertRelativePositionConstraint(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::RelativePositionEquality&
        relative_constraint);

// Converts a RelativeRotationEquality constraint into an equivalent
// RotationEquality constraint.
absl::StatusOr<intrinsic_proto::motion_planning::v1::RotationEquality>
ConvertRelativeRotationConstraint(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::RelativeRotationEquality&
        relative_constraint);

struct PositionAndRotationEquality {
  intrinsic_proto::motion_planning::v1::PositionEquality position_equality;
  intrinsic_proto::motion_planning::v1::RotationEquality rotation_equality;
};
// Converts a RelativePoseEquality constraint into an equivalent pair of
// PositionEquality and RotationEquality constraints.
absl::StatusOr<PositionAndRotationEquality> ConvertRelativePoseConstraint(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::RelativePoseEquality&
        relative_pose_constraint);

absl::StatusOr<intrinsic_proto::motion_planning::v1::PoseEquality>
ConvertRelativePoseConstraintToPoseConstraint(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::RelativePoseEquality&
        relative_pose_constraint);

// Given the `world` and `robot`, returns a representative Cartesian 3D
// position of the `geometric_constraint` in the robot base frame.
// This representative Cartesian 3D position can be used for example to
// represent the `geometric_constraint` in a graph of `GeometricConstraint`s, so
// that we can reason about its proximity relative to other
// `GeometricConstraint`s, for example for the purpose of solving Inverse
// Kinematics (IK) and use the IK solution as the IK hint joint configuration
// for solving the next IK problem on a nearby `GeometricConstraint`, much like
// a propagation of IK computations on the graph.
//
// The pose offset `flange_t_tool` needs to match those of every constraint
// involved.
// The supported `GeometricConstraint` types are:
// - every `GeometricConstraint` that is converted into
//   `PositionEqualityConstraint`, such as `PositionEquality`,
//   `RelativePositionEquality`, `PoseEquality`, `RelativePoseEquality`, etc.
// - `PositionBoundingBox` `GeometricConstraint`.
// - `ConstraintIntersection` of the above-mentioned `GeometricConstraint`s,
//   which may consist of one or multiple `PositionEqualityConstraint`s and
//   (only) one `PositionBoundingBoxConstraint`, as long as the consisting
//   `GeometricConstraint`s are consistent with each other.
//
// `PointAt` `GeometricConstraint` is not supported and will return an
// `UnimplementedError` if specified.
absl::StatusOr<eigenmath::Vector3d>
GetGeometricConstraintRepresentativeBasePPoint(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        geometric_constraint,
    const Pose3d& flange_t_tool = Pose3d::Identity());

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PROTO_MOTION_SPECIFICATION_PROTO_UTILS_H_
