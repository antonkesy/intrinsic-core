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

#include "intrinsic/motion_planning/testing/constrained_motion_test_utils.h"

#include <string>
#include <vector>

#include "absl/log/check.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/motion_planning/proto/motion_target.pb.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/skills/proto/motion_targets.pb.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/proto/entity_search.pb.h"

namespace intrinsic {

intrinsic_proto::motion_planning::v1::RotationCone
CreateRotationConeGeometricConstraint(absl::string_view tool_object_name,
                                      absl::string_view tool_name,
                                      double cone_half_angle,
                                      const Pose3d& world_t_target) {
  intrinsic_proto::motion_planning::v1::RotationCone cone_proto;

  // Add the world as reference frame
  cone_proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_object()
      ->set_object_name(RootObjectName().value());

  // Add the robot and tool label, to make clear which tool we want to address.
  cone_proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name);
  cone_proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name);

  eigenmath::Vector3d cone_axis({0, 0, 1});
  *cone_proto.mutable_moving_axis() = ToVectorProto(cone_axis);
  cone_proto.set_cone_opening_half_angle(cone_half_angle);

  eigenmath::Vector3d base_cone_axis =
      world_t_target.rotationMatrix() * cone_axis;
  *cone_proto.mutable_target_axis() = ToVectorProto(base_cone_axis);

  return cone_proto;
}

intrinsic_proto::motion_planning::v1::RotationCone
CreateRotationConeGeometricConstraint(const WorldObjectName& tool_object_name,
                                      const FrameName& tool_name,
                                      double cone_half_angle,
                                      const Pose3d& world_t_target) {
  return CreateRotationConeGeometricConstraint(tool_object_name.value(),
                                               tool_name.value(),
                                               cone_half_angle, world_t_target);
}

intrinsic_proto::motion_planning::v1::PositionEquality
CreatePositionEqualityGeometricConstraint(
    const WorldObjectName& tool_object_name, const FrameName& tool_name,
    const eigenmath::Vector3d& world_p_moving) {
  intrinsic_proto::motion_planning::v1::PositionEquality proto;

  // Add the world as reference frame
  proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_object()
      ->set_object_name(RootObjectName().value());

  // Add the robot and tool label, to make clear which tool we want to address.
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name.value());
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name.value());

  *proto.mutable_target_frame_offset() = ToProto(world_p_moving);

  return proto;
}

intrinsic_proto::motion_planning::v1::PointAt CreatePointAtGeometricConstraint(
    const WorldObjectName& reference_object_name,
    const FrameName& reference_name, const WorldObjectName& tool_object_name,
    const FrameName& tool_name) {
  intrinsic_proto::motion_planning::v1::PointAt point_at_proto;

  // Add the object and frame name of the reference, to make clear which frame
  // we address.
  point_at_proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(reference_object_name.value());
  point_at_proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(reference_name.value());

  // Add the object and tool label, to make clear which tool we want to address.
  point_at_proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name.value());
  point_at_proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name.value());
  return point_at_proto;
}

intrinsic_proto::motion_planning::v1::RotationEquality
CreateRotationEqualityGeometricConstraint(
    const WorldObjectName& tool_object_name, const FrameName& tool_name,
    const Pose3d& world_t_target) {
  intrinsic_proto::motion_planning::v1::RotationEquality proto;

  // Add the world as reference frame
  proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_object()
      ->set_object_name(RootObjectName().value());

  // Add the object and tool label, to make clear which tool we want to address.
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name.value());
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name.value());

  // Set orientation offset
  *proto.mutable_rotation_offset() = ToProto(world_t_target.quaternion());

  return proto;
}

intrinsic_proto::motion_planning::v1::PoseEquality
CreatePoseEqualityGeometricConstraint(const WorldObjectName& tool_object_name,
                                      const FrameName& tool_name,
                                      const Pose3d& world_t_target) {
  intrinsic_proto::motion_planning::v1::PoseEquality proto;

  // Add the world as reference frame
  proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_object()
      ->set_object_name(RootObjectName().value());

  // Add the object and tool label, to make clear which tool we want to address.
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name.value());
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name.value());

  // Set pose offset
  *proto.mutable_target_frame_offset() = ToProto(world_t_target);

  return proto;
}

intrinsic_proto::motion_planning::v1::PositionEquality
CreatePositionEqualityGeometricConstraint(
    const WorldObjectName& reference_object_name,
    const FrameName& reference_name, const WorldObjectName& tool_object_name,
    const FrameName& tool_name, const eigenmath::Vector3d& reference_p_moving) {
  intrinsic_proto::motion_planning::v1::PositionEquality proto;

  // Add the object and frame name of the reference, to make clear which frame
  // we address.
  proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(reference_object_name.value());
  proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(reference_name.value());

  // Add the object and tool label, to make clear which tool we want to address.
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name.value());
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name.value());

  // Set position offset
  *proto.mutable_target_frame_offset() = ToProto(reference_p_moving);

  return proto;
}

intrinsic_proto::motion_planning::v1::RotationEquality
CreateRotationEqualityGeometricConstraint(
    const WorldObjectName& reference_object_name,
    const FrameName& reference_name, const WorldObjectName& tool_object_name,
    const FrameName& tool_name, const Pose3d& reference_t_moving) {
  intrinsic_proto::motion_planning::v1::RotationEquality proto;

  // Add the object and frame name of the reference, to make clear which frame
  // we address.
  proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(reference_object_name.value());
  proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(reference_name.value());

  // Add the object and tool label, to make clear which tool we want to address.
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name.value());
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name.value());

  // Set orientation offset
  *proto.mutable_rotation_offset() = ToProto(reference_t_moving.quaternion());

  return proto;
}

intrinsic_proto::motion_planning::v1::PoseEquality
CreatePoseEqualityGeometricConstraint(
    const WorldObjectName& reference_object_name,
    const FrameName& reference_name, const WorldObjectName& tool_object_name,
    const FrameName& tool_name, const Pose3d& reference_t_moving) {
  intrinsic_proto::motion_planning::v1::PoseEquality proto;

  // Add the object and frame name of the reference, to make clear which frame
  // we address.
  proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(reference_object_name.value());
  proto.mutable_target_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(reference_name.value());

  // Add the object and tool label, to make clear which tool we want to address.
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name.value());
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name.value());

  // Set pose offset
  *proto.mutable_target_frame_offset() = ToProto(reference_t_moving);

  return proto;
}

intrinsic_proto::motion_planning::v1::RelativePoseEquality
CreateRelativePoseEqualityGeometricConstraint(
    const WorldObjectName& tool_object_name, const FrameName& tool_name,
    const WorldObjectName& reference_object_name, const Pose3d& relative_pose) {
  intrinsic_proto::motion_planning::v1::RelativePoseEquality proto;

  // Add the reference object.
  if (!reference_object_name.empty()) {
    proto.mutable_reference_frame()
        ->mutable_by_name()
        ->mutable_object()
        ->set_object_name(reference_object_name.value());
  }

  // Add the object and tool label, to make clear which tool we want to address.
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name.value());
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name.value());

  // Set pose offset
  *proto.mutable_relative_pose() = ToProto(relative_pose);

  return proto;
}

intrinsic_proto::motion_planning::v1::RelativePositionEquality
CreateRelativePositionEqualityGeometricConstraint(
    const WorldObjectName& tool_object_name, const FrameName& tool_name,
    const WorldObjectName& reference_object_name,
    const eigenmath::Vector3d& relative_position,
    const eigenmath::Vector3d& moving_frame_offset) {
  intrinsic_proto::motion_planning::v1::RelativePositionEquality proto;

  // Add the reference object.
  if (!reference_object_name.empty()) {
    proto.mutable_reference_frame()
        ->mutable_by_name()
        ->mutable_object()
        ->set_object_name(reference_object_name.value());
  }

  // Add the object and tool label, to make clear which tool we want to address.
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name.value());
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name.value());

  // Set pose offset
  *proto.mutable_relative_position() = ToProto(relative_position);

  *proto.mutable_moving_frame_offset() = ToProto(moving_frame_offset);

  return proto;
}

intrinsic_proto::motion_planning::v1::RelativeRotationEquality
CreateRelativeRotationEqualityGeometricConstraint(
    const WorldObjectName& tool_object_name, const FrameName& tool_name,
    const WorldObjectName& reference_object_name,
    const eigenmath::Quaterniond& relative_rotation) {
  intrinsic_proto::motion_planning::v1::RelativeRotationEquality proto;

  // Add the reference object.
  if (!reference_object_name.empty()) {
    proto.mutable_reference_frame()
        ->mutable_by_name()
        ->mutable_object()
        ->set_object_name(reference_object_name.value());
  }

  // Add the object and tool label, to make clear which tool we want to address.
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(tool_object_name.value());
  proto.mutable_moving_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(tool_name.value());

  // Set pose offset
  *proto.mutable_relative_rotation() = ToProto(relative_rotation);

  return proto;
}

intrinsic_proto::motion_planning::v1::JointPositionLimits
CreateJointLimitGeometricConstraint(absl::string_view robot_name,
                                    const eigenmath::VectorNd& min_position,
                                    const eigenmath::VectorNd& max_position) {
  intrinsic_proto::motion_planning::v1::JointPositionLimits
      joint_position_proto;

  VectorNdToRepeatedDouble(min_position,
                           joint_position_proto.mutable_lower_limits());
  VectorNdToRepeatedDouble(max_position,
                           joint_position_proto.mutable_upper_limits());
  joint_position_proto.mutable_object_id()->mutable_by_name()->set_object_name(
      robot_name);
  return joint_position_proto;
}

intrinsic_proto::motion_planning::v1::JointPositionEquality
CreateJointPositionEqualityGeometricConstraint(
    absl::string_view robot_name, const eigenmath::VectorNd& config) {
  intrinsic_proto::motion_planning::v1::JointPositionEquality
      joint_position_equality;
  joint_position_equality.mutable_object_id()
      ->mutable_by_name()
      ->set_object_name(robot_name);
  VectorNdToRepeatedDouble(
      config,
      joint_position_equality.mutable_joint_positions()->mutable_joints());
  return joint_position_equality;
}

// Helper method for the PositionBoundingBox Constraint
intrinsic_proto::Point ToPoint(const std::vector<double>& v) {
  CHECK_EQ(v.size(), 3);
  intrinsic_proto::Point p;
  p.set_x(v[0]);
  p.set_y(v[1]);
  p.set_z(v[2]);
  return p;
}

using ::intrinsic_proto::world::TransformNodeReference;
// Set frame to root, flange or gripper.
void SetFrame(TransformNodeReference* ref, const std::string& frame_label) {
  if (frame_label == "world_origin") {
    ref->mutable_by_name()->mutable_object()->set_object_name(
        RootObjectName().value());
  } else if (frame_label == "flange") {
    ref->mutable_by_name()->mutable_frame()->set_object_name("agilus-04");
    ref->mutable_by_name()->mutable_frame()->set_frame_name("flange");
  } else {
    CHECK_EQ(frame_label, "gripper");
    ref->mutable_by_name()->mutable_frame()->set_object_name("agilus-04::tool");
    ref->mutable_by_name()->mutable_frame()->set_frame_name("gripper");
  }
}

intrinsic_proto::motion_planning::v1::PositionBoundingBox
CreatePositionBoundingBoxProto(const std::vector<double>& lower_bounds,
                               const std::vector<double>& upper_bounds,
                               const std::string& target_frame,
                               const std::string& moving_frame) {
  intrinsic_proto::motion_planning::v1::PositionBoundingBox proto;
  *proto.mutable_lower_bounds() = ToPoint(lower_bounds);
  *proto.mutable_upper_bounds() = ToPoint(upper_bounds);

  SetFrame(proto.mutable_moving_frame(), moving_frame);
  SetFrame(proto.mutable_target_frame(), target_frame);
  proto.mutable_target_bounding_box_center()->mutable_position()->set_z(-1);
  proto.mutable_target_bounding_box_center()->mutable_orientation()->set_w(1);

  return proto;
}

intrinsic_proto::motion_planning::v1::PositionBoundingBox
CreatePositionBoundingBoxProto(const eigenmath::Vector3d& lower_bounds,
                               const eigenmath::Vector3d& upper_bounds,
                               const std::string& target_frame,
                               const std::string& moving_frame,
                               const Pose3d& target_t_bbox) {
  intrinsic_proto::motion_planning::v1::PositionBoundingBox proto;
  *proto.mutable_lower_bounds() = ToProto(lower_bounds);
  *proto.mutable_upper_bounds() = ToProto(upper_bounds);

  SetFrame(proto.mutable_moving_frame(), moving_frame);
  SetFrame(proto.mutable_target_frame(), target_frame);
  *proto.mutable_target_bounding_box_center() = ToProto(target_t_bbox);

  return proto;
}

}  // namespace intrinsic
