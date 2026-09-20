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

#include "intrinsic/motion_planning/service/motion_planner_cache_utils.h"

#include <queue>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/logging/proto/log_item.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_service.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_specification.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"
#include "intrinsic/world/objects/transform_node_internal.h"
#include "intrinsic/world/proto/attachment_component.pb.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/world.pb.h"

namespace intrinsic {

absl::Status ExtractIDWithPoseFromGeometricConstraint(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        top_level_constraint,
    absl::flat_hash_map<ObjectWorldResourceId, Pose3d>&
        transform_nodes_with_poses) {
  INTR_ASSIGN_OR_RETURN(const object_world::TransformNode* root_node,
                        object_world.GetObject(RootObjectId()));

  std::queue<intrinsic_proto::motion_planning::v1::GeometricConstraint>
      geometric_constraint_queue;
  geometric_constraint_queue.push(top_level_constraint);
  while (!geometric_constraint_queue.empty()) {
    const auto constraint = geometric_constraint_queue.front();
    geometric_constraint_queue.pop();

    intrinsic_proto::world::TransformNodeReference moving_frame_reference;
    intrinsic_proto::world::TransformNodeReference target_frame_reference;

    switch (constraint.constraint_case()) {
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          kPositionEquality: {
        moving_frame_reference = constraint.position_equality().moving_frame();
        target_frame_reference = constraint.position_equality().target_frame();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          kRotationEquality: {
        moving_frame_reference = constraint.rotation_equality().moving_frame();
        target_frame_reference = constraint.rotation_equality().target_frame();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          kRotationCone: {
        moving_frame_reference = constraint.rotation_cone().moving_frame();
        target_frame_reference = constraint.rotation_cone().target_frame();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          kCartesianPose: {
        moving_frame_reference = constraint.cartesian_pose().moving_frame();
        target_frame_reference = constraint.cartesian_pose().target_frame();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          kPositionBoundingBox: {
        moving_frame_reference =
            constraint.position_bounding_box().moving_frame();
        target_frame_reference =
            constraint.position_bounding_box().target_frame();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          kPointAt: {
        moving_frame_reference = constraint.point_at().moving_frame();
        target_frame_reference = constraint.point_at().target_frame();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          kConstraintIntersection: {
        for (const auto& element :
             constraint.constraint_intersection().constraints()) {
          geometric_constraint_queue.push(element);
        }
        continue;
      }
      default: {
        continue;
      }
    }

    INTR_ASSIGN_OR_RETURN(const object_world::TransformNode* moving_frame,
                          object_world::GetTransformNodeByReference(
                              object_world, moving_frame_reference));

    Pose3d root_to_this = Pose3d::Identity();
    if (moving_frame->GetParent() != nullptr) {
      INTR_ASSIGN_OR_RETURN(root_to_this,
                            root_node->GetTransform(moving_frame));
    }
    transform_nodes_with_poses.insert({moving_frame->GetId(), root_to_this});

    INTR_ASSIGN_OR_RETURN(const object_world::TransformNode* target_frame,
                          object_world::GetTransformNodeByReference(
                              object_world, target_frame_reference));
    root_to_this = Pose3d::Identity();
    if (target_frame->GetParent() != nullptr) {
      INTR_ASSIGN_OR_RETURN(root_to_this,
                            root_node->GetTransform(target_frame));
    }
    transform_nodes_with_poses.insert({target_frame->GetId(), root_to_this});
  }

  return absl::OkStatus();
}

absl::Status ExtractIDWithPoseFromMotionSpecification(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::MotionSpecification&
        motion_specification,
    absl::flat_hash_map<ObjectWorldResourceId, Pose3d>&
        transform_nodes_with_poses) {
  for (const auto& segment : motion_specification.motion_segments()) {
    INTR_RETURN_IF_ERROR(ExtractIDWithPoseFromMotionSegment(
        object_world, segment, transform_nodes_with_poses));
  }
  return absl::OkStatus();
}

absl::Status ExtractIDWithPoseFromMotionSegment(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::MotionSegment& motion_segment,
    absl::flat_hash_map<ObjectWorldResourceId, Pose3d>&
        transform_nodes_with_poses) {
  return ExtractIDWithPoseFromGeometricConstraint(
      object_world, motion_segment.target(), transform_nodes_with_poses);
}

}  // namespace intrinsic
