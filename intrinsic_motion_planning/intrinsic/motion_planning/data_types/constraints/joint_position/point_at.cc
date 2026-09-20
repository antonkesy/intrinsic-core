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

#include "intrinsic/motion_planning/data_types/constraints/joint_position/point_at.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/numopt/function_linearizer_numdiff.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"
#include "intrinsic/world/objects/transform_node_internal.h"

namespace intrinsic {
namespace motion_planning {

using ::intrinsic_proto::motion_planning::v1::PointAt;

namespace {

absl::Status ValidateConstraint(const object_world::ObjectWorld& world,
                                const PointAt& constraint) {
  if (constraint.has_tolerance()) {
    if (constraint.tolerance() < 0.0) {
      return absl::InvalidArgumentError("Tolerance must not be negative.");
    }
    if (constraint.tolerance() < PointAtConstraint::kDefaultTolerance) {
      // Don't allow a tolerance less than the default since its difficult to
      // satisfy the constraint with a very small tolerance.
      return absl::InvalidArgumentError(
          absl::StrCat("Tolerance must not be less than ",
                       PointAtConstraint::kDefaultTolerance));
    }
  }
  if (constraint.has_moving_axis()) {
    const eigenmath::Vector3d moving_axis = FromProto(constraint.moving_axis());
    if (moving_axis.isZero()) {
      return absl::InvalidArgumentError("Moving axis must not be zero.");
    }
  }

  if (constraint.has_min_distance() && constraint.min_distance() < 0.0) {
    return absl::InvalidArgumentError("Min distance must not be negative.");
  }

  if (constraint.has_max_distance() && constraint.max_distance() < 0.0) {
    return absl::InvalidArgumentError("Max distance must not be negative.");
  }
  if (constraint.has_max_distance() && constraint.has_min_distance() &&
      constraint.min_distance() > constraint.max_distance()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Min distance must not be greater than max distance: min:",
        constraint.min_distance(), ", max: ", constraint.max_distance()));
  }

  return absl::OkStatus();
}

eigenmath::VectorXd CreateToleranceVector(
    const double tolerance, const std::optional<double> min_distance,
    const std::optional<double> max_distance) {
  std::vector<double> tolerances;
  tolerances.push_back(tolerance);

  if (min_distance.has_value() || max_distance.has_value()) {
    tolerances.push_back(PointAtConstraint::kDefaultTolerance);
  }
  return eigenmath::VectorXd::Map(tolerances.data(), tolerances.size());
}

}  // namespace

absl::StatusOr<std::unique_ptr<PointAtConstraint>> PointAtConstraint::Create(
    const object_world::ObjectWorld& world, const PointAt& constraint) {
  INTR_RETURN_IF_ERROR(ValidateConstraint(world, constraint));

  const double tolerance =
      constraint.has_tolerance() ? constraint.tolerance() : kDefaultTolerance;
  const eigenmath::Vector3d moving_axis_in_tip_frame =
      constraint.has_moving_axis()
          ? FromProto(constraint.moving_axis()).normalized()
          : eigenmath::Vector3d::UnitZ();

  INTR_ASSIGN_OR_RETURN(
      const object_world::TransformNode* moving_frame_node,
      GetTransformNodeByReference(world, constraint.moving_frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId moving_id,
                        moving_frame_node->GetTransformOriginEntityId());

  INTR_ASSIGN_OR_RETURN(
      const object_world::TransformNode* target_frame_node,
      GetTransformNodeByReference(world, constraint.target_frame()));
  INTR_ASSIGN_OR_RETURN(AttachmentEntityId target_id,
                        target_frame_node->GetTransformOriginEntityId());

  const eigenmath::Vector3d tip_p_point =
      FromProto(constraint.moving_frame_offset());
  eigenmath::Vector3d target_p_offset =
      FromProto(constraint.target_frame_offset());
  // When target and moving are the same (i.e. the user only specifies the
  // difference between the target and moving frame by the offsets), check if a
  // robot exists between the world root and the moving. If a robot exists,
  // update the target id to root and calculate the target offset relative to
  // the root.
  if (target_id == moving_id) {
    INTR_ASSIGN_OR_RETURN(
        auto robot_ids,
        world.GetEntityWorld().GetRobotIdsInChain(kRootEntityId, moving_id));

    if (robot_ids.empty()) {
      return absl::NotFoundError(
          "Reference and moving frames are the same but no robot exists "
          "between the World root and the moving frame.");
    }

    const Pose3d root_t_target =
        world.GetEntityWorld().GetTransform(kRootEntityId, target_id);
    target_id = kRootEntityId;
    target_p_offset = root_t_target * target_p_offset;
  } else {
    const auto common_ancestor =
        world.GetEntityWorld().FindCommonAncestor(target_id, moving_id);
    // Check if moving_id is an ancestor of target_id. If so, the target_id
    // would move with the moving_id and we would not be able to find a
    // solution. To solve this, we change the target_id to the root and
    // re-calculate the target_p_offset relative to the root.
    // A use case for this is when a mounted camera is the moving_id
    // and a pose estimation result is the target_id. Now, the user might want
    // to look closer/directly at the pose estimation result.
    if (common_ancestor.status().ok() && common_ancestor.value() == moving_id) {
      INTR_ASSIGN_OR_RETURN(
          auto robot_ids,
          world.GetEntityWorld().GetRobotIdsInChain(kRootEntityId, moving_id));

      if (robot_ids.empty()) {
        return absl::NotFoundError(
            "Reference is a descendant of moving frame but no "
            "robot exists between the World root and the moving frame.");
      }
      const Pose3d root_t_target =
          world.GetEntityWorld().GetTransform(kRootEntityId, target_id);
      target_id = kRootEntityId;
      target_p_offset = root_t_target * target_p_offset;
    }
  }
  // Construct the kinematic chain needed to evaluate forward kinematics.
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<kinematics::ModelInterface> kinematic_model,
      world.GetEntityWorld().BuildChainSkeleton(target_id, moving_id));

  // Use `WrapUnique` because the constructor is private.
  return absl::WrapUnique(new PointAtConstraint(
      std::move(kinematic_model), target_p_offset, tip_p_point,
      moving_axis_in_tip_frame, tolerance,
      constraint.has_min_distance()
          ? std::optional<double>{constraint.min_distance()}
          : std::optional<double>{},
      constraint.has_max_distance()
          ? std::optional<double>{constraint.max_distance()}
          : std::optional<double>{}));
}

PointAtConstraint::PointAtConstraint(
    std::unique_ptr<kinematics::ModelInterface> kinematic_model,
    const eigenmath::Vector3d& target_p_offset,
    const eigenmath::Vector3d& tip_p_point,
    const eigenmath::Vector3d& moving_axis_in_tip_frame, const double tolerance,
    const std::optional<double> min_distance,
    const std::optional<double> max_distance)
    : constraint_dimensions_(
          (min_distance.has_value() || max_distance.has_value() ? 2 : 1)),
      kinematic_model_(std::move(kinematic_model)),
      kinematic_state_(kinematics::State(kinematic_model_.get())),
      tip_element_id_(kinematic_model_->GetTipIds().front()),
      target_p_offset_(target_p_offset),
      tip_p_point_(tip_p_point),
      moving_axis_in_tip_frame_(moving_axis_in_tip_frame),
      max_distance_to_ray_(tolerance),
      min_distance_(min_distance),
      max_distance_(max_distance),
      tolerance_vector_(
          CreateToleranceVector(tolerance, min_distance, max_distance)) {}

namespace {

struct ClosestPointOnLineResult {
  // Closest point on the line. Could also be on the opposite side of the
  // `line_point` and `line_direction`.
  eigenmath::Vector3d point;
  // If positive, the point is on the side in direction of the `line_direction`.
  // If negative, the point is on the opposite side of the `line_point`, e.g.
  // behind the camera/light source. If zero, the `line_point` and `point` are
  // the same.
  double signed_distance_from_line_point;
};

// Computes the closest point on the line defined by `line_point` and
// `line_direction` to `point`.
ClosestPointOnLineResult ClosestPointOnLine(
    const eigenmath::Vector3d& point, const eigenmath::Vector3d& line_point,
    const eigenmath::Vector3d& line_direction) {
  const eigenmath::Vector3d dir = line_direction.normalized();
  const eigenmath::Vector3d v = point - line_point;
  const double t = v.dot(dir);
  return ClosestPointOnLineResult{line_point + t * dir, t};
}

// Computes the distance from `point` to the ray defined by `ray_origin` and
// `ray_direction`. If the ray points away from the point (angle > 90 degrees),
// the distance is adjusted to be the distance to point on the opposite side of
// the ray plus the distance from `ray_origin` to `point`. This is done to
// penalize rays pointing away from the point of interest.
double DistancePointToRay(const eigenmath::Vector3d& point,
                          const eigenmath::Vector3d& ray_origin,
                          const eigenmath::Vector3d& ray_direction) {
  const ClosestPointOnLineResult result =
      ClosestPointOnLine(point, ray_origin, ray_direction);
  const double distance_to_ray = (point - result.point).norm();
  if (result.signed_distance_from_line_point < 0.0) {
    // At 90 degrees from the optimal direction (ray goes directly through the
    // point), the distance is (point - ray_origin).norm(), which is the largest
    // distance a point can be from our line. If the direction of the ray shows
    // away from the point, the distance from point to line becomes smaller
    // again. Since we don't want the ray to point away from the point of
    // interest, we want to penalize if its showing in the wrong direction. If
    // the ray points in the opposite direction, we return the largest distance,
    // which is `2*(point - ray_origin).norm()` since distance_to_ray is then
    // zero.
    const double adjusted_distance =
        2 * (point - ray_origin).norm() - distance_to_ray;
    return adjusted_distance;
  }
  return distance_to_ray;
}

}  // namespace

absl::StatusOr<eigenmath::VectorXd> PointAtConstraint::Evaluate(
    const eigenmath::VectorXd& joint_positions) {
  INTRINSIC_RT_RETURN_IF_ERROR(kinematic_state_.SetDofPositions(
      joint_positions, /*check_limits=*/false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Pose3d target_t_tip,
                                kinematic_state_.GetTransform(tip_element_id_));
  const eigenmath::Vector3d target_p_tip_point = target_t_tip * tip_p_point_;
  const eigenmath::Vector3d direction_in_target_frame =
      target_t_tip.quaternion() * moving_axis_in_tip_frame_;
  const double distance_to_ray = DistancePointToRay(
      target_p_offset_, target_p_tip_point, direction_in_target_frame);

  // Sanity check if the constructor did its work correctly.
  if (constraint_dimensions_ == 0) [[unlikely]] {
    return absl::InternalError(
        "Problem during the constraint evaluation: Constraint dimension is "
        "zero. Please file a bug ticket.");
  }

  // If min and max distance are not set, the constraint has only 1 dimension
  // and we do not need to compute the distance to the target.
  if (constraint_dimensions_ == 1) {
    return eigenmath::VectorXd::Constant(constraint_dimensions_,
                                         distance_to_ray);
  }

  eigenmath::VectorXd residual(constraint_dimensions_);
  residual[0] = distance_to_ray;
  const double distance_tip_to_target =
      (target_p_tip_point - target_p_offset_).norm();
  if (!min_distance_.has_value() && !max_distance_.has_value()) [[unlikely]] {
    return absl::InternalError(
        "Constraint dimension is 2 but min and max "
        "distance are not set. This is a bug.");
  }
  double distance_to_inside_limits =
      -1e7;  // Very satisfied. But should always be overwritten. If not, this
             // is a bug.
  // Min distance is also modeled as a <= constraint:
  // If the distance is greater than the min distance, the residual is
  // negative to satisfy the <= constraint.
  if (min_distance_.has_value()) {
    distance_to_inside_limits = *min_distance_ - distance_tip_to_target;
  }
  // If the distance is less than the max distance, the residual is
  // negative to satisfy the <= constraint.
  if (max_distance_.has_value()) {
    distance_to_inside_limits = std::max(
        distance_to_inside_limits, distance_tip_to_target - *max_distance_);
  }
  residual[1] = distance_to_inside_limits;

  return residual;
}

absl::StatusOr<eigenmath::MatrixXd> PointAtConstraint::Gradient(
    const eigenmath::VectorXd& joint_positions) {
  return ComputeDerivative(
      // The num-diff linearizer does not support status codes, so Evaluate
      // must be wrapped in a lambda.
      [this](const eigenmath::VectorXd& inner_joint_positions) {
        auto value_or = this->Evaluate(inner_joint_positions);
        CHECK_OK(value_or.status());
        return *value_or;
      },
      joint_positions,
      /*double_sided_derivative=*/false);
}

absl::StatusOr<bool> PointAtConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_positions) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_positions));
  return (residual.array() <= tolerance_vector_.array()).all();
}

ConstraintInterface::ConstraintType PointAtConstraint::Type() const {
  if (constraint_dimensions_ == 1 &&
      max_distance_to_ray_ <= kDefaultTolerance) {
    return GENERAL_EQUALITY;
  }
  return GENERAL_INEQUALITY;
}

const eigenmath::VectorXd& PointAtConstraint::Tolerance() const {
  return tolerance_vector_;
}
}  // namespace motion_planning
}  // namespace intrinsic
