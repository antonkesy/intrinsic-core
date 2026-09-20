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

#include "intrinsic/kinematics/ik/constrained/constrained_ik_proto_conversions.h"

#include <memory>
#include <string>

#include "absl/log/die_if_null.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik.pb.h"
#include "intrinsic/kinematics/ik/constrained/constraints.h"
#include "intrinsic/kinematics/ik/constrained/cost_functions.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::kinematics {

namespace {

// A helper for checking if requested 'element_id' exists in 'chain'.
absl::Status CheckIfElementExists(const Chain* chain, ElementId element_id) {
  if (!ABSL_DIE_IF_NULL(chain)->GetElement(element_id).ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("Requested element with id ",
                     std::to_string(static_cast<int>(element_id)),
                     " does not exist in chain ", chain->GetName()));
  }

  return absl::OkStatus();
}

}  // namespace

// Converts joint limits constraint in proto format to JointStateP.
// Returns 'kInvalidArgument' if any joint dimensions are inconsistent.
absl::Status FromProto(
    const intrinsic_proto::kinematics::JointPositionLimitsConstraint& proto,
    JointStateP& min_position, JointStateP& max_position) {
  INTR_RETURN_IF_ERROR(
      min_position.SetSize(proto.lower_limits().joints_size()));
  INTR_ASSIGN_OR_RETURN(min_position.position,
                        icon::FromProto(proto.lower_limits()));
  INTR_RETURN_IF_ERROR(
      max_position.SetSize(proto.upper_limits().joints_size()));
  INTR_ASSIGN_OR_RETURN(max_position.position,
                        icon::FromProto(proto.upper_limits()));
  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<PointConstraint>> FromProto(
    const intrinsic_proto::kinematics::PointConstraint& proto,
    const Chain* chain) {
  ElementId robot_frame_id = FromRobotFrame(proto.robot_frame());
  INTR_RETURN_IF_ERROR(CheckIfElementExists(chain, robot_frame_id));

  if (!proto.has_base_p_target_desired()) {
    return absl::FailedPreconditionError("Proto missing base_p_target_desired");
  }
  eigenmath::Vector3d base_p_target_desired =
      FromProto(proto.base_p_target_desired());

  eigenmath::Vector3d robot_frame_p_target = eigenmath::Vector3d::Zero();
  if (proto.has_robot_frame_p_target()) {
    robot_frame_p_target = FromProto(proto.robot_frame_p_target());
  }

  eigenmath::Vector3d tolerance = eigenmath::Vector3d::Constant(1e-4);
  if (proto.has_max_position_error()) {
    tolerance.setConstant(proto.max_position_error());
  }

  return PointConstraint::Create(chain, base_p_target_desired, robot_frame_id,
                                 robot_frame_p_target, tolerance);
}

absl::StatusOr<std::unique_ptr<PoseConstraint>> FromProto(
    const intrinsic_proto::kinematics::PoseConstraint& proto,
    const Chain* chain) {
  ElementId robot_frame_id = FromRobotFrame(proto.robot_frame());
  INTR_RETURN_IF_ERROR(CheckIfElementExists(chain, robot_frame_id));

  if (!proto.has_base_t_target_desired()) {
    return absl::FailedPreconditionError("Proto missing base_t_target_desired");
  }
  INTR_ASSIGN_OR_RETURN(
      Pose3d base_t_target_desired,
      intrinsic_proto::FromProtoNormalized(proto.base_t_target_desired()));

  Pose3d robot_frame_t_target = Pose3d::Identity();
  if (proto.has_robot_frame_t_target()) {
    INTR_ASSIGN_OR_RETURN(
        robot_frame_t_target,
        intrinsic_proto::FromProtoNormalized(proto.robot_frame_t_target()));
  }

  eigenmath::Vector6d tolerance;
  tolerance.head<3>().setConstant(1e-4);  // translational tolerance default.
  if (proto.has_max_position_error()) {
    tolerance.head<3>().setConstant(proto.max_position_error());
  }
  tolerance.tail<3>().setConstant(1e-3);  // rotational tolerance default.
  if (proto.max_angle_deviation()) {
    tolerance.tail<3>().setConstant(proto.max_angle_deviation());
  }

  return PoseConstraint::Create(chain, base_t_target_desired, robot_frame_id,
                                robot_frame_t_target, tolerance);
}

absl::StatusOr<std::unique_ptr<EllipsoidConstraint>> FromProto(
    const intrinsic_proto::kinematics::PositionEllipsoidConstraint& proto,
    const Chain* chain) {
  ElementId root_frame_id = FromRobotFrame(proto.root_frame());
  INTR_RETURN_IF_ERROR(CheckIfElementExists(chain, root_frame_id));
  ElementId target_frame_id = FromRobotFrame(proto.target_frame());
  INTR_RETURN_IF_ERROR(CheckIfElementExists(chain, target_frame_id));

  if (!proto.has_root_t_target_ellipsoid()) {
    return absl::FailedPreconditionError(
        "Proto missing root_t_target_ellipsoid");
  }
  INTR_ASSIGN_OR_RETURN(Pose3d root_t_target_ellipsoid,
                        FromProto(proto.root_t_target_ellipsoid()));

  eigenmath::Vector3d ellipsoid_half_axes;
  ellipsoid_half_axes.x() = proto.rx();
  ellipsoid_half_axes.y() = proto.ry();
  ellipsoid_half_axes.z() = proto.rz();

  eigenmath::Vector3d p_in_target_frame = eigenmath::Vector3d::Zero();
  if (proto.has_p_in_target_frame()) {
    p_in_target_frame = FromProto(proto.p_in_target_frame());
  }

  double tolerance = 1e-3;
  if (proto.has_tolerance()) {
    tolerance = proto.tolerance();
  }

  return EllipsoidConstraint::Create(
      chain, root_frame_id, target_frame_id, root_t_target_ellipsoid,
      ellipsoid_half_axes, p_in_target_frame, tolerance);
}

absl::StatusOr<std::unique_ptr<PlaneConstraint>> FromProto(
    const intrinsic_proto::kinematics::PlaneConstraint& proto,
    const Chain* chain) {
  ElementId target_frame_id = FromRobotFrame(proto.target_frame());
  INTR_RETURN_IF_ERROR(CheckIfElementExists(chain, target_frame_id));

  if (!proto.has_base_p_point_on_plane()) {
    return absl::FailedPreconditionError("Proto missing base_p_point_on_plane");
  }
  eigenmath::Vector3d base_p_point_on_plane =
      FromProto(proto.base_p_point_on_plane());

  if (!proto.has_plane_normal_in_base_frame()) {
    return absl::FailedPreconditionError(
        "Proto missing plane_normal_in_base_frame");
  }
  eigenmath::Vector3d plane_normal_in_base_frame =
      FromProto(proto.plane_normal_in_base_frame());

  eigenmath::Vector3d target_frame_p_offset = eigenmath::Vector3d::Zero();
  if (proto.has_target_frame_p_offset()) {
    target_frame_p_offset = FromProto(proto.target_frame_p_offset());
  }

  double tolerance = 1e-3;
  if (proto.has_tolerance()) {
    tolerance = proto.tolerance();
  }

  return PlaneConstraint::Create(chain, base_p_point_on_plane,
                                 plane_normal_in_base_frame, target_frame_id,
                                 target_frame_p_offset, tolerance);
}

absl::StatusOr<std::unique_ptr<OrientationWithFreeAxisConstraint>> FromProto(
    const intrinsic_proto::kinematics::OrientationWithFreeAxisConstraint& proto,
    const Chain* chain) {
  ElementId robot_frame_id = FromRobotFrame(proto.robot_frame());
  INTR_RETURN_IF_ERROR(CheckIfElementExists(chain, robot_frame_id));

  if (!proto.has_target_axis_in_base_frame()) {
    return absl::FailedPreconditionError(
        "Proto missing target_axis_in_base_frame");
  }
  eigenmath::Vector3d target_axis_in_base_frame =
      FromProto(proto.target_axis_in_base_frame());

  eigenmath::Vector3d axis_to_align_in_target_frame =
      eigenmath::Vector3d::Zero();
  if (proto.has_axis_to_align_in_target_frame()) {
    axis_to_align_in_target_frame =
        FromProto(proto.axis_to_align_in_target_frame());
  }

  Pose3d robot_frame_t_target = Pose3d::Identity();
  if (proto.has_robot_frame_t_target()) {
    INTR_ASSIGN_OR_RETURN(
        robot_frame_t_target,
        intrinsic_proto::FromProtoNormalized(proto.robot_frame_t_target()));
  }

  double tolerance_rad = 1e-3;
  if (proto.has_tolerance_rad()) {
    tolerance_rad = proto.tolerance_rad();
  }

  return OrientationWithFreeAxisConstraint::Create(
      chain, target_axis_in_base_frame, robot_frame_id,
      axis_to_align_in_target_frame, robot_frame_t_target, tolerance_rad);
}

absl::StatusOr<std::unique_ptr<OrientationConeConstraint>> FromProto(
    const intrinsic_proto::kinematics::OrientationConeConstraint& proto,
    const Chain* chain) {
  ElementId robot_frame_id = FromRobotFrame(proto.robot_frame());
  INTR_RETURN_IF_ERROR(CheckIfElementExists(chain, robot_frame_id));

  if (!proto.has_cone_axis_in_base_frame()) {
    return absl::FailedPreconditionError(
        "Proto missing cone_axis_in_base_frame");
  }
  eigenmath::Vector3d cone_axis_in_base_frame =
      FromProto(proto.cone_axis_in_base_frame());

  eigenmath::Vector3d axis_to_align_in_target_frame =
      eigenmath::Vector3d::Zero();
  if (proto.has_axis_to_align_in_target_frame()) {
    axis_to_align_in_target_frame =
        FromProto(proto.axis_to_align_in_target_frame());
  }

  Pose3d robot_frame_t_target = Pose3d::Identity();
  if (proto.has_robot_frame_t_target()) {
    INTR_ASSIGN_OR_RETURN(
        robot_frame_t_target,
        intrinsic_proto::FromProtoNormalized(proto.robot_frame_t_target()));
  }

  double tolerance_rad = 1e-3;
  if (proto.has_tolerance_rad()) {
    tolerance_rad = proto.tolerance_rad();
  }

  return OrientationConeConstraint::Create(
      chain, cone_axis_in_base_frame, proto.cone_opening_half_angle_rad(),
      robot_frame_id, axis_to_align_in_target_frame, robot_frame_t_target,
      tolerance_rad);
}

absl::StatusOr<std::unique_ptr<JointPositionCost>> FromProto(
    const intrinsic_proto::kinematics::JointPositionCost& proto,
    const Chain* chain) {
  INTR_ASSIGN_OR_RETURN(eigenmath::VectorNd xref,
                        icon::FromProto(proto.target_joint_position()));
  INTR_ASSIGN_OR_RETURN(eigenmath::MatrixXd q_weight,
                        FromProto(proto.q_matrix()));
  return JointPositionCost::Create(chain, xref, q_weight);
}

absl::StatusOr<std::unique_ptr<ManipulabilityCost>> FromProto(
    const intrinsic_proto::kinematics::ManipulabilityCost& proto,
    const Chain* chain) {
  ElementId robot_frame_id = FromRobotFrame(proto.robot_frame());
  INTR_RETURN_IF_ERROR(CheckIfElementExists(chain, robot_frame_id));

  return ManipulabilityCost::Create(chain, robot_frame_id, proto.weight());
}

intrinsic_proto::kinematics::RobotFrame ToRobotFrame(ElementId element_id) {
  intrinsic_proto::kinematics::RobotFrame proto;
  proto.set_element_id(element_id.value());
  return proto;
}

ElementId FromRobotFrame(
    const intrinsic_proto::kinematics::RobotFrame& robot_frame) {
  return ElementId(robot_frame.element_id());
}

}  // namespace intrinsic::kinematics
