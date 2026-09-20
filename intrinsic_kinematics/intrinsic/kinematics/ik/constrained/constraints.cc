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

#include "intrinsic/kinematics/ik/constrained/constraints.h"

#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

#include "absl/log/check.h"
#include "absl/log/die_if_null.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/math.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/numopt/function_linearizer_numdiff.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::kinematics {

namespace {

// Returns true if 'v' has unit length with given 'tolerance', returns false
// otherwise.
bool VectorIsNormalized(const eigenmath::VectorXd& v, double tolerance = 1e-6) {
  return (std::abs(v.norm() - 1.0) < tolerance);
}

// A helper to reduce code duplication for evaluating forward kinematics.
icon::RealtimeStatusOr<Pose3d> EvaluateForwardKinematics(
    const eigenmath::VectorNd& joint_angles, ElementId robot_frame_id,
    State& state, const Pose3d& robot_frame_t_target = Pose3d::Identity()) {
  INTRINSIC_RT_RETURN_IF_ERROR(
      state.SetDofPositions(joint_angles, /*check_limits=*/false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(Pose3d base_t_robot_frame,
                                state.GetTransform(robot_frame_id));
  return Pose3d(base_t_robot_frame * robot_frame_t_target);
}

// A helper to reduce code duplication for evaluating the Jacobian.
icon::RealtimeStatusOr<eigenmath::Matrix6Nd> EvaluateJacobian(
    const eigenmath::VectorXd& q, const kinematics::ElementId frame_id,
    const eigenmath::Vector3d& frame_p_target, State& state) {
  INTRINSIC_RT_RETURN_IF_ERROR(
      state.SetDofPositions(q, /*check_limits=*/false));
  return state.ComputeJacobian(frame_id, frame_p_target);
}

}  // namespace

PointConstraint::PointConstraint(
    const Chain* chain, const eigenmath::Vector3d& base_p_target,
    kinematics::ElementId robot_frame_id,
    const eigenmath::Vector3d& robot_frame_p_target,
    const eigenmath::Vector3d& tolerance)
    : base_p_target_(base_p_target),
      state_(ABSL_DIE_IF_NULL(chain)),
      robot_frame_id_(robot_frame_id),
      robot_frame_p_target_(robot_frame_p_target),
      tolerance_(tolerance) {}

/*static*/
absl::StatusOr<std::unique_ptr<PointConstraint>> PointConstraint::Create(
    const Chain* chain, const eigenmath::Vector3d& base_p_target,
    kinematics::ElementId robot_frame_id,
    const eigenmath::Vector3d& robot_frame_p_target,
    const eigenmath::Vector3d& tolerance) {
  if (tolerance.minCoeff() < 0.0) {
    return absl::FailedPreconditionError(
        "Invalid tolerance, must not be negative.");
  }

  // Check if requested frame exists in chain.
  if (!ABSL_DIE_IF_NULL(chain)->GetElement(robot_frame_id).ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("Requested element with id ",
                     std::to_string(static_cast<int>(robot_frame_id)),
                     " does not exist in chain ", chain->GetName()));
  }

  // Using new in order to access private constructor.
  return absl::WrapUnique(new PointConstraint(
      chain, base_p_target, robot_frame_id, robot_frame_p_target, tolerance));
}

absl::StatusOr<eigenmath::VectorXd> PointConstraint::Evaluate(
    const eigenmath::VectorXd& joint_angles) {
  if (!joint_angles.allFinite()) {
    return absl::InternalError("Joint angles are not finite.");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const Pose3d base_t_frame,
      EvaluateForwardKinematics(joint_angles, robot_frame_id_, state_));

  return (base_t_frame *
          Pose3d(eigenmath::Quaterniond::Identity(), robot_frame_p_target_))
             .translation() -
         base_p_target_;
}

absl::StatusOr<eigenmath::MatrixXd> PointConstraint::Gradient(
    const eigenmath::VectorXd& joint_angles) {
  if (!joint_angles.allFinite()) {
    return absl::InternalError("Joint angles are not finite.");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto jac, EvaluateJacobian(joint_angles, robot_frame_id_,
                                 robot_frame_p_target_, state_));
  return eigenmath::MatrixXd(jac.topRows(kConstraintDim));
}

absl::StatusOr<bool> PointConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_angles) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_angles));
  return (residual.array().abs() <= tolerance_.array()).all();
}

PoseConstraint::PoseConstraint(const Chain* chain,
                               const Pose3d& base_t_target_desired,
                               kinematics::ElementId robot_frame_id,
                               const Pose3d& robot_frame_t_target,
                               const eigenmath::Vector6d& tolerance)
    : base_t_target_desired_(base_t_target_desired),
      state_(ABSL_DIE_IF_NULL(chain)),
      robot_frame_id_(robot_frame_id),
      robot_frame_t_target_(robot_frame_t_target),
      tolerance_(tolerance) {}

/*static*/
absl::StatusOr<std::unique_ptr<PoseConstraint>> PoseConstraint::Create(
    const Chain* chain, const Pose3d& base_t_target_desired,
    kinematics::ElementId robot_frame_id, const Pose3d& robot_frame_t_target,
    const eigenmath::Vector6d& tolerance) {
  if (tolerance.minCoeff() < 0.0) {
    return absl::FailedPreconditionError(
        "Invalid tolerance, must not be negative.");
  }

  if (!ABSL_DIE_IF_NULL(chain)->GetElement(robot_frame_id).ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ElementId ", static_cast<int>(robot_frame_id),
                     " does not exist in chain '", chain->GetName(), "."));
  }

  // Using new in order to access private constructor.
  return absl::WrapUnique(new PoseConstraint(chain, base_t_target_desired,
                                             robot_frame_id,
                                             robot_frame_t_target, tolerance));
}

absl::StatusOr<eigenmath::VectorXd> PoseConstraint::Evaluate(
    const eigenmath::VectorXd& joint_angles) {
  if (!joint_angles.allFinite()) {
    return absl::InternalError("Joint angles are not finite.");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto base_t_frame,
      EvaluateForwardKinematics(joint_angles, robot_frame_id_, state_));
  return EvaluatePoseError(base_t_frame * robot_frame_t_target_,
                           base_t_target_desired_);
}

absl::StatusOr<eigenmath::MatrixXd> PoseConstraint::Gradient(
    const eigenmath::VectorXd& joint_angles) {
  if (!joint_angles.allFinite()) {
    return absl::InternalError("Joint angles are not finite.");
  }
  return ComputeDerivative(
      // Passing a lambda instead of the class function is required due to
      // missing treatment of status codes in the num-diff linearizer.
      [this](const eigenmath::VectorXd& joint_angles) {
        auto value_or = this->Evaluate(joint_angles);
        CHECK_OK(value_or);
        return *value_or;
      },
      joint_angles, /*double_sided_derivative=*/false);
}

absl::StatusOr<bool> PoseConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_angles) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_angles));
  return (residual.array().abs() <= tolerance_.array()).all();
}

OrientationWithFreeAxisConstraint::OrientationWithFreeAxisConstraint(
    const Chain* chain, const eigenmath::Vector3d& free_axis_in_base_frame,
    kinematics::ElementId robot_frame_id,
    const eigenmath::Vector3d& axis_to_align_in_target_frame,
    const Pose3d& robot_frame_t_target, double tolerance_radians)
    : state_(ABSL_DIE_IF_NULL(chain)),
      free_axis_in_base_frame_(free_axis_in_base_frame),
      axis_to_align_in_target_frame_(axis_to_align_in_target_frame),
      robot_frame_id_(robot_frame_id),
      robot_frame_t_target_(robot_frame_t_target),
      // We need to translate the "angular" tolerance in radians into a
      // tolerance in the space of the dot-product:
      tolerance_(eigenmath::VectorXd::Constant(
          kConstraintDim, 1.0 - std::cos(tolerance_radians))) {}

/*static*/
absl::StatusOr<std::unique_ptr<OrientationWithFreeAxisConstraint>>
OrientationWithFreeAxisConstraint::Create(
    const Chain* chain, const eigenmath::Vector3d& free_axis_in_base_frame,
    kinematics::ElementId robot_frame_id,
    const eigenmath::Vector3d& axis_to_align_in_target_frame,
    const Pose3d& robot_frame_t_target, double tolerance_radians) {
  if (!VectorIsNormalized(free_axis_in_base_frame)) {
    return absl::FailedPreconditionError("Free rotation axis not normalized.");
  }

  if (!VectorIsNormalized(axis_to_align_in_target_frame)) {
    return absl::FailedPreconditionError("Axis to align not normalized.");
  }

  if (tolerance_radians < 0.0) {
    return absl::FailedPreconditionError(
        "Invalid tolerance, must not be negative.");
  }

  if (!ABSL_DIE_IF_NULL(chain)->GetElement(robot_frame_id).ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ElementId ", static_cast<int>(robot_frame_id),
                     " does not exist in chain ", chain->GetName()));
  }

  // Uses 'new' to access private constructor.
  return absl::WrapUnique(new OrientationWithFreeAxisConstraint(
      chain, free_axis_in_base_frame, robot_frame_id,
      axis_to_align_in_target_frame, robot_frame_t_target, tolerance_radians));
}

absl::StatusOr<eigenmath::VectorXd> OrientationWithFreeAxisConstraint::Evaluate(
    const eigenmath::VectorXd& joint_angles) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto base_t_target,
      EvaluateForwardKinematics(joint_angles, robot_frame_id_, state_,
                                robot_frame_t_target_));

  eigenmath::Vector3d target_axis_in_base_frame =
      base_t_target.rotationMatrix() * axis_to_align_in_target_frame_;

  // For this constraint to be satisfied, the angle between the two axes should
  // be zero.
  double dot_product = free_axis_in_base_frame_.dot(target_axis_in_base_frame);

  // The error angle in rad would be given as
  // double angle = acos(dot_product /
  // sqrt(free_axis_in_base_frame_.squaredNorm() *
  // ee_axis_in_base_frame.squaredNorm())); However, we can instead directly
  // optimize over '1.0- dot_product', which is equivalent, and even more
  // efficient and better conditioned numerically.
  return eigenmath::VectorXd::Constant(kConstraintDim, 1.0 - dot_product);
}

absl::StatusOr<eigenmath::MatrixXd> OrientationWithFreeAxisConstraint::Gradient(
    const eigenmath::VectorXd& joint_angles) {
  if (!joint_angles.allFinite()) {
    return absl::InternalError("Joint angles are not finite.");
  }

  // Numerically approximate the constraint gradient w.r.t. joint positions.
  return ComputeDerivative(
      // Passing a lambda instead of the class function is is required due to
      // missing treatment of status codes in the num-diff linearizer.
      [this](const eigenmath::VectorXd& joint_angles) {
        auto value_or = this->Evaluate(joint_angles);
        CHECK_OK(value_or.status());
        return *value_or;
      },
      joint_angles, /*double_sided_derivative=*/false);
}

absl::StatusOr<bool> OrientationWithFreeAxisConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_angles) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_angles));
  return (residual.array().abs() <= tolerance_.array()).all();
}

OrientationConeConstraint::OrientationConeConstraint(
    const Chain* chain, const eigenmath::Vector3d& cone_axis_in_base_frame,
    double cone_opening_half_angle, ElementId robot_frame_id,
    const eigenmath::Vector3d& axis_to_align_in_target_frame,
    const Pose3d& robot_frame_t_target, double tolerance_rad)
    // Directly evaluate cos(cone_opening_half_angle) since we only need this
    // quantity when evaluating the constraint.
    : cos_cone_opening_half_angle_(std::cos(cone_opening_half_angle)),
      state_(ABSL_DIE_IF_NULL(chain)),
      cone_axis_in_base_frame_(cone_axis_in_base_frame),
      axis_to_align_in_target_frame_(axis_to_align_in_target_frame),
      robot_frame_id_(robot_frame_id),
      robot_frame_t_target_(robot_frame_t_target),
      // Map the tolerance from radians to the dot-product space:
      tolerance_(eigenmath::VectorXd::Constant(
          kConstraintDim, 1.0 - std::cos(tolerance_rad))) {}

/*static*/
absl::StatusOr<std::unique_ptr<OrientationConeConstraint>>
OrientationConeConstraint::Create(
    const Chain* chain, const eigenmath::Vector3d& cone_axis_in_base_frame,
    double cone_half_angle, ElementId robot_frame_id,
    const eigenmath::Vector3d& axis_to_align_in_target_frame,
    const Pose3d& robot_frame_t_target, double tolerance_rad) {
  if (!VectorIsNormalized(cone_axis_in_base_frame)) {
    return absl::FailedPreconditionError("Cone axis not normalized.");
  }
  if (!VectorIsNormalized(axis_to_align_in_target_frame)) {
    return absl::FailedPreconditionError(
        "End effector axis to align not normalized.");
  }
  if (tolerance_rad < 0.0) {
    return absl::FailedPreconditionError(
        "Invalid tolerance, must not be negative.");
  }
  if (cone_half_angle < 0.0) {
    return absl::FailedPreconditionError(
        "Invalid cone half angle, must not be negative.");
  }

  if (!ABSL_DIE_IF_NULL(chain)->GetElement(robot_frame_id).ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ElementId ", static_cast<int>(robot_frame_id),
                     " does not exist in chain ", chain->GetName()));
  }

  // Uses 'new' to access private constructor.
  return absl::WrapUnique(new OrientationConeConstraint(
      chain, cone_axis_in_base_frame, cone_half_angle, robot_frame_id,
      axis_to_align_in_target_frame, robot_frame_t_target, tolerance_rad));
}

absl::StatusOr<eigenmath::VectorXd> OrientationConeConstraint::Evaluate(
    const eigenmath::VectorXd& joint_angles) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto base_t_target,
      EvaluateForwardKinematics(joint_angles, robot_frame_id_, state_,
                                robot_frame_t_target_));

  eigenmath::Vector3d target_axis_in_base_frame =
      base_t_target.rotationMatrix() * axis_to_align_in_target_frame_;

  // For this constraint to be satisfied, the angle between the two axes
  // should be less than the cone opening angle.
  double dot_product = cone_axis_in_base_frame_.dot(target_axis_in_base_frame);

  // The angle between cone axis and the desired end-effector axis would be
  // given as double angle = acos(dot_product /
  // sqrt(cone_axis_in_base_frame_.squaredNorm() *
  // frame_axis_in_base_frame.squaredNorm())), however, here we adopt a
  // numerically more favourable formulation which avoids the sqrt and the acos,
  // and work directly with the dot-product:

  return eigenmath::VectorXd::Constant(
      kConstraintDim, cos_cone_opening_half_angle_ - dot_product);
}

absl::StatusOr<eigenmath::MatrixXd> OrientationConeConstraint::Gradient(
    const eigenmath::VectorXd& joint_angles) {
  return ComputeDerivative(
      // Passing a lambda instead of the class function is required because of
      // missing compatibility of status codes in the num-diff linearizer.
      [this](const eigenmath::VectorXd& joint_angles) {
        auto value_or = this->Evaluate(joint_angles);
        CHECK_OK(value_or.status());
        return *value_or;
      },
      joint_angles,
      /*double_sided_derivative=*/false);
}

absl::StatusOr<bool> OrientationConeConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_angles) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_angles));
  return (residual.array() <= tolerance_.array()).all();
}

EllipsoidConstraint::EllipsoidConstraint(
    const Chain* chain, ElementId root_frame, ElementId target_frame,
    const Pose3d& root_t_goal_ellipsoid,
    const eigenmath::Vector3d& ellipsoid_half_axes_xyz,
    const eigenmath::Vector3d& target_frame_p_offset, double tolerance)
    : state_(chain),
      root_frame_(root_frame),
      target_frame_(target_frame),
      goal_ellipsoid_t_root_(root_t_goal_ellipsoid.inverse()),
      ellipsoid_half_axes_xyz_inverted_(1.0 / ellipsoid_half_axes_xyz.array()),
      target_frame_p_offset_(target_frame_p_offset),
      tolerance_(eigenmath::VectorXd::Constant(kConstraintDim, tolerance)) {}

/*static*/
absl::StatusOr<std::unique_ptr<EllipsoidConstraint>>
EllipsoidConstraint::Create(const Chain* chain, ElementId root_frame,
                            ElementId target_frame,
                            const Pose3d& root_t_goal_ellipsoid,
                            const eigenmath::Vector3d& ellipsoid_half_axes_xyz,
                            const eigenmath::Vector3d& target_frame_p_offset,
                            double tolerance) {
  if (!ABSL_DIE_IF_NULL(chain)->GetElement(root_frame).ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ElementId ", static_cast<int>(root_frame),
                     " does not exist in chain ", chain->GetName()));
  }
  if (!ABSL_DIE_IF_NULL(chain)->GetElement(target_frame).ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ElementId ", static_cast<int>(target_frame),
                     " does not exist in chain ", chain->GetName()));
  }

  if (ellipsoid_half_axes_xyz.minCoeff() < 0.0) {
    return absl::FailedPreconditionError(
        "All ellipsoid half axes must be >= 0.0");
  }

  if (tolerance < 0.0) {
    return absl::FailedPreconditionError(
        "Invalid tolerance, must not be negative.");
  }

  // Using 'new' to access private constructor.
  return absl::WrapUnique(new EllipsoidConstraint(
      chain, root_frame, target_frame, root_t_goal_ellipsoid,
      ellipsoid_half_axes_xyz, target_frame_p_offset, tolerance));
}

absl::StatusOr<eigenmath::VectorXd> EllipsoidConstraint::Evaluate(
    const eigenmath::VectorXd& joint_angles) {
  if (!joint_angles.allFinite()) {
    return absl::InternalError("Joint angles are not finite.");
  }

  JointStateP joint_positions;
  INTR_RETURN_IF_ERROR(joint_positions.SetSize(joint_angles.size()));
  joint_positions.position = joint_angles;
  INTR_RETURN_IF_ERROR(state_.SetDofPositions(joint_positions, false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      Pose3d root_t_frame, state_.GetTransform(root_frame_, target_frame_));
  Pose3d root_t_target =
      root_t_frame *
      Pose3d(eigenmath::Quaterniond::Identity(), target_frame_p_offset_);

  // Express pose in ellipsoid frame:
  Pose3d eps_t_target = goal_ellipsoid_t_root_ * root_t_target;

  // The ellipsoid inequality equation, centered around the origin, is:
  // x^2 / a^2 + y^2 / b^2 + z^2 / c^2 -1.0 <= 0.0
  // where a,b,c are the half axes. We efficiently write this as follows:
  eigenmath::Vector3d eps_p_target_scaled =
      eps_t_target.translation().cwiseProduct(
          ellipsoid_half_axes_xyz_inverted_);
  return eigenmath::VectorXd::Constant(kConstraintDim,
                                       eps_p_target_scaled.squaredNorm() - 1.0);
}

absl::StatusOr<eigenmath::MatrixXd> EllipsoidConstraint::Gradient(
    const eigenmath::VectorXd& joint_angles) {
  if (!joint_angles.allFinite()) {
    return absl::InternalError("Joint angles are not finite.");
  }
  // Numerically approximate the constraint gradient w.r.t. joint positions.
  return eigenmath::MatrixXd(ComputeDerivative(
      // This lambda is required because num-diff currently does
      // not handle StatusOr.
      [this](const eigenmath::VectorXd& joint_angles) {
        auto value_or = this->Evaluate(joint_angles);
        CHECK_OK(value_or);
        return *value_or;
      },
      joint_angles,
      /*double_sided_derivative=*/false));
}

absl::StatusOr<bool> EllipsoidConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_angles) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_angles));
  return (residual.array() <= tolerance_.array()).all();
}

PlaneConstraint::PlaneConstraint(
    const Chain* chain, const eigenmath::Vector3d& base_p_point_on_plane,
    const eigenmath::Vector3d& plane_normal_in_base_frame,
    ElementId target_frame_id, const eigenmath::Vector3d& target_frame_p_offset,
    double tolerance)
    : state_(ABSL_DIE_IF_NULL(chain)),
      base_p_point_on_plane_(base_p_point_on_plane),
      plane_normal_in_base_frame_(plane_normal_in_base_frame),
      target_frame_id_(target_frame_id),
      target_frame_p_offset_(target_frame_p_offset),
      tolerance_(eigenmath::VectorXd::Constant(kConstraintDim, tolerance)) {}

/*static*/
absl::StatusOr<std::unique_ptr<PlaneConstraint>> PlaneConstraint::Create(
    const Chain* chain, const eigenmath::Vector3d& base_p_point_on_plane,
    const eigenmath::Vector3d& plane_normal_in_base_frame,
    ElementId target_frame_id, const eigenmath::Vector3d& target_frame_p_offset,
    double tolerance) {
  if (!VectorIsNormalized(plane_normal_in_base_frame)) {
    return absl::FailedPreconditionError("Plane normal vector not normalized.");
  }

  if (tolerance < 0.0) {
    return absl::FailedPreconditionError(
        "Invalid tolerance, must not be negative.");
  }

  if (!ABSL_DIE_IF_NULL(chain)->GetElement(target_frame_id).ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ElementId ", static_cast<int>(target_frame_id),
                     " does not exist in chain ", chain->GetName()));
  }

  // Uses 'new' to access private constructor.
  return absl::WrapUnique(new PlaneConstraint(
      chain, base_p_point_on_plane, plane_normal_in_base_frame, target_frame_id,
      target_frame_p_offset, tolerance));
}

absl::StatusOr<eigenmath::VectorXd> PlaneConstraint::Evaluate(
    const eigenmath::VectorXd& joint_angles) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      Pose3d base_t_frame,
      EvaluateForwardKinematics(joint_angles, target_frame_id_, state_));
  Pose3d base_t_target = base_t_frame * Pose3d(target_frame_p_offset_);

  // The plane inequality constraint equation in normal form is given as
  // 'n.dot(r_0-r) <= 0.0' where 'n' is the plane normal, 'r_0' is a point on
  // the plane, and 'r' the point to be tested. Points fulfilling this equation
  // will lie on the side of the plane into which the normal vector points. We
  // implement this straight-forwardly here:
  double eval = plane_normal_in_base_frame_.dot(base_p_point_on_plane_ -
                                                base_t_target.translation());

  return eigenmath::VectorXd::Constant(kConstraintDim, eval);
}

absl::StatusOr<eigenmath::MatrixXd> PlaneConstraint::Gradient(
    const eigenmath::VectorXd& joint_angles) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::MatrixNMd jac,
      EvaluateJacobian(joint_angles, target_frame_id_, target_frame_p_offset_,
                       state_));

  // Above constraint has a straight-forward analytic gradient.
  return -plane_normal_in_base_frame_.transpose() * (jac.topRows(3));
}

absl::StatusOr<bool> PlaneConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_angles) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_angles));
  return (residual.array() <= tolerance_.array()).all();
}

}  // namespace intrinsic::kinematics
