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

#ifndef INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINTS_H_
#define INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINTS_H_

#include <memory>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/math.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/numopt/function_linearizer_numdiff.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::kinematics {

// Equality constraint enforcing that the position of a 'target' must match
// desired target location.
class PointConstraint : public ConstraintInterface {
 public:
  static constexpr int kConstraintDim = 3;
  static constexpr char kConstraintName[] = "PointConstraint";
  static constexpr double kDefaultTolerance = 1e-4;

  // Creates a 'PointConstraint'. 'base_p_target_desired' is the desired
  // position of the target point expressed in the base frame, 'chain'
  // represents the serial kinematic robot model. 'robot_frame_id' is the
  // ElementId of the frame which is to be placed at the target location, and
  // 'robot_frame_p_target' is an optional constant offset between the frame
  // specified by 'robot_frame_id' and the 'target' location, expressed in the
  // coordinates of the former. 'tolerance' corresponds to the maximum allowed
  // l1-norm position deviation. Returns 'kFailedPreconditionError' if the chain
  // pointer is null or the tolerance contains negative entries.
  static absl::StatusOr<std::unique_ptr<PointConstraint>> Create(
      const Chain* chain, const eigenmath::Vector3d& base_p_target_desired,
      kinematics::ElementId robot_frame_id,
      const eigenmath::Vector3d& robot_frame_p_target =
          eigenmath::Vector3d::Zero(),
      const eigenmath::Vector3d& tolerance =
          eigenmath::Vector3d::Constant(kConstraintDim, kDefaultTolerance));

  ConstraintType Type() const override { return GENERAL_EQUALITY; }

  int ConstraintDimension() const override { return kConstraintDim; }

  int DomainDimension() const override {
    return state_.GetKinematicModel()->GetNumberDegreesOfFreedom();
  }

  absl::string_view Name() const override { return kConstraintName; }

  // Evaluates residual of the PointConstraint at a given set of 'joint_angles',
  // returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<eigenmath::VectorXd> Evaluate(
      const eigenmath::VectorXd& joint_angles) override;

  // Returns analytical gradient of constraint residual, returns
  // 'kInternalError' in case of infinite joint angles or failed evaluation of
  // forward kinematics.
  absl::StatusOr<eigenmath::MatrixXd> Gradient(
      const eigenmath::VectorXd& joint_angles) override;

  const eigenmath::VectorXd& Tolerance() const override { return tolerance_; }

  // Returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<bool> IsSatisfied(
      const eigenmath::VectorXd& joint_angles) override;

 private:
  PointConstraint(const Chain* chain, const eigenmath::Vector3d& base_p_target,
                  kinematics::ElementId robot_frame_id,
                  const eigenmath::Vector3d& robot_frame_p_target,
                  const eigenmath::Vector3d& tolerance);

  const eigenmath::Vector3d base_p_target_;
  // Contains a pointer to the underlying Chain, which must outlive the State.
  State state_;
  const kinematics::ElementId robot_frame_id_;
  const eigenmath::Vector3d robot_frame_p_target_;
  const eigenmath::VectorXd tolerance_;
};

// Equality constraint achieving that pose of a target frame matches desired
// target pose.
class PoseConstraint : public ConstraintInterface {
 public:
  static constexpr int kConstraintDim = 6;
  static constexpr char kConstraintName[] = "PoseConstraint";
  static constexpr double kDefaultTolerance = 1e-4;

  // Creates a 'PoseConstraint' from a target pose 'base_t_target_desired' for a
  // given 'robot_frame_id' with optional offset 'robot_frame_t_target' between
  // the specified robot frame and the target location. 'robot_frame_t_target'
  // can be employed to solve inverse kinematics with arbitrary poses, which do
  // not have an own ElementId. Returns 'kFailedPreconditionError' if the chain
  // pointer is null or the tolerance contains negative entries.
  static absl::StatusOr<std::unique_ptr<PoseConstraint>> Create(
      const Chain* chain, const Pose3d& base_t_target_desired,
      kinematics::ElementId robot_frame_id,
      const Pose3d& robot_frame_t_target = Pose3d::Identity(),
      const eigenmath::Vector6d& tolerance =
          eigenmath::Vector6d::Constant(kConstraintDim, kDefaultTolerance));

  ConstraintType Type() const override { return GENERAL_EQUALITY; }

  int ConstraintDimension() const override { return kConstraintDim; }

  int DomainDimension() const override {
    return state_.GetKinematicModel()->GetNumberDegreesOfFreedom();
  }

  absl::string_view Name() const override { return kConstraintName; };

  // Evaluates residual of the PoseConstraint at a given set of 'joint_angles',
  // returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<eigenmath::VectorXd> Evaluate(
      const eigenmath::VectorXd& joint_angles) override;

  // Returns analytical gradient of constraint residual, returns
  // 'kInternalError' in case of infinite joint angles or failed evaluation of
  // forward kinematics.
  absl::StatusOr<eigenmath::MatrixXd> Gradient(
      const eigenmath::VectorXd& joint_angles) override;

  const eigenmath::VectorXd& Tolerance() const override { return tolerance_; }

  // Returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<bool> IsSatisfied(
      const eigenmath::VectorXd& joint_angles) override;

 private:
  PoseConstraint(const Chain* chain, const Pose3d& base_t_target_desired,
                 kinematics::ElementId robot_frame_id,
                 const Pose3d& robot_frame_t_target,
                 const eigenmath::Vector6d& tolerance);

  const Pose3d base_t_target_desired_;
  // Contains a pointer to the underlying Chain, which must outlive the State.
  State state_;
  const kinematics::ElementId robot_frame_id_;
  const Pose3d robot_frame_t_target_;
  const eigenmath::VectorXd tolerance_;
};

// Equality constraint which realizes a desired orientation of the target frame,
// but allows free rotation about a user-defined axis.
class OrientationWithFreeAxisConstraint : public ConstraintInterface {
 public:
  static constexpr int kConstraintDim = 1;
  static constexpr char kConstraintName[] = "OrientationWithFreeAxisConstraint";
  static constexpr double kDefaultToleranceRadians = 1e-3;

  // Creates an OrientationWithFreeAxisConstraint using the unit vector
  // 'free_axis_in_base_frame' as the targeted axis expressed in the robot base
  // frame. The constraint will align the unit vector
  // 'axis_to_align_in_target_frame' (defaults to the unit-z axis), which is
  // expressed w.r.t. the frame specified by 'robot_frame_id' and the relative
  // pose `robot_frame_t_target`, with the 'free_axis_in_base_frame'. Returns
  // kFailedPrecondition if either of the axes does not have unit length or if
  // the tolerance is negative. This constraint uses the relationship cos(alpha)
  // = a.dot(b), where alpha is the angle between two _normalized_ vectors 'a'
  // and 'b'. In order to be numerically better conditioned, the constraint
  // alpha = 0.0 can be reformulated as 'a.dot(b) = 1.0'.
  static absl::StatusOr<std::unique_ptr<OrientationWithFreeAxisConstraint>>
  Create(const Chain* chain, const eigenmath::Vector3d& free_axis_in_base_frame,
         kinematics::ElementId robot_frame_id,
         const eigenmath::Vector3d& axis_to_align_in_target_frame =
             eigenmath::Vector3d::UnitZ(),
         const Pose3d& robot_frame_t_target = Pose3d::Identity(),
         double tolerance_radians = kDefaultToleranceRadians);

  ConstraintType Type() const override { return GENERAL_EQUALITY; }

  int ConstraintDimension() const override { return kConstraintDim; }

  int DomainDimension() const override {
    return state_.GetKinematicModel()->GetNumberDegreesOfFreedom();
  }

  absl::string_view Name() const override { return kConstraintName; };

  // Evaluates residual of the constraint at a given set of 'joint_angles',
  // returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<eigenmath::VectorXd> Evaluate(
      const eigenmath::VectorXd& joint_angles) override;

  // Returns analytical gradient of constraint residual, returns
  // 'kInternalError' in case of infinite joint angles or failed evaluation of
  // forward kinematics.
  absl::StatusOr<eigenmath::MatrixXd> Gradient(
      const eigenmath::VectorXd& joint_angles) override;

  const eigenmath::VectorXd& Tolerance() const override { return tolerance_; }

  // Returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<bool> IsSatisfied(
      const eigenmath::VectorXd& joint_angles) override;

 private:
  OrientationWithFreeAxisConstraint(
      const Chain* chain, const eigenmath::Vector3d& free_axis_in_base_frame,
      kinematics::ElementId robot_frame_id,
      const eigenmath::Vector3d& axis_to_align_in_target_frame,
      const Pose3d& robot_frame_t_target, double tolerance_radians);

  // Contains a pointer to the underlying Chain, which must outlive the State.
  State state_;
  const eigenmath::Vector3d free_axis_in_base_frame_;
  const eigenmath::Vector3d axis_to_align_in_target_frame_;
  const ElementId robot_frame_id_;
  const Pose3d robot_frame_t_target_;
  const eigenmath::VectorXd tolerance_;
};

// Defines a Cone inequality constraint which aligns a user-definable axis of
// the target frame with the given cone defined by a cone axis and a cone
// opening angle.
class OrientationConeConstraint : public ConstraintInterface {
 public:
  static constexpr int kConstraintDim = 1;
  static constexpr char kConstraintName[] = "OrientationConeConstraint";
  static constexpr double kDefaultToleranceRadians = 1e-3;

  // Creates an OrientationConeConstraint using the axis
  // 'cone_axis_in_base_frame' of the targeted cone expressed in the robot base
  // frame with a cone opening angle defined by 'cone_half_angle_rad'. The
  // constraint will align 'axis_to_align_in_target_frame', which is
  // expressed w.r.t. the frame specified by 'robot_frame_id' and the relative
  // pose `robot_frame_t_target` with the cone. The maximum angle between
  // 'cone_axis_in_base_frame' and 'axis_to_align_in_target_frame' is restricted
  // to [0, cone_half_angle_rad]. Returns kFailedPrecondition in case of a
  // negative 'cone_half_angle_rad' or if the axes vectors are not normalized.
  static absl::StatusOr<std::unique_ptr<OrientationConeConstraint>> Create(
      const Chain* chain, const eigenmath::Vector3d& cone_axis_in_base_frame,
      double cone_half_angle_rad, ElementId robot_frame_id,
      const eigenmath::Vector3d& axis_to_align_in_target_frame =
          eigenmath::Vector3d::UnitZ(),
      const Pose3d& robot_frame_t_target = Pose3d::Identity(),
      double tolerance_rad = kDefaultToleranceRadians);

  ConstraintType Type() const override { return GENERAL_INEQUALITY; }

  int ConstraintDimension() const override { return kConstraintDim; }

  int DomainDimension() const override {
    return state_.GetKinematicModel()->GetNumberDegreesOfFreedom();
  }

  absl::string_view Name() const override { return kConstraintName; };

  // Evaluates residual of the constraint at a given set of 'joint_angles',
  // returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<eigenmath::VectorXd> Evaluate(
      const eigenmath::VectorXd& joint_angles) override;

  // Returns analytical gradient of constraint residual, returns
  // 'kInternalError' in case of infinite joint angles or failed evaluation of
  // forward kinematics.
  absl::StatusOr<eigenmath::MatrixXd> Gradient(
      const eigenmath::VectorXd& joint_angles) override;

  const eigenmath::VectorXd& Tolerance() const override { return tolerance_; }

  // Returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<bool> IsSatisfied(
      const eigenmath::VectorXd& joint_angles) override;

 private:
  OrientationConeConstraint(
      const Chain* chain, const eigenmath::Vector3d& cone_axis_in_base_frame,
      double cone_opening_half_angle, ElementId robot_frame_id,
      const eigenmath::Vector3d& axis_to_align_in_target_frame,
      const Pose3d& robot_frame_t_target, double tolerance_rad);

  const double cos_cone_opening_half_angle_;
  // Contains a pointer to the underlying Chain, which must outlive the State.
  State state_;
  const eigenmath::Vector3d cone_axis_in_base_frame_;
  const eigenmath::Vector3d axis_to_align_in_target_frame_;
  const ElementId robot_frame_id_;
  const Pose3d robot_frame_t_target_;
  const eigenmath::VectorXd tolerance_;
};

class EllipsoidConstraint : public ConstraintInterface {
 public:
  static constexpr int kConstraintDim = 1;
  static constexpr char kConstraintName[] = "EllipsoidConstraint";
  static constexpr double kDefaultTolerance = 1e-4;

  // Creates an EllipsoidConstraint which forces a point described by an offset
  // 'target_frame_p_offset' w.r.t. a 'target_frame' to lie inside an ellipsoid
  // with given 'ellipsoid_half_axes_xyz' defined by the pose
  // 'root_t_goal_ellipsoid' w.r.t. a 'root_frame'. Returns
  // kFailedPrecondition if provided ellipsoid half-axes are not positive or
  // tolerance is negative. For a use-case with zero half axes, employ the
  // 'PointConstraint' instead.
  static absl::StatusOr<std::unique_ptr<EllipsoidConstraint>> Create(
      const Chain* chain, ElementId root_frame, ElementId target_frame,
      const Pose3d& root_t_goal_ellipsoid,
      const eigenmath::Vector3d& ellipsoid_half_axes_xyz,
      const eigenmath::Vector3d& target_frame_p_offset =
          eigenmath::Vector3d::Zero(),
      double tolerance = kDefaultTolerance);

  ConstraintType Type() const override { return GENERAL_INEQUALITY; }

  int ConstraintDimension() const override { return kConstraintDim; }

  int DomainDimension() const override {
    return state_.GetKinematicModel()->GetNumberDegreesOfFreedom();
  }

  absl::string_view Name() const override { return kConstraintName; };

  // Evaluates residual of the constraint at a given set of 'joint_angles',
  // returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<eigenmath::VectorXd> Evaluate(
      const eigenmath::VectorXd& joint_angles) override;

  // Returns analytical gradient of constraint residual, returns
  // 'kInternalError' in case of infinite joint angles or failed evaluation of
  // forward kinematics.
  absl::StatusOr<eigenmath::MatrixXd> Gradient(
      const eigenmath::VectorXd& joint_angles) override;

  const eigenmath::VectorXd& Tolerance() const override { return tolerance_; }

  // Returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<bool> IsSatisfied(
      const eigenmath::VectorXd& joint_angles) override;

 private:
  EllipsoidConstraint(const Chain* chain, ElementId root_frame,
                      ElementId target_frame,
                      const Pose3d& root_t_goal_ellipsoid,
                      const eigenmath::Vector3d& ellipsoid_half_axes_xyz,
                      const eigenmath::Vector3d& target_frame_p_offset,
                      double tolerance);

  // Contains a pointer to the underlying Chain, which must outlive the State.
  State state_;
  const ElementId root_frame_;
  const ElementId target_frame_;
  const Pose3d goal_ellipsoid_t_root_;
  const eigenmath::Vector3d
      ellipsoid_half_axes_xyz_inverted_;  // = 1/ellipsoid_half_axes_xyz
  const eigenmath::Vector3d target_frame_p_offset_;
  const eigenmath::VectorXd tolerance_;
};

// Defines a Plane inequality constraint which forces a given frame to stay on
// one side of the plane.
class PlaneConstraint : public ConstraintInterface {
 public:
  static constexpr int kConstraintDim = 1;
  static constexpr char kConstraintName[] = "PlaneConstraint";
  static constexpr double kDefaultTolerance = 1e-4;

  // Creates an PlaneConstraint using the coordinates of a point on the plane
  // 'base_p_point_on_plane', the plane's normal expressed in the base frame
  // 'plane_normal_in_base_frame'. The constraint will enforce that the target
  // frame (plus optional 'target_frame_p_offset') lies on the side of the plain
  // into which the normal vector points. Returns 'kFailedPrecondition' if the
  // plane vector is not normalized or if the tolerance is negative.
  static absl::StatusOr<std::unique_ptr<PlaneConstraint>> Create(
      const Chain* chain, const eigenmath::Vector3d& base_p_point_on_plane,
      const eigenmath::Vector3d& plane_normal_in_base_frame,
      ElementId target_frame_id,
      const eigenmath::Vector3d& target_frame_p_offset =
          eigenmath::Vector3d::Zero(),
      double tolerance = kDefaultTolerance);

  ConstraintType Type() const override { return GENERAL_INEQUALITY; }

  int ConstraintDimension() const override { return kConstraintDim; }

  int DomainDimension() const override {
    return state_.GetKinematicModel()->GetNumberDegreesOfFreedom();
  }

  absl::string_view Name() const override { return kConstraintName; };

  // Evaluates residual of the constraint at a given set of 'joint_angles',
  // returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<eigenmath::VectorXd> Evaluate(
      const eigenmath::VectorXd& joint_angles) override;

  // Returns analytical gradient of constraint residual, returns
  // 'kInternalError' in case of infinite joint angles or failed evaluation of
  // forward kinematics.
  absl::StatusOr<eigenmath::MatrixXd> Gradient(
      const eigenmath::VectorXd& joint_angles) override;

  const eigenmath::VectorXd& Tolerance() const override { return tolerance_; }

  // Returns 'kInternalError' in case of infinite joint angles or failed
  // evaluation of forward kinematics.
  absl::StatusOr<bool> IsSatisfied(
      const eigenmath::VectorXd& joint_angles) override;

 private:
  PlaneConstraint(const Chain* chain,
                  const eigenmath::Vector3d& base_p_point_on_plane,
                  const eigenmath::Vector3d& plane_normal_in_base_frame,
                  ElementId target_frame_id,
                  const eigenmath::Vector3d& target_frame_p_offset,
                  double tolerance);

  // Contains a pointer to the underlying Chain, which must outlive the State.
  State state_;
  const eigenmath::Vector3d base_p_point_on_plane_;
  const eigenmath::Vector3d plane_normal_in_base_frame_;
  const ElementId target_frame_id_;
  const eigenmath::Vector3d target_frame_p_offset_;
  const eigenmath::VectorXd tolerance_;
};

}  // namespace intrinsic::kinematics

#endif  // INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINTS_H_
