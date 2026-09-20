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

#include "intrinsic/kinematics/ik_solvers/ur_ik_solver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/rotation_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {
namespace {

using eigenmath::Vector3d;
using eigenmath::VectorNd;

Vector3d ProjectToXYPlane(Vector3d v) {
  v.z() = 0;
  return v;
}

bool IsCloseTo(double value, double target, double eps) {
  return std::fabs(value - target) < eps;
}

bool IsCloseTo(const Vector3d& value, const Vector3d& target, double eps) {
  return (value - target).norm() < eps;
}

}  // namespace

absl::Status URIKSolver::Init(const Chain* model) {
  CHECK(model != nullptr);

  INTR_ASSIGN_OR_RETURN(const bool has_dependent_joints,
                        model->HasDependentJoints());
  if (has_dependent_joints) {
    return absl::InvalidArgumentError(
        "UR IK solver does not support chains with dependent "
        "joints.");
  }

  DVLOG(1) << "Initializing IK solver for chain name:" << model->GetName();
  if (!ValidateURKinematics(*model)) {
    model_ = nullptr;
    return absl::InvalidArgumentError(
        "The robot not compatible with the UR solver.");
  }

  model_ = model;

  for (int i = 0; i < model_->GetNumberDegreesOfFreedom(); i++) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto joint_id,
                                  model_->GetElementIdForDofIndex(i));
    INTRINSIC_RT_ASSIGN_OR_RETURN(joints_[i], model_->GetJoint(joint_id));
    DVLOG(1) << "Joint " << i << " origin: ("
             << toString(joints_[i]->GetParentTThis()) << ") axis: ("
             << toString(joints_[i]->GetAxis()) << ")";
  }

  // Extract the necessary canonical link lengths for computing kinematic
  // branch using the relative position of the following joints.
  l2_ = joints_[2]->GetParentTThis().translation().x();
  l3_ = joints_[3]->GetParentTThis().translation().x();
  l4_ = joints_[4]->GetParentTThis().translation().z();

  DVLOG(1) << "Canonical link lengths for branch labeling: " << l2_ << ", "
           << l3_ << ", " << l4_;

  return absl::OkStatus();
}

// Returns true if the passed model defines kinematics that can be
// solved by the solver. However, note that there are kinematic descriptions of
// UR arms that the class doesn't handle and will therefore be rejected by this
// function.
//
// This function is more strict than necessary to make parameter extraction
// easier (certain link lengths, and plane offsets for example).
// TODO(gaschler): Return RealtimeStatus with explanation instead and the remove
// duplicate checks from intrinsic/kinematics/ik/ur_inverse_kinematics.cc.
bool URIKSolver::ValidateURKinematics(const Chain& model) {
  if (model.GetNumberDegreesOfFreedom() != kNbDof) {
    INTRINSIC_RT_LOG(ERROR)
        << "Invalid joint count: " << model.GetNumberDegreesOfFreedom();
    return false;
  }

  INTRINSIC_RT_ASSIGN_OR_DIE(auto dof_chain,
                             model.GetDofChainForTip(model.GetTipId()));

  // Run FK and get the position of each joint at the zero position.
  //
  // All joints must be revolute. We check this here as well.
  std::array<Pose3d, kNbDof> base_t_joint_poses;
  for (int i = 0; i < kNbDof; i++) {
    INTRINSIC_RT_ASSIGN_OR_DIE(const auto* joint_i,
                               model.GetJoint(dof_chain[i]));
    if (joint_i->GetParameters().type != Joint::Type::REVOLUTE) {
      INTRINSIC_RT_LOG(ERROR) << "Joint " << i << " must be a revolve joint.";
      return false;
    }
    if (i == 0) {
      base_t_joint_poses.at(i) = joint_i->GetParentTThis();
    } else {
      base_t_joint_poses.at(i) =
          base_t_joint_poses.at(i - 1) * joint_i->GetParentTThis();
    }
  }

  constexpr double kMaxRotationError = 1e-8;
  constexpr double kMaxVectorError = 1e-8;

  // Expect that the first frame has only a z rotation and its axis is positive
  // z.
  eigenmath::Vector3d base_t_joint0_axis_angle =
      eigenmath::QuaternionToAngleTimesAxis(
          base_t_joint_poses.at(0).quaternion());
  if (std::abs(base_t_joint0_axis_angle.x()) > kMaxRotationError ||
      std::abs(base_t_joint0_axis_angle.y()) > kMaxRotationError) {
    INTRINSIC_RT_LOG(ERROR)
        << "Joint 0 must only have a z rotation in base frame.";
    return false;
  }
  INTRINSIC_RT_ASSIGN_OR_DIE(const auto* joint_0, model.GetJoint(dof_chain[0]));
  if (!IsCloseTo(joint_0->GetAxis(), Vector3d(0, 0, 1), kMaxVectorError)) {
    INTRINSIC_RT_LOG(ERROR)
        << "Joint 0 axis must be (0, 0, 1) in joint 0 frame.";
    return false;
  }

  // Joints 1, 2, 3 just lie on the x-z plane (in joint 0 frame), and must have
  // their axis be in direction (0, 1, 0).
  for (int i = 1; i < 4; i++) {
    if ((base_t_joint_poses.at(0).quaternion().inverse() *
         base_t_joint_poses.at(i).translation())
            .dot(Vector3d(0, 1, 0)) > kMaxVectorError) {
      INTRINSIC_RT_LOG(ERROR)
          << "Joint " << i
          << " expected to lie on plane x-z plane in joint 0 "
             "frame but did not.";
      return false;
    }
    INTRINSIC_RT_ASSIGN_OR_DIE(const auto* joint_i,
                               model.GetJoint(dof_chain[i]));
    if (!IsCloseTo(base_t_joint_poses.at(0).quaternion().inverse() *
                       base_t_joint_poses.at(i).quaternion() *
                       joint_i->GetAxis(),
                   Vector3d(0, 1, 0), kMaxVectorError)) {
      INTRINSIC_RT_LOG(ERROR)
          << "Joint " << i << " axis must be (0, 1, 0) in joint 0 frame";
      return false;
    }
  }

  // Joint 4 must have a positive offset in the y direction in joint 0 frame
  // (which influences a hard-coded offset when using computing j0). Its axis
  // should be +- z in joint 0 frame.
  if ((base_t_joint_poses.at(0).rotationMatrix().transpose() *
       base_t_joint_poses.at(4).translation())
          .dot(Vector3d(0, 1, 0)) <= 0) {
    INTRINSIC_RT_LOG(ERROR)
        << "Joint 4 expected to be offset in the positive y "
           "direction in joint 0 frame.";
    return false;
  }
  INTRINSIC_RT_ASSIGN_OR_DIE(const auto* joint_4, model.GetJoint(dof_chain[4]));
  Vector3d joint_4_axis_in_joint_0_frame =
      base_t_joint_poses.at(0).quaternion().inverse() *
      base_t_joint_poses.at(4).quaternion() * joint_4->GetAxis();
  if (!IsCloseTo(joint_4_axis_in_joint_0_frame, Vector3d(0, 0, -1),
                 kMaxVectorError) &&
      !IsCloseTo(joint_4_axis_in_joint_0_frame, Vector3d(0, 0, 1),
                 kMaxVectorError)) {
    INTRINSIC_RT_LOG(ERROR)
        << "Joint 4 axis must be (0, 0, +-1) in joint 0 frame";
    return false;
  }

  INTRINSIC_RT_ASSIGN_OR_DIE(const auto* joint_3, model.GetJoint(dof_chain[3]));
  INTRINSIC_RT_ASSIGN_OR_DIE(const auto* joint_5, model.GetJoint(dof_chain[5]));

  // Joint 5 must have the same axis direction as joint 3.
  if (!IsCloseTo(base_t_joint_poses.at(3).quaternion() * joint_3->GetAxis(),
                 base_t_joint_poses.at(5).quaternion() * joint_5->GetAxis(),
                 kMaxVectorError)) {
    INTRINSIC_RT_LOG(ERROR) << "Joint 5 axis must be the same as joint 3 axis";
    return false;
  }

  return true;
}

int URIKSolver::Solve(
    const Pose3d& desired_base_t_tip, const JointStateP& prev_joint_state,
    std::array<VectorNd, kSolutionBufferSize>* solutions) const {
  std::pair<int, std::array<VectorNd, kSolutionBufferSize>>
      number_and_solutions = {0, {}};

  CHECK(model_ != nullptr);

  // The kinematics of a ur arm result in a specific analytical ik. The
  // properties we use are:
  //   * The axes of joints 1, 2, and 3 are always parallel and thus can be
  //     considered to be on the same plane (even though physically the motors
  //     are shifted along the axes).
  //   * The point at the intersection of axes 4 and 5 has a fixed offset to the
  //     plane of joints 1, 2, and 3. Thus, from the knowledge of where that
  //     point is, we can figure out what values for joint 0 orient the plane
  //     to create this offset.
  //   * The axis of joint 4 is always perpendicular to the axes joints 3 and 5.

  DVLOG(1) << "ComputePositionIK: pose: " << toString(desired_base_t_tip);

  const eigenmath::AngleAxisd base_t_joint_0_frame(
      joints_[0]->GetParentTThis().quaternion());
  double base_joint_0_frame_z_rotation =
      base_t_joint_0_frame.angle() * base_t_joint_0_frame.axis().z();
  DVLOG(1) << "base_joint_0_frame_z_rotation: "
           << base_joint_0_frame_z_rotation;
  const Pose3d desired_joint_0_frame_t_tip =
      intrinsic::Pose3d(
          eigenmath::Quaterniond(base_t_joint_0_frame).inverse()) *
      desired_base_t_tip;

  // Unpack joints for convenience.
  const auto& joint0 = *joints_[0];
  const auto& joint1 = *joints_[1];
  const auto& joint2 = *joints_[2];
  const auto& joint3 = *joints_[3];
  const auto& joint4 = *joints_[4];
  const auto& joint5 = *joints_[5];

  // Compute a number of constant values first which will be used throughout.

  const Vector3d base_t_j4_position =
      (desired_joint_0_frame_t_tip * joint5.GetParentTThis().inverse())
          .translation();
  DVLOG(1) << "base_t_j4_position: (" << toString(base_t_j4_position) << ")";

  const double j4_distance_to_plane =
      (joint4.GetParentTThis().translation() -
       joint4.GetAxis() *
           joint4.GetAxis().dot(joint4.GetParentTThis().translation()))
          .norm();
  DVLOG(1) << "j4 offset from plane: " << j4_distance_to_plane;

  const double link_1_offset = joint2.GetParentTThis().translation().norm();
  const double link_2_offset = joint3.GetParentTThis().translation().norm();
  DVLOG(1) << "  link_1_offset: " << link_1_offset;
  DVLOG(1) << "  link_2_offset: " << link_2_offset;

  const Vector3d joint_5_axis_in_base =
      desired_joint_0_frame_t_tip.quaternion() * joint5.GetAxis();
  DVLOG(1) << "joint_5_axis_in_base: (" << toString(joint_5_axis_in_base)
           << ")";

  const Vector3d joint_4_axis_in_frame_5_for_j5_equals_0 =
      joint5.GetParentTThis().quaternion().inverse() * joint4.GetAxis();
  const Vector3d joint_4_axis_direction_for_desired_frame_in_base =
      desired_joint_0_frame_t_tip.quaternion() *
      joint_4_axis_in_frame_5_for_j5_equals_0;
  DVLOG(1) << "joint_4_axis_in_frame_6_for_j5_equals_0: ("
           << toString(joint_4_axis_in_frame_5_for_j5_equals_0) << ")";
  DVLOG(1) << "joint_4_axis_direction_for_desired_frame_in_base: ("
           << toString(joint_4_axis_direction_for_desired_frame_in_base) << ")";

  // TODO(keegang): Add validation to ensure axes 1, 2, 3 are aligned.
  // TODO(keegang): Add validation to check that 3 and 5 are aligned in the zero
  // position.

  // The computation starts here. We begin by computing j0. This works since
  // joints 0, 1, 2, and 3 are on a plane (vertical through the origin), and
  // joint 4 have a fixed offset to the plane.
  //
  // These is an assumption here about what plane the first joint operates in.
  const Vector3d base_t_joint4_projected_to_plane =
      ProjectToXYPlane(base_t_j4_position);
  const double cos_distance_to_plane_over_projected =
      j4_distance_to_plane / base_t_joint4_projected_to_plane.norm();
  // If this condition is true, it implies that joint 4 is within a cylinder of
  // radius j4_distance_to_plane around joint 0. However, given the kinematics
  // of the arm, there is no possible solution for this case since the smallest
  // feasible distance between joint 4 and joint 0 is exactly
  // j4_distance_to_plane. To see this intuitively, note that moving joints 0,
  // 1, 2, 3 do not change this distance to the plane for joint 4, only its
  // position in the plane.
  if (std::fabs(cos_distance_to_plane_over_projected) > 1) {
    DVLOG(1)
        << "Too close to center axis. cos_distance_to_plane_over_projected: "
        << cos_distance_to_plane_over_projected;
    *solutions = number_and_solutions.second;
    return number_and_solutions.first;
  }

  // TODO(keegang): The -pi/2 offset comes from the structure of a particular
  // sdf. This could be generalized.
  const double j0 = atan2(base_t_joint4_projected_to_plane(1),
                          base_t_joint4_projected_to_plane(0)) +
                    acos(cos_distance_to_plane_over_projected) - M_PI / 2.0;
  const double j0a = atan2(base_t_joint4_projected_to_plane(1),
                           base_t_joint4_projected_to_plane(0)) -
                     acos(cos_distance_to_plane_over_projected) - M_PI / 2.0;
  const std::array<double, 2> possible_j0_values = {j0, j0a};

  for (int i = 0; i < possible_j0_values.size(); i++) {
    const double j0 = possible_j0_values.at(i);
    DVLOG(1) << "  j0: " << j0;

    // Knowing j0 fixes the direction of the axes of joints 1, 2, and in
    // particular 3.
    //
    // From the kinematics of the robot, it can be concluded that the axis
    // of joint 4 is always perpendicular to the axes of joints 3 and 5. Given
    // that in the zero configuration the axes of joints 3 and joints 5 are
    // aligned, the value of joint 4 is precisely the angle between these two
    // axes (up to a sign since the axis of joint 4 could be flipped in either
    // direction.

    // The algorithm assumes that joint0 frame has no rotation.
    // However, in UR's recommended base frame, it has a 180 deg z rotation.
    // So, we run the algorithm in joint0's frame rather than the base frame.
    const intrinsic::Pose3d joint0_transform =
        intrinsic::Pose3d(
            eigenmath::Quaterniond(base_t_joint_0_frame).inverse()) *
        joint0.GetJointOutboundTransform(j0);
    const Vector3d joint_1_2_3_axis_in_base =
        joint0_transform.quaternion() * joint1.GetParentTThis().quaternion() *
        joint1.GetAxis();
    const double j4 = acos(std::clamp(
        joint_1_2_3_axis_in_base.dot(joint_5_axis_in_base), -1.0, 1.0));
    DVLOG(1) << "joint_1_2_3_axis_in_base: ("
             << toString(joint_1_2_3_axis_in_base) << ")";
    DVLOG(1) << "joint_5_axis_in_base: " << toString(joint_5_axis_in_base);
    const std::array<double, 2> possible_j4_values = {j4, -j4};

    for (int j = 0; j < possible_j4_values.size(); j++) {
      const double j4 = possible_j4_values.at(j);
      DVLOG(1) << "    j4: " << j4;

      // Compute j5.
      //
      // If j4 is close to zero or pi, we have a singularity where the axis
      // joint 5 is aligned with the axes of joints 1, 2, an 3. This means that
      // the system is underdetermined, so we take a desired value for joint 4
      // from the previous joint state.
      //
      // Note that this simplistic approach may cause issues when operating at
      // the boundaries of the workspace, since fixing the value of j5 might
      // reduce the reach of the robot. The alternative would be doing some
      // sampling around different values of joint 5, but we'll choose to forgo
      // that for now since it increases the complexity of this code and
      // its runtime.
      double j5 = 0;  // Essentially const, but set in one of the two branches.
      Vector3d joint_4_axis_in_base;
      if (IsCloseTo(j4, 0, 1e-5) || IsCloseTo(std::fabs(j4), M_PI, 1e-5)) {
        DVLOG(1) << "    using provided value for j5";
        j5 = prev_joint_state.position(5);

        // Compute the rotation component of j4 axis in the tip frame
        // [tip_r_j4_axis] by "undoing" (1) the rotation in j5, and
        // (2) the relative rotation between j4 and j5 since some robot
        // descriptions have non-identity rotation offsets between j4 and j5.
        eigenmath::Quaterniond j4_axis_r_j5_axis =
            joint5.GetParentTThis().quaternion();
        eigenmath::Quaterniond tip_r_j4_axis =
            eigenmath::Quaterniond(
                Eigen::AngleAxis<double>(-j5, joint5.GetAxis())) *
            j4_axis_r_j5_axis.inverse();

        // Compute the rotation component of j4 axis w.r.t. the base frame for
        // the desired tip pose.
        eigenmath::Quaterniond base_r_j4_axis =
            desired_joint_0_frame_t_tip.quaternion() * tip_r_j4_axis;

        // Compute j4 axis in base frame.
        joint_4_axis_in_base = (base_r_j4_axis * joint4.GetAxis()).normalized();
      } else {
        // Given the kinematics of the robot, and knowledge of the
        // directions of the axes of joints 4 and 5, we can compute where the
        // axis of joint 4 would be if j5 = 0. We can then compare this
        // direction against the direction if would be in the desired ik target
        // frame and find the rotation from this.
        // The assumption is that that the axes vectors are normalized. Need to
        // clamp their dot product between -1.0 and 1.0 to prevent numeric
        // instability in acos.
        joint_4_axis_in_base =
            joint_1_2_3_axis_in_base.cross(joint_5_axis_in_base).normalized() *
            (j4 < 0 ? -1.0 : 1.0);

        auto dot_product_clamped =
            std::clamp(joint_4_axis_in_base.dot(
                           joint_4_axis_direction_for_desired_frame_in_base),
                       -1.0, 1.0);
        j5 = acos(dot_product_clamped);

        // Resolve sign ambiguity of acos using the direction of the axis of
        // joint 5.
        const double projection =
            joint_4_axis_direction_for_desired_frame_in_base.dot(
                joint_5_axis_in_base.cross(joint_4_axis_in_base));
        if (projection < 0.0) {
          j5 *= -1;
        }
      }
      DVLOG(1) << "    j5: " << j5;
      DCHECK_LT(fabs(joint_4_axis_in_base.norm() - 1), 1e-6)
          << "joint_4_axis_in_base=" << toString(joint_4_axis_in_base);
      DVLOG(1) << "    joint_4_axis_in_base: ("
               << toString(joint_4_axis_in_base) << ")";

      // At this point we have j0, j4, and j5. Given that joints 1, 2, and 3 are
      // in a plane and aligned, we now essentially solve a RRR manipulator
      // problem to determine the remaining angles.
      //
      // We start by finding joints 1 and 2.
      //
      // We need to find the target location for joint 3. Since we have the
      // values for joints 4 and 5 already known, we can just work backwards.
      // Note the orientation of the pose is not correct yet, but we only use
      // the position.
      const Vector3d base_t_joint3_position =
          (desired_joint_0_frame_t_tip *
           joint5.GetJointOutboundTransform(j5).inverse() *
           joint4.GetJointOutboundTransform(j4).inverse())
              .translation();
      DVLOG(1) << "    joint3_position: (" << toString(base_t_joint3_position)
               << ")";
      const Vector3d base_t_joint1_position =
          (joint0_transform * joint1.GetParentTThis()).translation();
      DVLOG(1) << "    joint1_position: (" << toString(base_t_joint1_position)
               << ")";
      const Vector3d joint1_to_joint3_in_base =
          base_t_joint3_position - base_t_joint1_position;
      DVLOG(1) << "    joint1_to_joint3_in_base: ("
               << toString(joint1_to_joint3_in_base) << ")";
      DVLOG(1) << "    joint1_to_joint3_in_base norm: "
               << joint1_to_joint3_in_base.norm();
      // Assumes that j1 is on the same plane.
      //
      // This equation comes from slide 17 from here:
      // http://www.diag.uniroma1.it/~deluca/rob1_en/10_InverseKinematics.pdf
      // Note the reduction uses the difference of cosines.
      double cosine_j2 =
          ((joint1_to_joint3_in_base).squaredNorm() -
           link_1_offset * link_1_offset - link_2_offset * link_2_offset) /
          2.0 / link_1_offset / link_2_offset;
      // When the arm is fully extended, cosine_j2 could come in just above 1 or
      // below -1. Since these types of positions could be used in particular,
      // we add some acceptable epsilon so avoid a failure in the acos.
      constexpr double kJ2Epsilon = 1.000001;
      if (cosine_j2 < kJ2Epsilon && cosine_j2 > 1) {
        cosine_j2 = 1;
      }
      if (cosine_j2 > -kJ2Epsilon && cosine_j2 < -1) {
        cosine_j2 = -1;
      }
      DVLOG(1) << "    cosine_j2: " << cosine_j2;
      if (cosine_j2 > 1 || cosine_j2 < -1) {
        DVLOG(1) << "    No solutions, cosine too large.";
        DVLOG(1) << "    -----------------------------------";
        continue;
      }
      const double sine_j2 = sqrt(1 - cosine_j2 * cosine_j2);
      const double j2 = atan2(sine_j2, cosine_j2);

      // Two solutions are possible, but sometime we get one (when j2 = 0 or j2
      // = pi). For simplicity for now, we allow duplicate solutions since the
      // user can always sort/filter the results.
      const std::array<double, 2> possible_j2_values = {j2, -j2};

      for (int k = 0; k < possible_j2_values.size(); k++) {
        const double j2 = possible_j2_values.at(k);
        DVLOG(1) << "    j2: " << j2;

        // Decompose the offset into the vertical and horizontal components.
        const double joint1_to_joint3_vertical =
            joint1_to_joint3_in_base.dot(joint0.GetAxis());
        const double joint1_to_joint3_horizontal = joint1_to_joint3_in_base.dot(
            joint0_transform.quaternion() *
            joint1.GetParentTThis().quaternion() *
            joint2.GetParentTThis().quaternion() *
            joint2.GetParentTThis().translation().normalized());
        DVLOG(1) << "      joint1_to_joint3_vertical: "
                 << joint1_to_joint3_vertical;
        DVLOG(1) << "      joint1_to_joint3_horizontal: "
                 << joint1_to_joint3_horizontal;
        const double j1 =
            -1 * atan2(joint1_to_joint3_vertical, joint1_to_joint3_horizontal) -
            atan2(link_2_offset * sin(j2),
                  link_1_offset + link_2_offset * cos(j2));
        DVLOG(1) << "      j1: " << j1;

        // Finally we have to find j3. We run fk up to that point, then look at
        // where the axis of joint 4 is to figure out the rotation.
        const Pose3d j3_pose_with_j3_equals_zero_in_base =
            joint0_transform * joint1.GetJointOutboundTransform(j1) *
            joint2.GetJointOutboundTransform(j2) * joint3.GetParentTThis();
        const Vector3d joint_4_axis_with_j3_equals_zero_in_base =
            j3_pose_with_j3_equals_zero_in_base.quaternion() *
            joint4.GetParentTThis().quaternion() * joint4.GetAxis();
        double j3 = acos(std::clamp(
            joint_4_axis_with_j3_equals_zero_in_base.dot(joint_4_axis_in_base),
            -1.0, 1.0));
        // Fix the sign with the direction of the axis of joint 3.
        if (joint_4_axis_in_base.dot(joint_1_2_3_axis_in_base.cross(
                joint_4_axis_with_j3_equals_zero_in_base)) < 0) {
          j3 *= -1;
        }

        auto& solution =
            number_and_solutions.second.at(number_and_solutions.first);
        solution.resize(model_->GetNumberDegreesOfFreedom());
        solution << j0, j1, j2, j3, j4, j5;
        ++number_and_solutions.first;

        DVLOG(1) << "      j3: " << j3;
        DVLOG(1) << "      ---------------------------------";
        DVLOG(1) << number_and_solutions.first << "th IK solution"
                 << toString(solution);
      }
    }
  }

  *solutions = number_and_solutions.second;
  return number_and_solutions.first;
}

size_t URIKSolver::GetBranch(const JointStateP& joint_state) const {
  size_t branch = 0;

  // Unpack joint states required for branch labeling. j_i and joint_state are
  // 0-indexed, where:
  // j1 -> "shoulder_lift_joint"
  // j2 -> "elbow_joint"
  // j3 -> "wrist_1_joint"
  // j4 -> "wrist_2_joint"
  // as defined in the UR robot descriptions.
  const double j1 = joint_state.position(1);
  const double j2 = joint_state.position(2);
  const double j3 = joint_state.position(3);
  const double j4 = joint_state.position(4);

  // Determine the waist branch
  const double waist_determinant =
      l2_ * cos(j1) + l3_ * cos(j1 + j2) + l4_ * sin(j1 + j2 + j3);
  if (waist_determinant >= 0.0) branch |= 1 << 0;

  // Determine the elbow branch
  const double elbow_determinant = copysign(1.0, waist_determinant) * sin(j2);
  // Consider elbow is "up" when theta3 ~ 0.0, i.e., arm is stretched out.
  if (fabs(elbow_determinant) <= 1e-8 || elbow_determinant >= 0.0)
    branch |= 1 << 1;

  // Determine the wrist branch
  const double wrist_determinant = -copysign(1.0, waist_determinant) * sin(j4);
  if (wrist_determinant >= 0.0) branch |= 1 << 2;

  return branch;
}

}  // namespace kinematics
}  // namespace intrinsic
