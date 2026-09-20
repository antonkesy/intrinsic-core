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

#include "intrinsic/kinematics/ik_solvers/spherical_wrist_ik_solver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/so3.h"
#include "intrinsic/eigenmath/swing_twist.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/joint_wrapping.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/analytic_geometry/constructions.h"
#include "intrinsic/motion_planning/path_planning/analytic_geometry/intersections.h"
#include "intrinsic/motion_planning/path_planning/analytic_geometry/types.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {

using eigenmath::Vector3d;
using eigenmath::VectorNd;

namespace {

// TODO(b/187413466): Review tolerance
constexpr double kEpsilon = 2e-7;

double RotationAroundAxis(const eigenmath::SO3d& rotation,
                          const eigenmath::Vector3d& axis) {
  auto [swing, twist] = eigenmath::SwingTwistDecomposition(rotation, axis);
  Eigen::AngleAxisd twist_aa(twist.quaternion());
  return twist_aa.angle();
}

bool IsWithinLimits(const eigenmath::VectorNd& q,
                    const eigenmath::VectorNd& lower_limits,
                    const eigenmath::VectorNd& upper_limits) {
  for (size_t i = 0; i < q.size(); ++i) {
    if (q[i] < lower_limits[i] || q[i] > upper_limits[i]) {
      return false;
    }
  }
  return true;
}

// Wrap the joint value within the joint limits. If it falls outside the joint
// limits, it sets the error flag and return the passed value.
double WrapJointAndSetFlag(double q, double upper_limit, double lower_limit,
                           SphericalWristIKResultFlags* flag) {
  double wrapped_q = WrapJoint(q, upper_limit, lower_limit);
  if (!isfinite(wrapped_q)) {
    // We are outside the joint limits.
    *flag = SphericalWristIKResultFlags(*flag | RESULT_JOINT_LIMIT_EXCEEDED);
    return q;
  }
  return wrapped_q;
}

}  // namespace

std::string ToString(SphericalWristIKResultFlags flags) {
  std::vector<std::string> flags_str;
  if (flags == RESULT_BLANK) {
    flags_str.emplace_back("RESULT_BLANK");
  }
  if (flags & RESULT_OK) {
    flags_str.emplace_back("RESULT_OK");
  }
  if (flags & RESULT_NONE) {
    flags_str.emplace_back("RESULT_NONE");
  }
  if (flags & RESULT_INPUT_INVALID) {
    flags_str.emplace_back("RESULT_INPUT_INVALID");
  }
  if (flags & RESULT_POS_Q0_INDETERMINITE) {
    flags_str.emplace_back("RESULT_POS_Q0_INDETERMINITE");
  }
  if (flags & RESULT_POS_Q35_INDETERMINATE) {
    flags_str.emplace_back("RESULT_POS_Q35_INDETERMINATE");
  }
  if (flags & RESULT_POS_UNREACHABLE) {
    flags_str.emplace_back("RESULT_POS_UNREACHABLE");
  }
  if (flags & RESULT_VEL_Q0_SINGULARITY) {
    flags_str.emplace_back("RESULT_VEL_Q0_SINGULARITY");
  }
  if (flags & RESULT_VEL_Q0_INFEASIBLE) {
    flags_str.emplace_back("RESULT_VEL_Q0_INFEASIBLE");
  }
  if (flags & RESULT_VEL_STRETCH_SINGULARITY) {
    flags_str.emplace_back("RESULT_VEL_STRETCH_SINGULARITY");
  }
  if (flags & RESULT_VEL_STRETCH_INFEASIBLE) {
    flags_str.emplace_back("RESULT_VEL_STRETCH_INFEASIBLE");
  }
  if (flags & RESULT_VEL_WRIST_SINGULARITY) {
    flags_str.emplace_back("RESULT_VEL_WRIST_SINGULARITY");
  }
  if (flags & RESULT_VEL_WRIST_INFEASIBLE) {
    flags_str.emplace_back("RESULT_VEL_WRIST_INFEASIBLE");
  }
  if (flags & RESULT_ACC_STRETCH_SINGULARITY) {
    flags_str.emplace_back("RESULT_ACC_STRETCH_SINGULARITY");
  }
  if (flags & RESULT_ACC_STRETCH_INFEASIBLE) {
    flags_str.emplace_back("RESULT_ACC_STRETCH_INFEASIBLE");
  }
  if (flags & RESULT_ACC_WRIST_SINGULARITY) {
    flags_str.emplace_back("RESULT_ACC_WRIST_SINGULARITY");
  }
  if (flags & RESULT_ACC_WRIST_INFEASIBLE) {
    flags_str.emplace_back("RESULT_ACC_WRIST_INFEASIBLE");
  }
  if (flags & RESULT_JOINT_LIMIT_EXCEEDED) {
    flags_str.emplace_back("RESULT_JOINT_LIMIT_EXCEEDED");
  }
  return absl::StrJoin(flags_str, ",");
}

absl::StatusOr<SphericalWristIKParameters> ExtractParameters(
    const ModelInterface& model) {
  const int kNbJoints = 6;

  if (!model.HasOneTip()) {
    return absl::InvalidArgumentError("The model should be a chain.");
  }

  if (model.GetNumberDegreesOfFreedom() != kNbJoints) {
    return absl::InvalidArgumentError("The robot should have 6-dof");
  }

  SphericalWristIKParameters parameters;
  parameters.joint_offsets.resize(model.GetNumberDegreesOfFreedom());
  parameters.joint_axis_directions_diff.resize(
      model.GetNumberDegreesOfFreedom());

  for (const auto& joint_id : model.GetAllJointIds()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint, model.GetJoint(joint_id));
    DVLOG(2) << "Joint " << joint->GetName()
             << " origin: " << toString(joint->GetParentTThis())
             << " axis: " << toString(joint->GetAxis());
  }

  std::array<Pose3d, kNbJoints> base_t_joint;
  std::array<Vector3d, kNbJoints> axis_in_base_frame;

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto dof_chain, model.GetDofChainForTip(model.GetTipIds().front()));

  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint_0,
                                model.GetJoint(dof_chain[0]));
  base_t_joint[0] = joint_0->GetParentTThis();
  axis_in_base_frame[0] = base_t_joint[0].rotationMatrix() * joint_0->GetAxis();

  // The dot product between `axis_in_base_frame[0]` and the unit-z axis is
  // either -1 or 1, hence we take the absolute value below.
  if (fabs(fabs(axis_in_base_frame[0].dot(Vector3d::UnitZ())) - 1.0) >
      kEpsilon) {
    return absl::InvalidArgumentError(
        "The axis of the first joint must be aligned with the Z axis.");
  }

  for (int i = 1; i < dof_chain.size(); ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint_i,
                                  model.GetJoint(dof_chain[i]));

    base_t_joint[i] = base_t_joint[i - 1] * joint_i->GetParentTThis();

    axis_in_base_frame[i] =
        (base_t_joint[i].rotationMatrix() * joint_i->GetAxis()).normalized();
  }

  for (size_t i = 0; i < kNbJoints; ++i) {
    DVLOG(2) << "Joint " << i << " base_t_joint: " << toString(base_t_joint[i]);
    DVLOG(2) << "Joint " << i
             << " axis_in_base_frame: " << toString(axis_in_base_frame[i]);
  }

  // Extract the canonical position of all joints.
  // TODO(b/344664395): Can we have a more formal definition of "canonical joint
  // position"?
  std::array<Vector3d, kNbJoints> base_p_joint_canonical;

  // For i = 0, ..., (kNbJoints - 1):
  // joint_canonical_i_p_joint_canonical_i_plus_one[i] =
  // (base_p_joint_canonical[i+1] - base_p_joint_canonical[i]);
  std::array<Vector3d, kNbJoints - 1>
      joint_canonical_i_p_joint_canonical_i_plus_one;

  // Joint 1 should be at the intersection of the plane normal to A1
  // (`axis_in_base_frame[0]`) and going through A2 (`axis_in_base_frame[1]`).
  INTR_ASSIGN_OR_RETURN(
      const analytical_geometry::Plane3d plane_with_normal_to_a1,
      analytical_geometry::Plane3d::Create(
          /*origin=*/base_t_joint[1].translation(),
          /*normal=*/axis_in_base_frame[0]));
  INTR_ASSIGN_OR_RETURN(
      const analytical_geometry::Line3d line_passing_through_a2,
      analytical_geometry::Line3d::Create(
          /*origin=*/base_t_joint[0].translation(),
          /*direction=*/axis_in_base_frame[0]));
  INTR_ASSIGN_OR_RETURN(base_p_joint_canonical[0],
                        analytical_geometry::PlaneLineIntersection(
                            plane_with_normal_to_a1, line_passing_through_a2));
  parameters.basel1z = base_p_joint_canonical[0][2];

  // The robot is defined in the third or fourth quadrant of the y-z plane
  // formed by the y and z axes of the joint 1's coordinate frame. We need to
  // bring it up to the first or second quadrant as the solver expect a positive
  // distance along the z-axis between A1 (`axis_in_base_frame[0]`) and A2
  // (`axis_in_base_frame[1]`).
  if (parameters.basel1z < 0) {
    Pose3d flip_around_x(eigenmath::Quaterniond(
        eigenmath::AngleAxisd(M_PI, eigenmath::Vector3d::UnitX())));
    parameters.base_offset = flip_around_x;
    parameters.basel1z = std::fabs(parameters.basel1z);

    base_p_joint_canonical[0] =
        parameters.base_offset * base_p_joint_canonical[0];
    for (size_t i = 0; i < kNbJoints; ++i) {
      base_t_joint[i] = parameters.base_offset * base_t_joint[i];
      axis_in_base_frame[i] = parameters.base_offset * axis_in_base_frame[i];
    }
  }

  if (std::fabs(base_p_joint_canonical[0](0)) > kEpsilon ||
      std::fabs(base_p_joint_canonical[0](1)) > kEpsilon) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Both the x and y-components of `base_p_joint_canonical[0]` must be 0, "
        "but instead have values ",
        base_p_joint_canonical[0](0), ", ", base_p_joint_canonical[0](1),
        ", respectively."));
  }

  // Joint 2 should be on the plane containing A1 (`axis_in_base_frame[0]`) and
  // normal to A2 (`axis_in_base_frame[1]`).
  INTR_ASSIGN_OR_RETURN(
      const analytical_geometry::Plane3d plane_with_normal_to_a2,
      analytical_geometry::Plane3d::Create(
          /*origin=*/base_t_joint[0].translation(),
          /*normal=*/axis_in_base_frame[1]));
  INTR_ASSIGN_OR_RETURN(const analytical_geometry::Line3d line1,
                        analytical_geometry::Line3d::Create(
                            /*origin=*/base_t_joint[1].translation(),
                            /*direction=*/axis_in_base_frame[1]));
  INTR_ASSIGN_OR_RETURN(base_p_joint_canonical[1],
                        analytical_geometry::PlaneLineIntersection(
                            plane_with_normal_to_a2, line1));
  joint_canonical_i_p_joint_canonical_i_plus_one[0] =
      base_p_joint_canonical[1] - base_p_joint_canonical[0];
  if (std::fabs(joint_canonical_i_p_joint_canonical_i_plus_one[0](1)) >
          kEpsilon ||
      std::fabs(joint_canonical_i_p_joint_canonical_i_plus_one[0](2)) >
          kEpsilon) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Both the y and z-components of "
        "`joint_canonical_i_p_joint_canonical_i_plus_one[0]` must be 0, "
        "but instead have values ",
        joint_canonical_i_p_joint_canonical_i_plus_one[0](1), ", ",
        joint_canonical_i_p_joint_canonical_i_plus_one[0](2),
        ", respectively."));
  }
  // Although the value of `parameters.l12x` below only contains the absolute
  // value of `joint_canonical_i_p_joint_canonical_i_plus_one[0](0)`, the sign
  // (-/+) information of `joint_canonical_i_p_joint_canonical_i_plus_one[0](0)`
  // will be stored by `parameters.joint_offsets[0]`, i.e. if
  // `joint_canonical_i_p_joint_canonical_i_plus_one[0](0)` < 0,
  // `parameters.joint_offsets[0]` will contain a value of (2 * n + 1) * M_PI
  // for n being any integers, otherwise `parameters.joint_offsets[0]` will
  // contain a value of (2 * n) * M_PI. These are verified towards the end of
  // this `ExtractParameters()` method.
  parameters.l12x =
      std::fabs(joint_canonical_i_p_joint_canonical_i_plus_one[0](0));

  // Joint 3 (defined by A3 (`axis_in_base_frame[2]`)) must be parallel to joint
  // 2 (defined by A2 (`axis_in_base_frame[1]`)).
  if (std::fabs(std::fabs(axis_in_base_frame[1].dot(axis_in_base_frame[2])) -
                1) > kEpsilon) {
    return absl::InvalidArgumentError(
        "The axes of joints 2 and 3 must be parallel to each other.");
  }

  // Joint 3 must be on the plane containing A1 (`axis_in_base_frame[0]`) and
  // normal to A2 (`axis_in_base_frame[1]`).
  INTR_ASSIGN_OR_RETURN(const analytical_geometry::Line3d line2,
                        analytical_geometry::Line3d::Create(
                            /*origin=*/base_t_joint[2].translation(),
                            /*direction=*/axis_in_base_frame[2]));
  INTR_ASSIGN_OR_RETURN(base_p_joint_canonical[2],
                        analytical_geometry::PlaneLineIntersection(
                            plane_with_normal_to_a2, line2));
  joint_canonical_i_p_joint_canonical_i_plus_one[1] =
      base_p_joint_canonical[2] - base_p_joint_canonical[1];
  if (std::fabs(joint_canonical_i_p_joint_canonical_i_plus_one[1](1)) >
      kEpsilon) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The y-component of "
        "`joint_canonical_i_p_joint_canonical_i_plus_one[1]` must be 0, "
        "but instead have the value ",
        joint_canonical_i_p_joint_canonical_i_plus_one[1](1), "."));
  }
  // Although the value of `parameters.l23z` below only contains the norm
  // information of `joint_canonical_i_p_joint_canonical_i_plus_one[1]`, the
  // remaining information is stored in `parameters.joint_offsets[1]`, in a way
  // such that the relationship
  // (`joint_canonical_i_p_joint_canonical_i_plus_one[1]` ==
  // RotationMatrixAroundTheYaxisWithAngle(`parameters.joint_offsets[1]`) * [0,
  // 0, `parameters.l23z`].transpose()) is always satisfied. These are verified
  // towards the end of this `ExtractParameters()` method.
  parameters.l23z = joint_canonical_i_p_joint_canonical_i_plus_one[1].norm();

  // Joint 4 (defined by A4 (`axis_in_base_frame[3]`)) must be orthogonal to
  // joint 3 (defined by A3 (`axis_in_base_frame[2]`)).
  if (std::fabs(axis_in_base_frame[2].dot(axis_in_base_frame[3])) >= kEpsilon) {
    return absl::InvalidArgumentError(
        "The axes of joints 3 and 4 must be orthogonal to each other.");
  }

  // Joint 4 must be located where axis of A4 (`axis_in_base_frame[3]`)
  // intersects the plane going through A3 (`axis_in_base_frame[2]`) and normal
  // to A4.
  INTR_ASSIGN_OR_RETURN(
      const analytical_geometry::Plane3d plane_with_normal_to_a4,
      analytical_geometry::Plane3d::Create(
          /*origin=*/base_t_joint[2].translation(),
          /*normal=*/axis_in_base_frame[3]));
  INTR_ASSIGN_OR_RETURN(const analytical_geometry::Line3d line3,
                        analytical_geometry::Line3d::Create(
                            /*origin=*/base_t_joint[3].translation(),
                            /*direction=*/axis_in_base_frame[3]));
  INTR_ASSIGN_OR_RETURN(base_p_joint_canonical[3],
                        analytical_geometry::PlaneLineIntersection(
                            plane_with_normal_to_a4, line3));
  joint_canonical_i_p_joint_canonical_i_plus_one[2] =
      base_p_joint_canonical[3] - base_p_joint_canonical[2];
  const double joint_canonical_2_p_joint_canonical_3_dot_axis_3_in_base_frame =
      joint_canonical_i_p_joint_canonical_i_plus_one[2].dot(
          axis_in_base_frame[3]);
  if (std::fabs(
          joint_canonical_2_p_joint_canonical_3_dot_axis_3_in_base_frame) >
      kEpsilon) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The dot product between "
        "`joint_canonical_i_p_joint_canonical_i_plus_one[2]` and "
        "`axis_in_base_frame[3]` must be 0, "
        "but instead have the value ",
        joint_canonical_2_p_joint_canonical_3_dot_axis_3_in_base_frame,
        "; `joint_canonical_i_p_joint_canonical_i_plus_one[2]` = [",
        joint_canonical_i_p_joint_canonical_i_plus_one[2](0), ", ",
        joint_canonical_i_p_joint_canonical_i_plus_one[2](1), ", ",
        joint_canonical_i_p_joint_canonical_i_plus_one[2](2),
        "] and `axis_in_base_frame[3]` = [", axis_in_base_frame[3](0), ", ",
        axis_in_base_frame[3](1), ", ", axis_in_base_frame[3](2), "]."));
  }
  parameters.l34y =
      joint_canonical_i_p_joint_canonical_i_plus_one[2].dot(Vector3d::UnitY());
  Vector3d v34 = joint_canonical_i_p_joint_canonical_i_plus_one[2] -
                 parameters.l34y * Vector3d::UnitY();
  parameters.l34z = v34.norm();

  // Joint 5 should be located at the intersection of the A4 axis and A5 axis.
  INTR_ASSIGN_OR_RETURN(
      const analytical_geometry::LineSegment3d line_segment_on_a4_axis,
      analytical_geometry::LineSegment3d::Create(
          /*p1=*/base_t_joint[3].translation(),
          /*p2=*/base_t_joint[3].translation() - axis_in_base_frame[3]));
  INTR_ASSIGN_OR_RETURN(
      const analytical_geometry::LineSegment3d line_segment_on_a5_axis,
      analytical_geometry::LineSegment3d::Create(
          /*p1=*/base_t_joint[4].translation(),
          /*p2=*/base_t_joint[4].translation() + axis_in_base_frame[4]));
  INTR_ASSIGN_OR_RETURN(
      const analytical_geometry::LineSegment3d intersection_a4_a5,
      analytical_geometry::ShortestLineSegmentBtwTwoLines(
          line_segment_on_a4_axis, line_segment_on_a5_axis,
          /*disable_line_segment_length_check=*/true));

  if (intersection_a4_a5.length() >= kEpsilon) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "A4 and A5 should be intersecting. Missed by "
           << intersection_a4_a5.length();
  }
  base_p_joint_canonical[4] =
      (intersection_a4_a5.p1() + intersection_a4_a5.p2()) / 2;
  joint_canonical_i_p_joint_canonical_i_plus_one[3] =
      base_p_joint_canonical[4] - base_p_joint_canonical[3];
  const Vector3d& v45 = joint_canonical_i_p_joint_canonical_i_plus_one[3];
  parameters.l45x = v45.norm();

  // The tool mounting frame should be located at the end of the arm which
  // should be a given.
  base_p_joint_canonical[5] = base_t_joint[5].translation();
  parameters.l56x =
      (base_p_joint_canonical[5] - base_p_joint_canonical[4]).norm();

  // Check that A6 is intersecting with A4 and A5
  INTR_ASSIGN_OR_RETURN(
      const analytical_geometry::LineSegment3d line_segment_on_a6_axis,
      analytical_geometry::LineSegment3d::Create(
          /*p1=*/base_t_joint[5].translation(),
          /*p2=*/base_t_joint[5].translation() + axis_in_base_frame[5]));
  absl::StatusOr<analytical_geometry::LineSegment3d>
      intersection_a4_a6_or_status =
          analytical_geometry::ShortestLineSegmentBtwTwoLines(
              line_segment_on_a4_axis, line_segment_on_a6_axis,
              /*disable_line_segment_length_check=*/true);
  if (intersection_a4_a6_or_status.ok()) {
    if (intersection_a4_a6_or_status.value().length() >= kEpsilon) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "A4 and A6 should be intersecting. Missed by "
             << intersection_a4_a6_or_status.value().length();
    }
  } else {
    // Verify that the parallel axis are co-linear
    if (!analytical_geometry::AreThreePointsCollinear(
            base_t_joint[3].translation(),
            base_t_joint[3].translation() - axis_in_base_frame[3],
            base_t_joint[5].translation() + axis_in_base_frame[5], 1e-5)) {
      return absl::InvalidArgumentError("A4 and A6 should be intersecting.");
    }
  }

  INTR_ASSIGN_OR_RETURN(
      const analytical_geometry::LineSegment3d intersection_a5_a6,
      analytical_geometry::ShortestLineSegmentBtwTwoLines(
          line_segment_on_a5_axis, line_segment_on_a6_axis,
          /*disable_line_segment_length_check=*/true));
  if (intersection_a5_a6.length() >= kEpsilon) {
    return absl::InvalidArgumentError("A5 and A6 should be intersecting.");
  }

  // The effective arm length of A3 = |A3,A5|
  parameters.lhypot35 = hypot(parameters.l34z, parameters.l45x);

  // Let's do some sanity checks here, to make sure the assumptions are correct:
  if (v34.norm() < kEpsilon) {
    return absl::InternalError(absl::StrCat(
        "v34.norm() is expected not to be close to 0, but its actual value is ",
        v34.norm(), ", which is less than the threshold ", kEpsilon, "."));
  }
  if (v45.norm() < kEpsilon) {
    return absl::InternalError(absl::StrCat(
        "v45.norm() is expected not to be close to 0, but its actual value is ",
        v45.norm(), ", which is less than the threshold ", kEpsilon, "."));
  }
  double dot_product_btw_normalized_v34_and_normalized_v45 =
      v34.normalized().dot(v45.normalized());
  if (std::fabs(dot_product_btw_normalized_v34_and_normalized_v45) > kEpsilon) {
    return absl::InternalError(absl::StrCat(
        "`v34.normalized()` vector and `v45.normalized()` vector are expected "
        "to be perpendicular to each other, so their dot product is expected "
        "to be close to 0, but the actual dot product value is ",
        dot_product_btw_normalized_v34_and_normalized_v45,
        ", which is greater than the threshold ", kEpsilon, "."));
  }
  double sign45x_determiner =
      (v34.normalized().cross(v45.normalized())).dot(Vector3d::UnitY());
  if (std::abs(std::abs(sign45x_determiner) - 1.0) > kEpsilon) {
    return absl::InternalError(
        absl::StrCat("The magnitude of `sign45x_determiner` is expected to be "
                     "close to 1.0, but the actual value is ",
                     sign45x_determiner, ", which is more than the threshold ",
                     kEpsilon, "-away from 1.0."));
  }
  if (sign45x_determiner < 0) {
    double v45_dot_unit_x = v45.dot(Vector3d::UnitX());
    if (std::abs(v45_dot_unit_x) < kEpsilon) {
      return absl::InternalError(absl::StrCat(
          "The magnitude of `v45.dot(Vector3d::UnitX())` is expected not to be "
          "close to 0, but the actual value is ",
          v45_dot_unit_x, ", which is less than the threshold ", kEpsilon,
          ". This is an un-handled case with respect to the bugfix for "
          "b/437055366, which needs to be revisited."));
    }
    sign45x_determiner *= v45_dot_unit_x;
  }
  // alpha35 is the angle between |A5,A4| and |A4,A3|, but we also need to take
  // into account the sign as reflected by `sign45x_determiner` above.
  parameters.alpha35_p_pi_2 =
      std::atan2(parameters.l34z,
                 copysign(parameters.l45x, sign45x_determiner)) +
      M_PI_2;

  // These are used in the law of cosines solution formulation with
  // c^2 = a^2 + b^2 - 2*a*b*cos(beta)
  // This would be 1/(2*a*b)
  parameters.one_over_l23z_m_lhypot35 =
      std::fabs(1.0 / (2.0 * parameters.l23z * parameters.lhypot35));
  // This would be a^2 - c^2
  parameters.l23z2_m_lhypot352 = parameters.l23z * parameters.l23z -
                                 parameters.lhypot35 * parameters.lhypot35;

  for (size_t i = 0; i < kNbJoints; ++i) {
    DVLOG(2) << "base_p_joint_canonical " << i << "="
             << toString(base_p_joint_canonical[i]);
  }

  // Compute the joint offsets from the canonical configuration

  // The code assume that the zero position of the robot is in the following
  // configuration:
  //      A4     A5  A6
  //       +-----+---+
  //       |
  //       +A3
  //       |
  //       |
  //   +---+A2
  //  A1
  // parameters._|parameters._
  // Where + represents the joints. All joint frames are align to the inertial
  // frame, excepts for A6, where a6_x is pointing towards ref_z- and a6_z is
  // points towards ref_x-.
  //
  // The robot is imported in its zero configuration. As a results, we need
  // compute the offset in the joint value between the computed one and the one
  // desired by the provided configuration.

  // We define the expected zero-angle plane for each joint. The direction of
  // the plane normal defines the direction of the rotation.
  const Vector3d zero_planes_expected_normals[kNbJoints] = {
      Vector3d::UnitY(), Vector3d::UnitX(), -Vector3d::UnitX(),
      Vector3d::UnitZ(), Vector3d::UnitZ(), Vector3d::UnitZ()};

  // The zero value of a joint is defined by the plane that rotate around the
  // axis of rotation of the joint and going through the child joint
  // canonical position.
  std::array<std::unique_ptr<analytical_geometry::Plane3d>, kNbJoints>
      zero_planes;
  std::array<bool, kNbJoints> joint_is_collinear;
  std::fill(joint_is_collinear.begin(), joint_is_collinear.end(), false);
  for (size_t i = 0; i < kNbJoints - 1; ++i) {
    Vector3d orientation_point = base_p_joint_canonical[i + 1];

    if (i == 0 && analytical_geometry::AreThreePointsCollinear(
                      orientation_point, base_p_joint_canonical[i],
                      base_p_joint_canonical[i] + axis_in_base_frame[i])) {
      // This can happen when A2 is on A1 axis. In this case, we simply use a
      // point on the world x-axis as a reference point since there is no other
      // reliable point to use.
      INTR_ASSIGN_OR_RETURN(
          zero_planes[0], analytical_geometry::PlaneFromThreePoints(
                              base_p_joint_canonical[0],
                              base_p_joint_canonical[0] + axis_in_base_frame[0],
                              Vector3d::UnitX()));
    } else if (i == 2 &&
               (analytical_geometry::AreThreePointsCollinear(
                    base_p_joint_canonical[i], base_p_joint_canonical[i + 1],
                    base_p_joint_canonical[i + 1] +
                        axis_in_base_frame[i + 1]) ||
                analytical_geometry::AreThreePointsCollinear(
                    orientation_point, base_p_joint_canonical[i],
                    base_p_joint_canonical[i] + axis_in_base_frame[i]))) {
      // This can happen when A3 is on A4 axis. We then use the A4 axis as the
      // normal of the zero plane.
      INTR_ASSIGN_OR_RETURN(analytical_geometry::Plane3d plane_i,
                            analytical_geometry::Plane3d::Create(
                                /*origin=*/base_p_joint_canonical[i],
                                /*normal=*/axis_in_base_frame[i + 1]));
      zero_planes[i] = std::make_unique<analytical_geometry::Plane3d>(plane_i);
    } else if (analytical_geometry::AreThreePointsCollinear(
                   orientation_point, base_p_joint_canonical[i],
                   base_p_joint_canonical[i] + axis_in_base_frame[i])) {
      // When the axis is collinear to the next joint, the sign of the offset is
      // ambiguous. We add an offset in the direction of the axis on the last
      // point so that we avoid defining a plane with collinear points when
      // joint are aligned.
      joint_is_collinear[i] = true;

      orientation_point += axis_in_base_frame[i + 1];

      INTR_ASSIGN_OR_RETURN(
          zero_planes[i], analytical_geometry::PlaneFromThreePoints(
                              base_p_joint_canonical[i],
                              base_p_joint_canonical[i] + axis_in_base_frame[i],
                              orientation_point));
    } else {
      INTR_ASSIGN_OR_RETURN(
          zero_planes[i], analytical_geometry::PlaneFromThreePoints(
                              base_p_joint_canonical[i],
                              base_p_joint_canonical[i] + axis_in_base_frame[i],
                              orientation_point));
    }
  }
  // For the last joint, we use the previous joint as a reference.
  INTR_ASSIGN_OR_RETURN(zero_planes[kNbJoints - 1],
                        analytical_geometry::PlaneFromThreePoints(
                            base_p_joint_canonical[kNbJoints - 1],
                            base_p_joint_canonical[kNbJoints - 1] +
                                axis_in_base_frame[kNbJoints - 1],
                            base_p_joint_canonical[kNbJoints - 2] +
                                axis_in_base_frame[kNbJoints - 2]));

  // Define the default rotation axes, their directions, and offset values.
  // The solver assumes this specific configuration, so we will use these values
  // to transform different kinematic configurations to work with the solver.
  const std::array<Vector3d, kNbJoints> joint_rot_axis_default_in_base_frame = {
      Vector3d::UnitZ(),  Vector3d::UnitY(),  -Vector3d::UnitY(),
      -Vector3d::UnitX(), -Vector3d::UnitY(), -Vector3d::UnitX()};

  std::array<eigenmath::Matrix3d, kNbJoints> base_r_offsetted_joint;
  base_r_offsetted_joint[0] = eigenmath::Matrix3d::Identity();

  // Calculate joint offsets and axis direction difference for each joint
  for (size_t i = 0; i < kNbJoints; i++) {
    // parameters.joint_axis_directions_diff should be set to 1 or -1 depending
    // on if the joint's rotation and default axes are in the same direction or
    // not.
    const eigenmath::Vector3d offsetted_axis_in_base_frame =
        base_r_offsetted_joint[i] * axis_in_base_frame[i];
    eigenmath::Vector3d offsetted_zero_planes_normal =
        (base_r_offsetted_joint[i] * zero_planes[i]->normal()).normalized();
    if (joint_rot_axis_default_in_base_frame[i].dot(
            offsetted_axis_in_base_frame) < 0) {
      parameters.joint_axis_directions_diff[i] = -1.0;

      // We should flip the plane to avoid introducing a pi offset.
      zero_planes[i]->normal() *= -1.0;
      offsetted_zero_planes_normal *= -1;
    } else {
      parameters.joint_axis_directions_diff[i] = 1.0;
    }

    double dot =
        zero_planes_expected_normals[i].dot(offsetted_zero_planes_normal);
    if (joint_is_collinear[i]) {
      // When the axis is collinear to the next joint (`joint_is_collinear[i]`
      // is true), the direction of `zero_planes[i]->normal()` is ambiguous,
      // i.e. `zero_planes[i]->normal()` will flip when `axis_in_base_frame[i +
      // 1]` is flipped (please see how `axis_in_base_frame[i + 1]` is added
      // into `orientation_point` above).
      //
      // Hence we compensate by modifying the dot product below. This will
      // ensure that when `axis_in_base_frame[i]` is flipped,
      // `parameters.joint_axis_directions_diff[i]` will flip as well.
      if (dot != -std::fabs(dot)) {
        dot = -std::fabs(dot);
        zero_planes[i]->normal() *= -1.0;
        offsetted_zero_planes_normal *= -1.0;
      }
    }
    eigenmath::Vector3d cross =
        zero_planes_expected_normals[i].cross(offsetted_zero_planes_normal);
    if (cross.norm() < kEpsilon) {
      cross = eigenmath::Vector3d::Zero();
    }

    parameters.joint_offsets[i] = std::atan2(cross.norm(), dot);
    const double measurement_axis_aligment =
        cross.dot(offsetted_axis_in_base_frame);
    if (measurement_axis_aligment < -kEpsilon) {
      parameters.joint_offsets[i] *= -1;
    }

    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint_i,
                                  model.GetJoint(dof_chain[i]));
    double joint_frame_offset = RotationAroundAxis(
        eigenmath::SO3d(joint_i->GetParentTThis().quaternion()),
        joint_i->GetAxis());
    joint_frame_offset *= parameters.joint_axis_directions_diff[i];

    // For the ortho-parallel robot, such as the bettina, we don't need to do
    // this.
    // TODO(jeanfrancoisd): This is somewhat of a hack and a more robust
    // approach should be done.
    if (joint_is_collinear[i] && (std::fabs(parameters.l34y) < 1e-4)) {
      // When the joint is collinear, we use the angle offset that aligns the
      // reference plane to the closest rotation specified by the initial
      // configuration of the robot.
      // We add some post-processing on cases where the absolute difference
      // between the joint offset and frame offset is close to pi, i.e. pi +/-
      // `kConsideredOffsetAbsoluteDifferenceDeviationFromPiRadians`.
      constexpr double
          kConsideredOffsetAbsoluteDifferenceDeviationFromPiRadians = 0.25;
      if (std::fabs(std::fabs(std::fabs(parameters.joint_offsets[i]) -
                              std::fabs(joint_frame_offset)) -
                    M_PI) <
          kConsideredOffsetAbsoluteDifferenceDeviationFromPiRadians) {
        parameters.joint_offsets[i] +=
            (parameters.joint_offsets[i] < -kEpsilon) ? M_PI : -M_PI;
      }
    }

    if (i == 5) {
      // For the last joint, we need to add the offset as specified by the joint
      // frame.
      parameters.joint_offsets[i] = joint_frame_offset;

      // There is also a pi offset assumed by the code.
      parameters.joint_offsets[i] -= M_PI;
    }

    if (i < kNbJoints - 1) {
      base_r_offsetted_joint[i + 1] =
          base_r_offsetted_joint[i] *
          Eigen::AngleAxis<double>(-parameters.joint_offsets[i],
                                   axis_in_base_frame[i])
              .matrix();
    }
  }

  for (size_t i = 0; i < kNbJoints; ++i) {
    DVLOG(1) << "Joint " << i << " offset: " << parameters.joint_offsets[i]
             << ", directions: " << parameters.joint_axis_directions_diff[i];
    // Parameter verifications:
    if (i == 0) {
      if (joint_canonical_i_p_joint_canonical_i_plus_one[0](0) < 0) {
        if (std::fabs(
                std::fabs(remainder(parameters.joint_offsets[i], 2.0 * M_PI)) -
                M_PI) > kEpsilon) {
          return absl::InternalError(absl::StrCat(
              "`joint_canonical_i_p_joint_canonical_i_plus_one[0](0)` = ",
              joint_canonical_i_p_joint_canonical_i_plus_one[0](0),
              " is negative, but `parameters.joint_offsets[0]` = ",
              parameters.joint_offsets[0],
              " does not satisfy the condition that "
              "abs(`parameters.joint_offsets[0]` modulo (2 * "
              "`M_PI`)) is close to `M_PI`."));
        }
      } else {
        if (std::fabs(remainder(parameters.joint_offsets[i], 2.0 * M_PI)) >
            kEpsilon) {
          return absl::InternalError(absl::StrCat(
              "`joint_canonical_i_p_joint_canonical_i_plus_one[0](0)` = ",
              joint_canonical_i_p_joint_canonical_i_plus_one[0](0),
              " is positive, but `parameters.joint_offsets[0]` = ",
              parameters.joint_offsets[0],
              " does not satisfy the condition that "
              "abs(`parameters.joint_offsets[0]` modulo (2 * "
              "`M_PI`)) is close to zero."));
        }
      }
    } else if (i == 1) {
      if ((joint_canonical_i_p_joint_canonical_i_plus_one[1] -
           (eigenmath::AngleAxisd(parameters.joint_offsets[1],
                                  Vector3d::UnitY()) *
            Vector3d::UnitZ() * parameters.l23z))
              .norm() > kEpsilon) {
        return absl::InternalError(absl::StrCat(
            "The relationship "
            "(`joint_canonical_i_p_joint_canonical_i_plus_one[1]` == "
            "RotationMatrixAroundTheYaxisWithAngle(`parameters.joint_offsets[1]"
            "`)"
            " * [0, 0, `parameters.l23z`].transpose()) is not satisfied! "
            "`joint_canonical_i_p_joint_canonical_i_plus_one[1]` = [",
            joint_canonical_i_p_joint_canonical_i_plus_one[1](0), ", ",
            joint_canonical_i_p_joint_canonical_i_plus_one[1](1), ", ",
            joint_canonical_i_p_joint_canonical_i_plus_one[1](2),
            "], and `parameters.joint_offsets[1]` = ",
            parameters.joint_offsets[1], "."));
      }
    }
  }

  // Show the parameter vector as defined in:
  // "Brandstötter, Mathias & Angerer, Arthur & Hofbaur, Michael. (2014). An
  // Analytical Solution of the Inverse Kinematics Problem of Industrial Serial
  // Manipulators with an Ortho-parallel Basis and a Spherical Wrist."
  DVLOG(1) << "Equivalent ortho-basis parameters for model " << model.GetName()
           << ": [a1,a2,b,c1,c2,c3,c4] = [" << parameters.l12x << ", "
           << parameters.l34z << ", " << parameters.l34y << ", "
           << parameters.basel1z << ", " << parameters.l23z << ", "
           << parameters.l45x << ", " << parameters.l56x << "]";

  return parameters;
}

SphericalWristIKSolver::SphericalWristIKSolver(const Chain* chain)
    : chain_(chain) {
  CHECK(chain_ != nullptr);
  nearby_q_.resize(chain_->GetNumberDegreesOfFreedom());
  nearby_q_.setZero();
}

absl::Status SphericalWristIKSolver::Init() {
  CHECK(chain_ != nullptr);
  if (chain_->GetNumberDegreesOfFreedom() != kNbJoints) {
    return absl::InvalidArgumentError("The robot should have 6-dof.");
  }
  INTR_ASSIGN_OR_RETURN(parameters_, ExtractParameters(*chain_));
  is_initialized_ = true;

  dof_limits_ = chain_->GetDofSystemLimits();
  return absl::OkStatus();
}

icon::RealtimeStatus SphericalWristIKSolver::WrapAndSort(
    const ModelInterface& model, const JointLimits& dof_limits,
    const eigenmath::VectorNd& nearby_q, absl::Span<eigenmath::VectorNd> qs) {
  DCHECK(is_initialized_);
  const int n_dof = model.GetNumberDegreesOfFreedom();
  if (nearby_q.size() != n_dof || dof_limits.size() != n_dof) {
    // We check size here to make sure we don't die in the sort loop.
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size mismatch, ndof=", n_dof, ", nearby_q=", nearby_q.size(),
        ", dof_limits=", dof_limits.size()));
  }
  if (!dof_limits.IsValid()) {
    return icon::InvalidArgumentError("Limits are invalid.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto is_nearby_q_within_limits,
                                IsWithinLimits(nearby_q, dof_limits));
  if (!is_nearby_q_within_limits) {
    return icon::InvalidArgumentError(
        "Nearby joint values should be within the dof limits.");
  }

  for (auto& q : qs) {
    if (q.allFinite()) {
      INTRINSIC_RT_RETURN_IF_ERROR(
          Wrap(model, dof_limits, nearby_q, &q).status());
    }
  }

  // Use a custom comparison operator to sort the joint configurations according
  // to limit respecting, distance to nearby_q, and branch.
  std::sort(
      qs.begin(), qs.end(),
      [&nearby_q, &dof_limits, this](const eigenmath::VectorNd& qa,
                                     const eigenmath::VectorNd& qb) {
        // qa invalid: choose qb
        INTRINSIC_RT_ASSIGN_OR_DIE(auto qa_is_within_limits,
                                   IsWithinLimits(qa, dof_limits));
        if (!qa.allFinite() || !qa_is_within_limits) {
          return false;
        }
        // qa valid and qb invalid: choose qa
        INTRINSIC_RT_ASSIGN_OR_DIE(auto qb_is_within_limits,
                                   IsWithinLimits(qb, dof_limits));
        if (!qb.allFinite() || !qb_is_within_limits) {
          return true;
        }

        const double qa_distance = (qa - nearby_q).norm();
        const double qb_distance = (qb - nearby_q).norm();

        // if qa and qb are equidistant, and one of them is on the same
        // branch as nearby_q, choose that one. The formulation here might seem
        // a bit bloated, but is necessary to fulfil strict/weak ordering.
        constexpr double kSameDistanceEpsilon = 1.0e-6;
        if (AlmostEquals(qa_distance, qb_distance, kSameDistanceEpsilon)) {
          const int branch_qa = GetBranch(qa);
          const int branch_qb = GetBranch(qb);
          const int branch_nearby_q = GetBranch(nearby_q);
          return std::abs(branch_qa - branch_nearby_q) <
                 std::abs(branch_qb - branch_nearby_q);
        }

        // qa and qb valid: choose the one closest to nearby_q (metric
        // based on L2 norm).
        return qa_distance < qb_distance;
      });

  return icon::OkStatus();
}

bool SphericalWristIKSolver::Solve(const Pose3d& base_t_tip,
                                   VectorNd* solution_q) {
  DCHECK(is_initialized_);
  SphericalWristIKSolutionSet solution_qs;
  if (Solve(base_t_tip, &solution_qs) == 0) {
    return false;
  }

  auto wrap_and_sort_status =
      WrapAndSort(*chain_, dof_limits_, nearby_q_, absl::MakeSpan(solution_qs));
  if (!wrap_and_sort_status.ok()) {
    LOG(ERROR) << "WrapAndSort failed: " << wrap_and_sort_status;
    return false;
  }
  *solution_q = solution_qs[0];
  return true;
}

int SphericalWristIKSolver::Solve(
    const Pose3d& base_t_tip_no_offset,
    SphericalWristIKSolutionSet* solution_qs_ptr) const {
  DCHECK(is_initialized_);

  using eigenmath::Matrix3d;
  using eigenmath::Matrix4d;
  using eigenmath::Vector2d;
  using eigenmath::Vector4d;
  DCHECK_NE(nullptr, solution_qs_ptr);

  const Pose3d base_t_tip = parameters_.base_offset * base_t_tip_no_offset;

  // Create reference to avoid de-referencing ptr throughout this function.
  SphericalWristIKSolutionSet& solution_qs = *solution_qs_ptr;
  for (VectorNd& it : solution_qs) {
    it.resize(kNbJoints);
  }

  // Uses the algorithm from
  // intrinsic/controller/kinematics/six_dof_kinematics.h, date:
  // 3/1/2017

  constexpr double kIkAlmostZero = 1e-8;

  // Temporary solutions set used during computation.
  SphericalWristIKSolutionSet solution_qs_tmp;
  for (VectorNd& it : solution_qs_tmp) {
    it.resize(kNbJoints);
  }

  VectorNd lower_limits(kNbJoints);
  VectorNd upper_limits(kNbJoints);
  VectorNd offset_nearby_q(kNbJoints);
  // We can't rely on GetDofSoftPositionLimits since they can change over time.
  // TODO(jeanfrancoisd): Provide a state to get the soft limits or rely on
  // external soft limit validation.
  for (size_t i = 0; i < kNbJoints; i++) {
    if (parameters_.joint_axis_directions_diff[i] == 1) {
      lower_limits[i] =
          dof_limits_.min_position[i] + parameters_.joint_offsets[i];
      upper_limits[i] =
          dof_limits_.max_position[i] + parameters_.joint_offsets[i];
      offset_nearby_q[i] = nearby_q_[i] + parameters_.joint_offsets[i];
    } else if (parameters_.joint_axis_directions_diff[i] == -1) {
      lower_limits[i] =
          -dof_limits_.max_position[i] - parameters_.joint_offsets[i];
      upper_limits[i] =
          -dof_limits_.min_position[i] - parameters_.joint_offsets[i];
      offset_nearby_q[i] = -nearby_q_[i] - parameters_.joint_offsets[i];
    } else {
      LOG(FATAL) << "joint_axis_directions_diff should contains -1 or 1. "
                    "joint_axis_directions_diff["
                 << i << "]=" << parameters_.joint_axis_directions_diff[i];
    }
  }

  // Position P of 5-th frame, this is where the axes intersect.
  Vector3d j1_in_base_frame;
  j1_in_base_frame << 0, 0, parameters_.basel1z;
  Vector3d p_in_base_frame = base_t_tip.translation() - j1_in_base_frame -
                             base_t_tip.zAxis() * parameters_.l56x;

  // Find solution for q(0).
  std::array<double, 2> q0;
  std::array<SphericalWristIKResultFlags, 2> q0_result_flags;
  q0_result_flags.fill(RESULT_BLANK);

  // Looking at the projection of the wrist position on the xy-plane, we can
  // find the angle to the xz-plane.
  const double pxy = hypot(p_in_base_frame[0], p_in_base_frame[1]);
  if (pxy <= kIkAlmostZero && std::fabs(parameters_.l34y) <= kIkAlmostZero) {
    // This situation happen when the wrist is aligned with the q0 axis.
    // Therefore, q0 can take any values. The value adopted is the one provided
    // as a the nearby configuration.
    DVLOG(2) << "Wrist lies on the axis of A1.";
    q0_result_flags[0] = RESULT_POS_Q0_INDETERMINITE;
    q0_result_flags[1] = RESULT_POS_Q0_INDETERMINITE;
    q0[0] = offset_nearby_q[0];
    // Also provide a backward solution.
    q0[1] = M_PI + offset_nearby_q[0];
  } else {
    if (std::fabs(parameters_.l34y) > kIkAlmostZero) {
      double nx_comp = p_in_base_frame[0] * p_in_base_frame[0] +
                       p_in_base_frame[1] * p_in_base_frame[1] -
                       parameters_.l34y * parameters_.l34y;
      if (nx_comp < 0 && abs(nx_comp) < kEpsilon) {
        nx_comp = 0.0;
      }
      const double nx1 = std::sqrt(nx_comp) - parameters_.l12x;
      const double q0_comp1 =
          std::atan2(p_in_base_frame[1], p_in_base_frame[0]);
      const double q0_comp2 =
          std::atan2(parameters_.l34y, nx1 + parameters_.l12x);
      // There are two solution,forward and backward.
      q0 = {q0_comp1 - q0_comp2, q0_comp1 + q0_comp2 - M_PI};
    } else {
      // When there wrist is in the plane of the first axis, the equation are
      // simplified. Equivalent to the code above but kept for
      // simplicity/accuracy.
      const double q = std::atan2(p_in_base_frame[1], p_in_base_frame[0]);
      // There is two solution,forward and backward.
      q0 = {q, M_PI + q};
    }
  }

  for (size_t j = 0; j < q0.size(); ++j) {
    q0[j] = WrapJointAndSetFlag(/*q=*/q0[j], /*upper_limit=*/upper_limits[0],
                                /*lower_limit=*/lower_limits[0],
                                /*flag=*/&q0_result_flags[j]);
  }

  // Find solutions for q(2) by analysing triangle A2/A3/A4 for a zero
  // configuration as shown below in the xz-plane.
  //      A4     A5  A6
  //       +-----+---+
  //       |     P
  //       +A3
  //       |
  //       |
  //   +---+A2
  //  A1
  std::array<double, 4> q1;
  std::array<double, 4> q2;

  const double& dz = p_in_base_frame[2];
  const double dz2 = p_in_base_frame[2] * p_in_base_frame[2];

  std::array<SphericalWristIKResultFlags, kMaxNumberSolutions> result_flags;
  result_flags.fill(RESULT_BLANK);

  // For the two solution of q0, shoulder forward and shoulder backward
  for (size_t i = 0; i < 2; ++i) {
    result_flags[2 * i] = q0_result_flags[i];
    result_flags[2 * i + 1] = q0_result_flags[i];

    // We add a small positive number `kEpsilon` to handle the case where
    // `parameters_.l12x` is zero.
    Vector3d a2 = {(parameters_.l12x + kEpsilon) * std::cos(q0[i]),
                   (parameters_.l12x + kEpsilon) * std::sin(q0[i]), 0};
    Vector3d a2_dir = a2.normalized();

    // We need to add or substract the |A1,A2| length from the wrist position
    // projected on the XY-plane. To establish the sign, we look at the sign of
    // the projection of the |A2,P| on the direction vector of |A1,A2|.
    double dxy = copysign(parameters_.l12x - p_in_base_frame.dot(a2_dir),
                          (p_in_base_frame - a2).dot(a2_dir));

    // H2 is the squared length of the segment |A2,P|
    const double H2 = dxy * dxy + dz2;
    const double H = std::sqrt(H2);

    double cos_gamma = std::numeric_limits<double>::quiet_NaN();

    double max_reach = parameters_.l23z + parameters_.lhypot35;
    double min_reach = std::fabs(parameters_.l23z - parameters_.lhypot35);

    // Check extended reach by looking at |A2,A3| + |A3,A5| >= |A2,P|
    if (H > max_reach) {
      if (H - max_reach < 5e-6) {
        DVLOG(2) << "Solution " << 2 * i << " and " << 2 * i + 1
                 << " are too far but within numerical precision tolerances";
        cos_gamma = -1;
      } else {
        // This is out of reach as full extension can't reach.
        result_flags[2 * i] = RESULT_POS_UNREACHABLE;
        result_flags[2 * i + 1] = RESULT_POS_UNREACHABLE;
        DVLOG(2) << "Solution " << 2 * i << " and " << 2 * i + 1
                 << " are unreachable, too far by " << H - max_reach;
        cos_gamma = -1;
      }
    }

    // Check collapsed reach by looking at |A2,A3| - |A3,A5| <= |A2,P|
    if (H < min_reach) {
      if (min_reach - H < 5e-6) {
        DVLOG(2) << "Solution " << 2 * i << " and " << 2 * i + 1
                 << " are too close but within numerical precision tolerances";
        cos_gamma = 1;
      } else {
        result_flags[2 * i] = RESULT_POS_UNREACHABLE;
        result_flags[2 * i + 1] = RESULT_POS_UNREACHABLE;
        DVLOG(2) << "Solution " << 2 * i << " and " << 2 * i + 1
                 << " are unreachable, too close by " << min_reach - H;
        cos_gamma = 1;
      }
    }

    if (!isfinite(cos_gamma)) {
      // From the law of cosines, we find the angle gamma between |A3,A2| and
      // |A3,P|
      cos_gamma = (-H2 + parameters_.l23z * parameters_.l23z +
                   parameters_.lhypot35 * parameters_.lhypot35) *
                  parameters_.one_over_l23z_m_lhypot35;
    }

    // Using sin^2(theta) + cos^2(theta) = 1;
    double sin_gamma = std::sqrt(1 - cos_gamma * cos_gamma);
    const double gamma = std::atan2(sin_gamma, cos_gamma);

    // We could have used +-acos(cos_gamma) since +-acos(cos_gamma) =
    // +-atan2(sin_gamma, cos_gamma). However atan2 is more numerically stable.
    // For instance, if cos_gamma is sliglty out of [-1,1], acos would be a
    // complex number. However, atan is bounded to [-pi/2, pi/2], so
    // atan(infinity)=pi/2.

    // There is an "elbow up" and an "elbow down" solution.
    const std::array<double, 2> gammas = {-gamma, gamma};
    for (size_t j = 0; j < gammas.size(); ++j) {
      // Note: MH80 has >360 deg. range for q2 according to data sheet, but
      // it's not clear that is possible.
      // If we need that: add all wrap_angled solutions that still are in
      // admissible range as additional solutions.
      q2[2 * i + j] = WrapJointAndSetFlag(
          /*q=*/gammas[j] - parameters_.alpha35_p_pi_2,
          /*upper_limit=*/upper_limits[2],
          /*lower_limit=*/lower_limits[2], /*flag=*/&result_flags[2 * i + j]);

      // Compute solution for q1 for each q2
      // We take q2 such that q2=0 is aligned with vector A2,A3
      const double q2_aligned = gammas[j] - M_PI;
      double q1_tmp = std::atan2(dz, dxy) -
                      std::atan2(parameters_.lhypot35 * std::sin(q2_aligned),
                                 parameters_.l23z + parameters_.lhypot35 *
                                                        std::cos(q2_aligned));

      // We add pi/2 to the q1 computed above since the reference zero-position
      // of the arm has |A2,A3| vertical.
      q1[2 * i + j] = WrapJointAndSetFlag(/*q=*/M_PI_2 - q1_tmp,
                                          /*upper_limit=*/upper_limits[1],
                                          /*lower_limit=*/lower_limits[1],
                                          /*flag=*/&result_flags[2 * i + j]);

      solution_qs_tmp[2 * i + j][0] = q0[i];
      solution_qs_tmp[2 * i + j][1] = q1[2 * i + j];
      solution_qs_tmp[2 * i + j][2] = q2[2 * i + j];
    }
  }

  // Copy base joint solutions to second set of solution vectors.
  solution_qs_tmp[4] = solution_qs_tmp[0];
  solution_qs_tmp[5] = solution_qs_tmp[1];
  solution_qs_tmp[6] = solution_qs_tmp[2];
  solution_qs_tmp[7] = solution_qs_tmp[3];

  result_flags[4] = result_flags[0];
  result_flags[5] = result_flags[1];
  result_flags[6] = result_flags[2];
  result_flags[7] = result_flags[3];

  // Add wrist solutions for all valid solutions.
  // Note first and second half of m_q have the same solutions for the first
  // three joints, so only loop until SphericalWristMaxNSolutions / 2.
  for (unsigned int ii = 0; ii < kMaxNumberSolutions / 2; ii++) {
    auto& q = solution_qs_tmp[ii];
    const double cq0 = std::cos(q[0]);
    const double cq12 = std::cos(q[1] - q[2]);
    const double sq0 = std::sin(q[0]);
    const double sq12 = std::sin(q[1] - q[2]);

    Matrix3d mat;
    mat << cq0 * cq12, -sq0, cq0 * sq12, sq0 * cq12, cq0, sq0 * sq12, -sq12,
        0.0, cq12;

    // Required transform from wrist to tool.
    Matrix3d R36 = mat.transpose() * base_t_tip.rotationMatrix();

    std::array<double, 2> q3;
    std::array<double, 2> q5;
    std::array<double, 2> q4;

    // The value of `kR36ZeroThreshold` below affects the sensitivity behavior
    // of the (Analytical) Spherical Wrist IK Solver around the
    // kinematic-singularity of the 5th joint, as shown in
    // b/506093975#comment10 .
    constexpr double kR36ZeroThreshold = 2e-5;
    if (std::fabs(R36(0, 0)) < kR36ZeroThreshold &&
        std::fabs(R36(0, 1)) < kR36ZeroThreshold) {
      // q4 == 0, so q3 and q5 are indeterminate.
      result_flags[ii] = SphericalWristIKResultFlags(
          result_flags[ii] | RESULT_POS_Q35_INDETERMINATE);

      double q35 = std::atan2(R36(1, 0), R36(2, 0));
      // Get closest match to reference solution.
      // Move q35 to the range corresponding to the reference solution.
      const double q35_ref = offset_nearby_q[3] + offset_nearby_q[5];
      while (q35 - q35_ref > M_PI) {
        q35 -= 2.0 * M_PI;
      }
      while (q35 - q35_ref < -M_PI) {
        q35 += 2.0 * M_PI;
      }

      // Since the hinted given joint configuration is assumed to be within the
      // joint limits, we use at least one given nearby joint values of q3 or q5
      // and apply the fix to the other.
      // TODO(jeanfrancoisd): Splitting the difference can be tricky near joint
      // limits. To do properly, we would need to bound lambda to the joint
      // limit on one side and then apply the reminding on the other.
      // We could also look at the impact on the limit of q5 to readjuste q3.
      const double lambda = offset_nearby_q[3] + offset_nearby_q[5] - q35;
      q3[0] = (offset_nearby_q[3]);
      q5[0] = (offset_nearby_q[5] - lambda);
      // Since we already know both solutions for q4 will be very close to zero,
      // we can safely reuse q3 and q5.
      q3[1] = q3[0];
      q5[1] = q5[0];

      const double sinq4 =
          -(std::cos(q5[0]) * R36(0, 0) + std::sin(q5[0]) * R36(0, 1));

      q4[0] = WrapJointAndSetFlag(/*q=*/std::atan2(sinq4, R36(0, 2)),
                                  /*upper_limit=*/upper_limits[4],
                                  /*lower_limit=*/lower_limits[4],
                                  /*flag=*/&result_flags[ii]);
      q4[1] = q4[0];
      // Only set one solution, flag the other as invalid
      result_flags[ii + kMaxNumberSolutions / 2] = SphericalWristIKResultFlags(
          result_flags[ii] | RESULT_NONE | RESULT_POS_Q35_INDETERMINATE);
    } else {
      // Regular case: joint angles between -pi and pi.
      // Multi-turns solutions are computed later in the getter function by
      // adding k*2*M_PI.

      // Since atan2 returns values between -pi/2 and pi/2, we don't need to
      // wrap the values.
      q3[0] = std::atan2(R36(1, 2), R36(2, 2));
      q3[1] = std::atan2(-R36(1, 2), -R36(2, 2));

      q5[0] = std::atan2(-R36(0, 1), -R36(0, 0));
      q5[1] = std::atan2(R36(0, 1), R36(0, 0));

      const std::array<double, 2> sinq4 = {
          -(std::cos(q5[0]) * R36(0, 0) + std::sin(q5[0]) * R36(0, 1)),
          -(std::cos(q5[1]) * R36(0, 0) + std::sin(q5[1]) * R36(0, 1))};

      q4[0] = WrapJointAndSetFlag(/*q=*/std::atan2(sinq4[0], R36(0, 2)),
                                  /*upper_limit=*/upper_limits[4],
                                  /*lower_limit=*/lower_limits[4],
                                  /*flag=*/&result_flags[ii]);
      q4[1] = WrapJointAndSetFlag(
          /*q=*/std::atan2(sinq4[1], R36(0, 2)),
          /*upper_limit=*/upper_limits[4], /*lower_limit=*/lower_limits[4],
          /*flag=*/&result_flags[ii + kMaxNumberSolutions / 2]);
    }

    solution_qs_tmp[ii][3] = q3[0];
    solution_qs_tmp[ii][4] = q4[0];
    solution_qs_tmp[ii][5] = q5[0];
    solution_qs_tmp[ii + kMaxNumberSolutions / 2][3] = q3[1];
    solution_qs_tmp[ii + kMaxNumberSolutions / 2][4] = q4[1];
    solution_qs_tmp[ii + kMaxNumberSolutions / 2][5] = q5[1];
  }

  // Check if at least one solution has been found.
  size_t valid_index = 0;
  size_t invalid_index = kMaxNumberSolutions - 1;
  for (size_t i = 0; i < kMaxNumberSolutions; i++) {
    auto& q = solution_qs_tmp[i];

    DCHECK(q.allFinite()) << "We should have valid angles q=" << toString(q)
                          << " flags: " << ToString(result_flags[i]);

    q = parameters_.joint_axis_directions_diff.array() * q.array() -
        parameters_.joint_offsets.array();

    // A final check on limits
    if (!IsWithinLimits(q, dof_limits_.min_position,
                        dof_limits_.max_position)) {
      // We try to wrap all dof first to see if would bring them back within the
      // limit.
      for (size_t j = 0; j < kNbJoints; ++j) {
        q[j] = WrapJointAndSetFlag(/*q=*/q[j],
                                   /*upper_limit=*/dof_limits_.max_position[j],
                                   /*lower_limit=*/dof_limits_.min_position[j],
                                   /*flag=*/&result_flags[i]);
      }
    }

    DCHECK_LE(valid_index, invalid_index);

    if (!(result_flags[i] & RESULT_NONE) &&
        !(result_flags[i] & RESULT_POS_UNREACHABLE) &&
        !(result_flags[i] & RESULT_JOINT_LIMIT_EXCEEDED)) {
      DVLOG(2) << "Adding valid solution[" << valid_index
               << "] q=" << toString(q)
               << ", flag=" << ToString(result_flags[i]);
      solution_qs[valid_index++] = q;
    } else {
      DVLOG(2) << "Adding invalid solution[" << invalid_index
               << "] q=" << toString(q)
               << ", flag=" << ToString(result_flags[i]);
      solution_qs[invalid_index--] = q;
    }
  }

  return valid_index;
}

FixedVector<eigenmath::VectorNd, SphericalWristIKSolver::kMaxNumberSolutions>
SphericalWristIKSolver::Solve(const Pose3d& base_t_tip) const {
  SphericalWristIKSolutionSet solution_set;
  int solution_count = Solve(base_t_tip, &solution_set);
  FixedVector<eigenmath::VectorNd, kMaxNumberSolutions> result;
  for (int i = 0; i < solution_count; ++i) {
    result.push_back(solution_set[i]);
  }
  return result;
}

void SphericalWristIKSolver::GetDisplacementToSingularity(
    const VectorNd& q_orig, double* displacement_to_wrist_singularity,
    double* displacement_to_elbow_singularity,
    double* displacement_to_overhead_singularity) const {
  DCHECK(is_initialized_);
  DCHECK_EQ(q_orig.size(), 6);

  DCHECK_NE(nullptr, displacement_to_wrist_singularity);
  DCHECK_NE(nullptr, displacement_to_elbow_singularity);
  DCHECK_NE(nullptr, displacement_to_overhead_singularity);

  VectorNd q_with_offsets(6);
  for (size_t i = 0; i < 6; i++) {
    q_with_offsets[i] = q_orig[i] + parameters_.joint_offsets[i];
    q_with_offsets[i] *= parameters_.joint_axis_directions_diff[i];
  }

  // Offset to fully stretched wrist, normalized with Pi
  *displacement_to_wrist_singularity = q_with_offsets[4] / M_PI;

  // Offset to fully stretched elbow, normalized with Pi
  const double a3_angle_stretched =
      std::atan2(parameters_.l45x, parameters_.l34z);
  *displacement_to_elbow_singularity =
      (q_with_offsets[2] - a3_angle_stretched) / M_PI;

  // Offset to wrist exactly above shoulder, normalized with length of fully
  // stretched arm
  //
  //             |----> d35xy = distance from j3 to j5 in xy plane
  //
  //                  o j5
  //                 /
  //                /
  //               /
  //              o     j3              z
  //               \                  ^
  //                \                 |
  //                 o  j2            |parameters_.___> x, y
  //
  //              <--|  d23xy = distance from j2 to j3 in xy plane

  const double d35 = std::sqrt(
      intrinsic::IPow(parameters_.l34z, 2) +
      intrinsic::IPow(parameters_.l45x, 2));  // distance from j3 to j5
  double alpha = -q_with_offsets[1];  // left angle between vertical and j2-j3
  double beta =
      alpha + q_with_offsets[2] +
      std::atan2(parameters_.l34z,
                 parameters_.l45x);  // angle between horizontal and j3-j5

  double d23xy =
      parameters_.l23z * std::sin(alpha);  // distance from j2 to j3 in xy plane
  double d35xy = d35 * std::cos(beta);     // distance from j3 to j5 in xy plane

  *displacement_to_overhead_singularity =
      (d23xy - d35xy) / (parameters_.l23z + d35);
}

size_t SphericalWristIKSolver::GetConfiguration(const VectorNd& q) const {
  DCHECK(is_initialized_);
  double displacement_to_wrist_singularity;
  double displacement_to_elbow_singularity;
  double displacement_to_overhead_singularity;

  GetDisplacementToSingularity(q, &displacement_to_wrist_singularity,
                               &displacement_to_elbow_singularity,
                               &displacement_to_overhead_singularity);

  size_t configuration = 0;

  if (displacement_to_wrist_singularity > 0) {
    configuration |= 1 << 0;
  }

  if (displacement_to_elbow_singularity > 0) {
    configuration |= 1 << 1;
  }

  if (displacement_to_overhead_singularity > 0) {
    configuration |= 1 << 2;
  }

  return configuration;
}

size_t SphericalWristIKSolver::GetBranch(const VectorNd& q) const {
  DCHECK(is_initialized_);
  DCHECK_EQ(q.size(), kNbJoints);

  VectorNd q_with_offsets(kNbJoints);
  for (int i = 0; i < kNbJoints; i++) {
    q_with_offsets[i] = q[i] + parameters_.joint_offsets[i];
    q_with_offsets[i] *= parameters_.joint_axis_directions_diff[i];
  }

  // Unpack joint states required for branch labeling
  const double j2 = q_with_offsets[1];
  const double j3 = q_with_offsets[2];
  const double j5 = q_with_offsets[4];

  size_t branch = 0;

  // Determine the waist branch
  const double d35 = std::sqrt(
      intrinsic::IPow(parameters_.l34z, 2) +
      intrinsic::IPow(parameters_.l45x, 2));  // distance from j3 to j5
  const double waist_determinant =
      -(parameters_.l12x + parameters_.l23z * std::sin(j2) +
        d35 * std::cos(-j2 + j3 +
                       std::atan2(parameters_.l34z, parameters_.l45x)));
  if (waist_determinant >= 0.0) branch |= 1 << 0;

  // Determine the elbow branch
  const double elbow_determinant =
      copysign(1.0, waist_determinant) *
      (parameters_.l34z * std::sin(j3) - parameters_.l45x * std::cos(j3));
  // Consider elbow is "up" when j3 ~ 0.0, i.e., arm is stretched out.
  if (elbow_determinant >= 0.0) branch |= 1 << 1;

  // Determine the wrist branch
  const double wrist_determinant =
      -copysign(1.0, waist_determinant) * std::sin(j5);
  if (wrist_determinant >= 0.0) branch |= 1 << 2;

  return branch;
}

double SphericalWristIKSolver::GetMinDistanceToSingularity(
    const VectorNd& q) const {
  DCHECK(is_initialized_);
  double displacement_to_wrist_singularity;
  double displacement_to_elbow_singularity;
  double displacement_to_overhead_singularity;

  GetDisplacementToSingularity(q, &displacement_to_wrist_singularity,
                               &displacement_to_elbow_singularity,
                               &displacement_to_overhead_singularity);

  return std::min(std::min(std::fabs(displacement_to_wrist_singularity),
                           std::fabs(displacement_to_elbow_singularity)),
                  std::fabs(displacement_to_overhead_singularity));
}

double SphericalWristIKSolver::GetMaximumArmLength() const {
  // Please see the documentation of the struct
  // `SphericalWristIKSolver::Parameters` in the header file.
  // This function returns the maximum arm length in meters as:
  // (sqrt(c1^2 + a1^2) + c2 + sqrt(c3^2 + a2^2) + c4).
  return std::sqrt(intrinsic::IPow(parameters_.basel1z, 2) +
                   intrinsic::IPow(parameters_.l12x, 2)) +
         parameters_.l23z +
         std::sqrt(intrinsic::IPow(parameters_.l45x, 2) +
                   intrinsic::IPow(parameters_.l34z, 2)) +
         parameters_.l56x;
}

}  // namespace kinematics
}  // namespace intrinsic
