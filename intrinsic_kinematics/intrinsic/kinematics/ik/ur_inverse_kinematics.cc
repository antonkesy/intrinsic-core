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

#include "intrinsic/kinematics/ik/ur_inverse_kinematics.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/rotation_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/ik_solvers/ur_ik_solver.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/joint_wrapping.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {
namespace {

using eigenmath::Vector3d;

bool IsCloseTo(const Vector3d& value, const Vector3d& target, double eps) {
  return (value - target).norm() < eps;
}

// Returns Ok if the passed model defines kinematics that can be solved by the
// UrInverseKinematics class. However, note that there are kinematic
// descriptions of UR arms that the class doesn't handle and will therefore be
// rejected by this function.
//
// This function is more strict than necessary to make parameter extraction
// easier (certain link lengths, and plane offsets for example).
absl::Status ValidateURKinematics(const ModelInterface& model) {
  if (model.GetNumberDegreesOfFreedom() != 6) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Invalid joint count: %d", model.GetNumberDegreesOfFreedom()));
  }

  if (!model.HasOneTip()) {
    return absl::InvalidArgumentError("model should be a chain.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto dof_chain, model.GetDofChainForTip(model.GetTipIds().front()));

  // Run FK and get the position of each joint at the zero position.
  //
  // All joints must be revolute. We check this here as well.
  std::array<intrinsic::Pose3d, 6> base_t_joint_poses;
  for (int i = 0; i < 6; i++) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint_i,
                                  model.GetJoint(dof_chain[i]));
    if (joint_i->GetParameters().type != Joint::Type::REVOLUTE) {
      return absl::InvalidArgumentError(
          absl::StrFormat("Joint %d must be a revolute joint.", i));
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

  // Expect that the first frame has only a z rotation and its axis is
  // positive z.
  eigenmath::Vector3d base_t_joint0_axis_angle =
      eigenmath::QuaternionToAngleTimesAxis(
          base_t_joint_poses.at(0).quaternion());
  if (std::abs(base_t_joint0_axis_angle.x()) > kMaxRotationError ||
      std::abs(base_t_joint0_axis_angle.y()) > kMaxRotationError) {
    return absl::InvalidArgumentError(
        "Joint 0 must only have a z rotation in base frame.");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint_0,
                                model.GetJoint(dof_chain[0]));
  if (!IsCloseTo(joint_0->GetAxis(), Vector3d(0, 0, 1), kMaxVectorError)) {
    return absl::InvalidArgumentError(
        "Joint 0 axis must be (0, 0, 1) in base frame.");
  }

  // Joints 1, 2, 3 just lie on the x-z plane (in joint 0 frame), and must have
  // their axis be in direction (0, 1, 0).
  for (int i = 1; i < 4; i++) {
    if ((base_t_joint_poses.at(0).quaternion().inverse() *
         base_t_joint_poses.at(i).translation())
            .dot(Vector3d(0, 1, 0)) > kMaxVectorError) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Joint %d expected to lie on plane x-z plane of joint 0 frame.", i));
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint_i,
                                  model.GetJoint(dof_chain[i]));
    if (!IsCloseTo(base_t_joint_poses.at(0).quaternion().inverse() *
                       base_t_joint_poses.at(i).quaternion() *
                       joint_i->GetAxis(),
                   Vector3d(0, 1, 0), kMaxVectorError)) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Joint %d axis must be (0, 1, 0) in joint 0 frame", i));
    }
  }

  // Joint 4 must have a positive offset in the y direction in the joint 0 frame
  // (which influences a hard-coded offset when using computing j0). Its axis
  // should be +- z in joint 0 frame.
  if ((base_t_joint_poses.at(0).rotationMatrix().transpose() *
       base_t_joint_poses.at(4).translation())
          .dot(Vector3d(0, 1, 0)) <= 0) {
    return absl::InvalidArgumentError(
        "Joint 4 expected to be offset in the positive y direction in joint 0 "
        "frame.");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint_4,
                                model.GetJoint(dof_chain[4]));
  Vector3d joint_4_axis_in_joint_0_frame =
      base_t_joint_poses.at(0).quaternion().inverse() *
      base_t_joint_poses.at(4).quaternion() * joint_4->GetAxis();
  if (!IsCloseTo(joint_4_axis_in_joint_0_frame, Vector3d(0, 0, -1),
                 kMaxVectorError) &&
      !IsCloseTo(joint_4_axis_in_joint_0_frame, Vector3d(0, 0, 1),
                 kMaxVectorError)) {
    return absl::InvalidArgumentError(
        "Joint 4 axis must be (0, 0, +-1) in joint 0 frame");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint_3,
                                model.GetJoint(dof_chain[3]));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint_5,
                                model.GetJoint(dof_chain[5]));

  // Joint 5 must have the same axis direction as joint 3.
  if (!IsCloseTo(base_t_joint_poses.at(3).quaternion() * joint_3->GetAxis(),
                 base_t_joint_poses.at(5).quaternion() * joint_5->GetAxis(),
                 kMaxVectorError)) {
    return absl::InvalidArgumentError(
        "Joint 5 axis must be the same as joint 3 axis");
  }

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<std::unique_ptr<UrInverseKinematics>>
UrInverseKinematics::Create(
    Chain chain, const InverseKinematicsInterface::Options& options) {
  auto ik_solver = std::make_unique<UrInverseKinematics>(std::move(chain));
  INTR_RETURN_IF_ERROR(ik_solver->Init());
  return std::move(ik_solver);
}

absl::Status UrInverseKinematics::Init() {
  INTR_RETURN_IF_ERROR(ValidateURKinematics(chain()));

  ik_solver_ = std::make_unique<URIKSolver>();
  return ik_solver_->Init(&chain());
}

bool UrInverseKinematics::IsCompatibleWithChain(const Chain& chain) const {
  return ValidateURKinematics(chain).ok();
}

icon::RealtimeStatusOr<InverseKinematicsInterface::IKResult>
UrInverseKinematics::RealtimeComputeIK(
    const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
    const JointLimits& limits, absl::Span<JointStateP> solutions) const {
  if (!ik_solver_) {
    return icon::InternalError(
        "The solver has not been properly initialized. Did you call Init?");
  }
  const int num_requested_solutions = solutions.size();
  if (num_requested_solutions < 1) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The solutions span needs to have size 1 or bigger, but it is: ",
        num_requested_solutions));
  }

  if (hint_joint_state.size() != GetNumDof()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The provided hint vector is of size ", hint_joint_state.size(),
        " but the chain has num DOFs: ", chain().GetNumberDegreesOfFreedom()));
  }

  std::array<intrinsic::eigenmath::VectorNd, URIKSolver::kSolutionBufferSize>
      candidate_solutions;

  const auto candidate_solution_count = ik_solver_->Solve(
      desired_base_t_tip, hint_joint_state.position, &candidate_solutions);

  if (candidate_solution_count == 0) {
    return InverseKinematicsInterface::IKResult(
        {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
         .number_of_solutions = 0});
  }

  auto candidate_solution_span =
      absl::MakeSpan(candidate_solutions).subspan(0, candidate_solution_count);

  INTRINSIC_RT_RETURN_IF_ERROR(
      WrapAndSort(chain(), chain().GetDofSystemLimits(),
                  hint_joint_state.position, &candidate_solution_span));

  int valid_solution_count = 0;
  for (int i = 0; valid_solution_count < num_requested_solutions &&
                  i < candidate_solution_count;
       i++) {
    JointStateP candidate_solution;
    candidate_solution.position = candidate_solution_span[i];
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto limit_check,
                                  IsWithinLimits(candidate_solution, limits));
    if (!limit_check.p_ok) {
      continue;
    }
    solutions[valid_solution_count] = candidate_solution;
    valid_solution_count++;
  }

  return InverseKinematicsInterface::IKResult(
      {.status = valid_solution_count > 0
                     ? InverseKinematicsInterface::IKResult::OK
                     : InverseKinematicsInterface::IKResult::NO_SOLUTION,
       .number_of_solutions = valid_solution_count});
}

icon::RealtimeStatusOr<InverseKinematicsInterface::IKResult>
UrInverseKinematics::RealtimeComputeIK(const JointStateP& hint_joint_state,
                                       const Pose3d& desired_base_t_tip,
                                       const JointLimits& limits,
                                       bool ensure_same_branch,
                                       JointStateP* solution) const {
  // If same branch IK is not enabled, return the default IK solution.
  if (!ensure_same_branch)
    return RealtimeComputeIK(hint_joint_state, desired_base_t_tip, limits,
                             solution);

  // Sanity checks for user-provided input and solver.
  if (solution == nullptr) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Provided with a null pointer to `solution`. Please provide a valid "
        "pointer to a JointStateP object."));
  }
  if (!ik_solver_) {
    return icon::InternalError(
        "The solver has not been properly initialized. Did you call Init?");
  }
  if (hint_joint_state.size() != GetNumDof()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The provided hint vector is of size ", hint_joint_state.size(),
        " but the chain has num DOFs: ", chain().GetNumberDegreesOfFreedom()));
  }

  // Calculate all IK solutions
  std::array<intrinsic::eigenmath::VectorNd, URIKSolver::kSolutionBufferSize>
      candidate_solutions;

  const auto candidate_solution_count = ik_solver_->Solve(
      desired_base_t_tip, hint_joint_state.position, &candidate_solutions);

  if (candidate_solution_count == 0) {
    return InverseKinematicsInterface::IKResult(
        {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
         .number_of_solutions = 0});
  }

  // Get the branch of the hint joint state.
  const size_t hint_joint_state_branch =
      ik_solver_->GetBranch(hint_joint_state.position);

  // Return the first IK solution with the same branch as the hint joint state.
  for (int i = 0; i < candidate_solution_count; i++) {
    const size_t branch = ik_solver_->GetBranch(candidate_solutions[i]);
    if (branch == hint_joint_state_branch) {
      // Wrap the solution close to the hint joint state before returning.
      INTRINSIC_RT_RETURN_IF_ERROR(Wrap(chain(), limits,
                                        hint_joint_state.position,
                                        &candidate_solutions[i])
                                       .status());
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const auto limit_check,
          IsWithinLimits(candidate_solutions[i], limits));
      if (!limit_check.p_ok) {
        return InverseKinematicsInterface::IKResult(
            {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
             .number_of_solutions = 0});
      }

      *solution = candidate_solutions[i];
      return InverseKinematicsInterface::IKResult(
          {.status = InverseKinematicsInterface::IKResult::OK,
           .number_of_solutions = 1});
    }
  }

  // Return result with no solution since none of the IK solutions had the same
  // branch as the hint joint state.
  return InverseKinematicsInterface::IKResult(
      {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
       .number_of_solutions = 0});
}

absl::StatusOr<size_t> UrInverseKinematics::ComputeBranch(
    const JointStateP& joint_state) const {
  return ik_solver_->GetBranch(joint_state.position);
}

}  // namespace kinematics
}  // namespace intrinsic
