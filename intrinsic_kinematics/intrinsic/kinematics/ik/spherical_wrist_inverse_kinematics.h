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

#ifndef INTRINSIC_KINEMATICS_IK_SPHERICAL_WRIST_INVERSE_KINEMATICS_H_
#define INTRINSIC_KINEMATICS_IK_SPHERICAL_WRIST_INVERSE_KINEMATICS_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/ik_solvers/spherical_wrist_ik_solver.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

// Implementation of the InverseKinematicsInterface for closed form 6-DoF
// spherical wrist kinematics. Uses spherical_wrist_ik_solver under the hood.
class SphericalWristInverseKinematics : public InverseKinematicsInterfaceRT {
 public:
  using InverseKinematicsInterface::ComputeIK;
  using InverseKinematicsInterface::RealtimeComputeIK;

  static constexpr char kSolverName[] = "spherical_wrist";

  static absl::StatusOr<std::unique_ptr<SphericalWristInverseKinematics>>
  Create(Chain chain, const InverseKinematicsInterface::Options& options);

  explicit SphericalWristInverseKinematics(Chain chain)
      : InverseKinematicsInterfaceRT(std::move(chain)) {}

  // Prepares the underlying solver. Needs to be called before attempting to
  // compute IK.
  absl::Status Init();

  int GetNumDof() const override { return 6; }

  const std::string_view GetName() const override { return kSolverName; }

  bool ImplementsSameBranchIK() const override { return true; }

  absl::StatusOr<size_t> ComputeBranch(
      const JointStateP& joint_state) const override;

  icon::RealtimeStatusOr<IKResult> RealtimeComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits,
      absl::Span<JointStateP> solutions) const override;

  icon::RealtimeStatusOr<IKResult> RealtimeComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits, bool ensure_same_branch,
      JointStateP* solution) const override;

  std::optional<ElementId> RetrieveMatchingStartJoint(
      const ModelInterface& model, ElementId end_joint) const override {
    LOG(FATAL) << "Not implemented.";
    return std::nullopt;
  }

  bool IsCompatibleWithChain(const Chain& chain) const override {
    LOG(FATAL) << "Not implemented.";
    return false;
  }

  // Returns the (ground-truth) maximum arm length of the robot
  // manipulator, computed by the Spherical Wrist Inverse Kinematics (IK)
  // solver's extracted kinematic parameters (e.g. those extracted from the
  // robot manipulator's URDF file). The maximum arm length is defined as the
  // maximum distance between the base and the tip of the robot manipulator.
  absl::StatusOr<double> GetMaximumArmLength() const override;

 private:
  std::unique_ptr<SphericalWristIKSolver> ik_solver_ = nullptr;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SPHERICAL_WRIST_INVERSE_KINEMATICS_H_
