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

#ifndef INTRINSIC_KINEMATICS_IK_CALIBRATED_UR_INVERSE_KINEMATICS_H_
#define INTRINSIC_KINEMATICS_IK_CALIBRATED_UR_INVERSE_KINEMATICS_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/kinematics/ik_solvers/ur_ik_solver.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

enum class UrRobotType { Ur3e, Ur5e, Ur10e, Ur16e };

// This class provides inverse kinematics for calibrated Universal Robotics'
// robots.
//
// This class computes an analytical IK result based on the nominal kinematic
// chain and refines the solution based on a calibrated kinematic chain with an
// iterative solver. There is partial validation of the passed model, but care
// should be taken to confirm that the model conforms to the required structure.
class CalibratedUrInverseKinematics : public InverseKinematicsInterfaceNonRT {
 public:
  using InverseKinematicsInterface::ComputeIK;

  static constexpr char kSolverName[] = "calibrated_ur";

  static absl::StatusOr<std::unique_ptr<CalibratedUrInverseKinematics>> Create(
      Chain chain, const UrRobotType& arm_type,
      const InverseKinematicsInterface::Options& options =
          InverseKinematicsInterface::Options());

  int GetNumDof() const override { return 6; }

  const std::string_view GetName() const override { return kSolverName; }

  bool ImplementsSameBranchIK() const override { return true; }

  absl::StatusOr<size_t> ComputeBranch(
      const JointStateP& joint_state) const override;

  absl::StatusOr<IKResult> ComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits,
      absl::Span<JointStateP> solutions) const override;

  absl::StatusOr<IKResult> ComputeIK(const JointStateP& hint_joint_state,
                                     const Pose3d& desired_base_t_tip,
                                     const JointLimits& limits,
                                     bool ensure_same_branch,
                                     JointStateP* solution) const override;

  std::optional<ElementId> RetrieveMatchingStartJoint(
      const ModelInterface& model, ElementId end_joint) const override {
    LOG(ERROR) << "Not implemented.";
    return std::nullopt;
  }

  bool IsCompatibleWithChain(const Chain& chain) const override {
    // TODO(b/265173620): Correct this implementation.
    return true;
  }

 private:
  explicit CalibratedUrInverseKinematics(Chain chain)
      : InverseKinematicsInterfaceNonRT(std::move(chain)) {}

  // Prepares the underlying solver. Needs to be called before attempting to
  // compute IK.
  absl::Status Init(const UrRobotType& arm_type);

  std::unique_ptr<kinematics::Skeleton> nominal_skeleton_;
  Chain nominal_chain_;

  std::unique_ptr<intrinsic::kinematics::URIKSolver> ur_ik_solver_;
  std::unique_ptr<intrinsic::kinematics::KinematicChainIKSolver>
      chain_ik_solver_;

  const absl::flat_hash_map<UrRobotType, std::string>
      arm_type_to_skeleton_proto_ = {
          {UrRobotType::Ur3e,
           "intrinsic_kinematics/intrinsic/kinematics/"
           "ur_proto_skeletons/ur3e.pbtxt"},
          {UrRobotType::Ur5e,
           "intrinsic_kinematics/intrinsic/kinematics/"
           "ur_proto_skeletons/ur5e.pbtxt"},
          {UrRobotType::Ur10e,
           "intrinsic_kinematics/intrinsic/kinematics/"
           "ur_proto_skeletons/ur10e.pbtxt"},
          {UrRobotType::Ur16e,
           "intrinsic_kinematics/intrinsic/kinematics/"
           "ur_proto_skeletons/ur16e.pbtxt"}};
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_CALIBRATED_UR_INVERSE_KINEMATICS_H_
