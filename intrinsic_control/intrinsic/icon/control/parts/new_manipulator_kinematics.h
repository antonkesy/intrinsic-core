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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_NEW_MANIPULATOR_KINEMATICS_H_
#define INTRINSIC_ICON_CONTROL_PARTS_NEW_MANIPULATOR_KINEMATICS_H_

#include <map>
#include <memory>
#include <string_view>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

// Helper class to allow L1ArmPart to optionally provide ManipulatorKinematics.
class NewManipulatorKinematicsImpl final : public ManipulatorKinematics {
 public:
  explicit NewManipulatorKinematicsImpl(
      std::unique_ptr<kinematics::InverseKinematicsInterface>
          inverse_kinematics,
      std::unique_ptr<kinematics::Skeleton> skeleton);

  const kinematics::InverseKinematicsInterface& GetInverseKinematicsSolver()
      const override {
    return *inverse_kinematics_;
  }

  std::string_view GetInverseKinematicsSolverName() const override {
    return inverse_kinematics_->GetName();
  }

  const kinematics::Skeleton& GetKinematicsModel() const override {
    return *skeleton_;
  }

  RealtimeStatusOr<const kinematics::Chain*> GetKinematicsChain(
      kinematics::ElementId tip_id) const override;

  RealtimeStatusOr<Pose3d> ComputeChainFK(
      const JointStateP& dof_positions) const override;

  RealtimeStatusOr<eigenmath::Matrix6Nd> ComputeChainJacobian(
      const JointStateP& dof_positions) const override;

 private:
  std::unique_ptr<kinematics::InverseKinematicsInterface> inverse_kinematics_;
  std::unique_ptr<kinematics::Skeleton> skeleton_;
  std::map<kinematics::ElementId, kinematics::Chain> kinematics_chains_;
  // The ID of the "overall tip" element.
  //
  // If `skeleton_` contains a single, non-branching kinematic chain of just
  // joint and link elements, the tip is the final element of that chain.
  //
  // Otherwise, the tip is the first of the tips that `skeleton_` reports. Note
  // that this might be a frame, not a link/joint!
  //
  // We cache the tip because
  // * it does not change at runtime
  // * finding the tip in the first case requires non-trivial graph traversal
  // that we do not want to repeat in the realtime thread
  kinematics::ElementId tip_id_ = kinematics::kInvalidElementId;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_NEW_MANIPULATOR_KINEMATICS_H_
