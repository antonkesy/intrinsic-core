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

#include "intrinsic/icon/control/parts/new_manipulator_kinematics.h"

#include <memory>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

NewManipulatorKinematicsImpl::NewManipulatorKinematicsImpl(
    std::unique_ptr<kinematics::InverseKinematicsInterface>
        inverse_kinematics_interface,
    std::unique_ptr<kinematics::Skeleton> skeleton)
    : inverse_kinematics_(std::move(inverse_kinematics_interface)),
      skeleton_(std::move(skeleton)) {
  if (skeleton_) {
    absl::Span<const kinematics::ElementId> tip_span = skeleton_->GetTipIds();
    absl::flat_hash_set<kinematics::ElementId> tips = {tip_span.begin(),
                                                       tip_span.end()};
    RealtimeStatusOr<kinematics::ElementId> non_frame_tip =
        skeleton_->FindNonBranchingKinematicChainTip();
    if (non_frame_tip.ok()) {
      tip_id_ = non_frame_tip.value();
      tips.insert(non_frame_tip.value());
    } else if (!tip_span.empty()) {
      tip_id_ = tip_span.front();
    }

    // TODO(b/200287437): Revisit when addressing multi-tips in ICON.
    // Technically, loading multiple tips works, but control is not yet
    // possible, so warn early, if:
    // * The Skeleton reports more than one tip (some or all of these might be
    //   frames!)
    // * There is *not* a single non-frame tip element
    if (tips.size() > 1 && !non_frame_tip.ok()) {
      LOG(WARNING) << "Kinematics model has multiple tips. Controlling "
                      "branched kinematics is currently not supported, first "
                      "tip will be used.";
    }
    for (const auto& tip_id : tips) {
      kinematics::Chain chain(absl::StrCat(skeleton_->GetName(), " with tip ",
                                           skeleton_->GetElementName(tip_id)));
      CHECK_OK(
          chain.ExtractFromModel(*skeleton_, skeleton_->GetBaseId(), tip_id));
      kinematics_chains_.insert({tip_id, std::move(chain)});
    }
  }
}

RealtimeStatusOr<const kinematics::Chain*>
NewManipulatorKinematicsImpl::GetKinematicsChain(
    kinematics::ElementId tip_id) const {
  auto it = kinematics_chains_.find(tip_id);
  if (it != kinematics_chains_.end()) {
    return &it->second;
  }
  return intrinsic::icon::NotFoundError(
      RealtimeStatus::StrCat("Tip ", skeleton_->GetElementName(tip_id),
                             "does not match any kinematics chain"));
}

RealtimeStatusOr<Pose3d> NewManipulatorKinematicsImpl::ComputeChainFK(
    const JointStateP& dof_positions) const {
  const kinematics::ModelInterface& model = GetKinematicsModel();
  intrinsic::kinematics::State kinematics_state(&model);
  INTRINSIC_RT_RETURN_IF_ERROR(
      kinematics_state.SetDofPositions(dof_positions, /*check_limits=*/false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(Pose3d base_t_tip,
                                kinematics_state.GetTransform(tip_id_));
  return base_t_tip;
}

RealtimeStatusOr<eigenmath::Matrix6Nd>
NewManipulatorKinematicsImpl::ComputeChainJacobian(
    const JointStateP& dof_positions) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const kinematics::Chain* chain,
                                GetKinematicsChain(tip_id_));
  intrinsic::kinematics::State kinematics_state(chain);
  INTRINSIC_RT_RETURN_IF_ERROR(
      kinematics_state.SetDofPositions(dof_positions, /*check_limits=*/false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto jacobian,
                                kinematics_state.ComputeJacobian());
  return jacobian;
}

}  // namespace intrinsic::icon
