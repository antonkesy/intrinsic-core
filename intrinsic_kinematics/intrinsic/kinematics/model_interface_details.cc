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

#include "intrinsic/kinematics/model_interface_details.h"

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {
namespace details {

icon::RealtimeStatusOr<ElementId> GetIdForElement(
    const absl::flat_hash_map<ElementId, std::unique_ptr<Element>>& element_map,
    const Element* element) {
  for (auto const& [key, val] : element_map) {
    if (val.get() == element) {
      return key;
    }
  }
  return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
      "Element ", element->GetName(), " not part of element_map."));
}

bool ElementIsContained(const std::vector<ElementId>& element_ids,
                        const ElementId& id) {
  auto it = std::find(element_ids.begin(), element_ids.end(), id);
  return (it != element_ids.end());
}

bool ElementIsDof(const ModelInterface& model,
                  const std::vector<ElementId>& joint_ids,
                  const ElementId& id) {
  if (!ElementIsContained(joint_ids, id)) {
    return false;
  }

  // Check type of joint. We currently do not support fixed as dof.
  INTRINSIC_RT_ASSIGN_OR_DIE(const auto* joint, model.GetJoint(id));
  return joint->IsDof();
}

icon::RealtimeStatusOr<ElementId> GetIdForElement(
    const absl::flat_hash_map<ElementId, const Element*>& element_map,
    const Element* element) {
  for (auto const& [key, val] : element_map) {
    if (val == element) {
      return key;
    }
  }
  return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
      "Element ", element->GetName(), " not part of element_map."));
}

// Decides an order of joints according to the following criteria
// * Sort lexicographically by value function expression
// * Sort by motion type
// * Sort by translation component
// * Sort by quaternion component in order of descending values
// * Sort by axis
// Follows the sorting function as implemented in the world kinematics:
// https://github.com/intrinsic-ai/intrinsic-core/blob/main/google3/intrinsic/world/world.cc#L2608
// Please note: Not all criteria are implemented yet. Distance and orientation
// with respect to parent joint will be completed in subsequent submissions.
bool operator<(const Joint& lhs, const Joint& rhs) {
  // TODO(kmuelling): Implement distance to parent joint criteria instead of
  // parent.
  const auto& ptt1 = lhs.GetParentTThis();
  const auto& ptt2 = rhs.GetParentTThis();
  const auto& axis1 = lhs.GetAxis();
  const auto& axis2 = rhs.GetAxis();

  // TODO(kmuelling): make the sorting function use directly the one used in
  // world without the need of copy and paste. Note that the orientation
  // components of parent_t_inboard are inverted because they're sorted in
  // ascending order.
  return std::make_tuple(lhs.GetType(), ptt1.translation().x(),
                         ptt1.translation().y(), ptt1.translation().z(),
                         -ptt1.quaternion().w(), -ptt1.quaternion().x(),
                         -ptt1.quaternion().y(), -ptt1.quaternion().z(),
                         axis1.x(), axis1.y(), axis1.z()) <
         std::make_tuple(rhs.GetType(), ptt2.translation().x(),
                         ptt2.translation().y(), ptt2.translation().z(),
                         -ptt2.quaternion().w(), -ptt2.quaternion().x(),
                         -ptt2.quaternion().y(), -ptt2.quaternion().z(),
                         axis2.x(), axis2.y(), axis2.z());
}

bool operator<(const Pose3d& ptt1, const Pose3d& ptt2) {
  return std::make_tuple(ptt1.translation().x(), ptt1.translation().y(),
                         ptt1.translation().z(), -ptt1.quaternion().w(),
                         -ptt1.quaternion().x(), -ptt1.quaternion().y(),
                         -ptt1.quaternion().z()) <
         std::make_tuple(ptt2.translation().x(), ptt2.translation().y(),
                         ptt2.translation().z(), -ptt2.quaternion().w(),
                         -ptt2.quaternion().x(), -ptt2.quaternion().y(),
                         -ptt2.quaternion().z());
}

// Return the ElementId of first parent to element_id that is a DOF. Will return
// kInvalidElementId if no more DOF are present between element_id and root.
icon::RealtimeStatusOr<ElementId> GetPreviousDofInTree(
    const ModelInterface& model, const ElementId& element_id) {
  auto current_id = element_id;
  while (current_id != kInvalidElementId) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto element, model.GetElement(current_id));
    auto parent_element_or = element->GetParentElement();
    if (parent_element_or == std::nullopt) {
      // We reached the root of the tree.
      return kInvalidElementId;
    }

    auto current_id_or = model.GetElementId(*parent_element_or);
    if (!current_id_or.ok()) {
      // The element is not part of the model. In a skeleton, this shouldn't
      // happen, but for a group, it means that we reached the root of the
      // included element.
      return kInvalidElementId;
    }
    current_id = current_id_or.value();

    const auto joint_or = model.GetJoint(current_id);
    if (joint_or.ok() && joint_or.value()->IsDof()) {
      // Found dof
      return current_id;
    }
  }
  return icon::InternalError(icon::RealtimeStatus::StrCat(
      "We shouldn't get here. Potentially loop in model: ", model.GetName()));
}

absl::StatusOr<std::map<ElementId, std::vector<ElementId>>>
UpdateDofChainMapping(const ModelInterface& model) {
  std::map<ElementId, std::vector<ElementId>> new_chain_map;
  for (const auto& tip_id : model.GetTipIds()) {
    auto current_id = tip_id;
    std::vector<ElementId> tip_chain;
    auto& dof_chain = new_chain_map[tip_id];

    // Handle the case when tip is a dof
    const auto joint_or = model.GetJoint(current_id);
    if (joint_or.ok() && joint_or.value()->IsDof()) {
      dof_chain.push_back(current_id);
    }

    while (current_id != kInvalidElementId) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(current_id,
                                    GetPreviousDofInTree(model, current_id));
      if (current_id != kInvalidElementId) {
        dof_chain.push_back(current_id);
      }
    }
    std::reverse(dof_chain.begin(), dof_chain.end());
  }
  return new_chain_map;
}

}  // namespace details
}  // namespace kinematics
}  // namespace intrinsic
