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

#include "intrinsic/kinematics/group.h"

#include <algorithm>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/model_interface_details.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {

Group::Group(absl::string_view name) : ModelInterface(name) {}

void Group::Clear() {
  base_element_id_ = kInvalidElementId;
  element_id_to_element_.clear();
  element_id_to_dof_index_.clear();
  tip_ids_.clear();
  links_.clear();
  joints_.clear();
  coordinate_frames_.clear();
  dof_index_to_joint_id_.clear();
  element_ids_.clear();
  tip_dof_chains_.clear();
}

absl::Status Group::AddElement(const ModelInterface& model,
                               const Element* new_element) {
  if (new_element == nullptr) {
    return absl::InvalidArgumentError("Attempted to add non-existing element.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto new_element_id,
                                model.GetElementId(new_element));

  // Check if element_id is already in the list
  const auto element_it = element_id_to_element_.find(new_element_id);
  if (element_it != element_id_to_element_.end()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "The element " << new_element->GetName()
           << " id=" << new_element_id.value() << " already exist in the group";
  }

  // Add element to set of elements
  element_id_to_element_[new_element_id] = new_element;
  element_ids_.push_back(new_element_id);

  // Find what type of element this is.
  if (std::find(model.GetAllJointIds().begin(), model.GetAllJointIds().end(),
                new_element_id) != model.GetAllJointIds().end()) {
    joints_.push_back(new_element_id);
  } else if (std::find(model.GetAllLinkIds().begin(),
                       model.GetAllLinkIds().end(),
                       new_element_id) != model.GetAllLinkIds().end()) {
    links_.push_back(new_element_id);
  } else if (std::find(model.GetAllCoordinateFrameIds().begin(),
                       model.GetAllCoordinateFrameIds().end(),
                       new_element_id) !=
             model.GetAllCoordinateFrameIds().end()) {
    coordinate_frames_.push_back(new_element_id);
  } else {
    return absl::InvalidArgumentError(absl::StrCat(
        "Can't identify the type of element for ",
        model.GetElementName(new_element_id), " id=", new_element_id.value()));
  }

  return absl::OkStatus();
}

absl::Status Group::ExtractFromModel(const ModelInterface& model,
                                     absl::Span<const ElementId> tip_ids) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto base_id, model.GetCommonAncestor(tip_ids));
  return ExtractFromModel(model, base_id, tip_ids);
}

absl::Status Group::ExtractFromModel(const ModelInterface& model,
                                     const ElementId& base_id,
                                     absl::Span<const ElementId> tip_ids) {
  Clear();
  tip_ids_ = {tip_ids.begin(), tip_ids.end()};
  base_element_id_ = base_id;

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto tip_common_ancestor_id,
                                model.GetCommonAncestor(tip_ids));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto base_ancestor_id,
      model.GetCommonAncestor(base_id, tip_common_ancestor_id));
  if (base_id != base_ancestor_id) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Base " << model.GetElementName(base_id) << " id=" << base_id
           << " is not a common ancestor of the tips common ancestor "
           << model.GetElementName(tip_common_ancestor_id)
           << " id=" << tip_common_ancestor_id;
  }

  // Add the elements from tips to base
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* base_element,
                                model.GetElement(base_element_id_),
                                "while extracting group and getting base.");
  INTR_RETURN_IF_ERROR(AddElement(model, base_element));

  for (auto current_id : tip_ids_) {
    // We add elements until we encounter one that was already inserted.
    while (std::find(element_ids_.begin(), element_ids_.end(), current_id) ==
           element_ids_.end()) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const auto* current_element, model.GetElement(current_id),
          "while extracting group and getting current element.");

      INTR_RETURN_IF_ERROR(AddElement(model, current_element));

      auto parent_element_or = current_element->GetParentElement();
      if (parent_element_or == std::nullopt) {
        return ::intrinsic::InvalidArgumentErrorBuilder()
               << "All travesed elements should have a parent. Element "
               << current_element->GetName() << " id=" << current_id
               << " has no parent.";
      }
      INTRINSIC_RT_ASSIGN_OR_RETURN(current_id,
                                    model.GetElementId(*parent_element_or));
    }
  }

  INTR_RETURN_IF_ERROR(details::UpdateJointDofMapping(
      *this, base_element_id_, element_id_to_element_, joints_,
      element_id_to_dof_index_, dof_index_to_joint_id_))
      << "while updating the joint dof mapping";

  INTR_ASSIGN_OR_RETURN(tip_dof_chains_, details::UpdateDofChainMapping(*this));

  // We copy the solver keys that are relevant for this group.
  for (const auto& [base, tips] : model.GetSolverKeys()) {
    if (!details::ElementIsContained(element_ids_, base)) {
      continue;
    }
    for (const auto& [tip, solver_keys] : tips) {
      if (!details::ElementIsContained(element_ids_, tip)) {
        continue;
      }
      ik_solver_keys_[base][tip].insert(ik_solver_keys_[base][tip].end(),
                                        solver_keys.begin(), solver_keys.end());
    }
  }

  // TODO(jeanfrancoisd): Update skeleton_base_offset_
  // What configuration should be used to compute the new transform?

  return icon::OkStatus();
}

icon::RealtimeStatusOr<int> Group::GetDofIndexForElementId(
    const ElementId& id) const {
  const auto it = element_id_to_dof_index_.find(id);
  if (it == element_id_to_dof_index_.end()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Provided element id doesn't exist in the group or not a dof. Id: ",
        id.value()));
  }
  return it->second;
}

icon::RealtimeStatusOr<ElementId> Group::GetElementIdForDofIndex(
    int dof_index) const {
  if (dof_index >= dof_index_to_joint_id_.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Provided dof index does not exist. Index: ", dof_index));
  }
  return dof_index_to_joint_id_[dof_index];
}

icon::RealtimeStatusOr<const Element*> Group::GetElement(
    const ElementId& id) const {
  const auto it = element_id_to_element_.find(id);
  if (it == element_id_to_element_.end()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The element with id ", id.value(), " does not exist in the group"));
  }
  return it->second;
}

icon::RealtimeStatusOr<ElementId> Group::GetElementId(
    const Element* element) const {
  for (auto const& [key, val] : element_id_to_element_) {
    if (val == element) {
      return key;
    }
  }
  return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
      "Element ", element->GetName(), " not part of element_map."));
}

}  // namespace kinematics
}  // namespace intrinsic
