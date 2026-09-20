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

#ifndef INTRINSIC_KINEMATICS_MODEL_INTERFACE_DETAILS_H_
#define INTRINSIC_KINEMATICS_MODEL_INTERFACE_DETAILS_H_

#include <algorithm>
#include <map>
#include <memory>
#include <stack>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {
namespace details {

bool operator<(const Joint& lhs, const Joint& rhs);

bool operator<(const Pose3d& ptt1, const Pose3d& ptt2);

bool ElementIsContained(const std::vector<ElementId>& element_ids,
                        const ElementId& id);

bool ElementIsDof(const ModelInterface& model,
                  const std::vector<ElementId>& joint_ids, const ElementId& id);

icon::RealtimeStatusOr<ElementId> GetPreviousDofInTree(
    const ModelInterface& model, const ElementId& element_id);

absl::StatusOr<std::map<ElementId, std::vector<ElementId>>>
UpdateDofChainMapping(const ModelInterface& model);

icon::RealtimeStatusOr<ElementId> GetIdForElement(
    const absl::flat_hash_map<ElementId, std::unique_ptr<Element>>& element_map,
    const Element* element);

icon::RealtimeStatusOr<ElementId> GetIdForElement(
    const absl::flat_hash_map<ElementId, const Element*>& element_map,
    const Element* element);

// Updates the mapping from joints to the dof. Order of the dofs is done
// following the criteria:
// * Deterministic
// * Based solely on the kinematic structure
// * Depth first
// * Human interpretable/authorable
// Which is realized by performing a depth first search on the elements and
// sorting the children of an element with the sorting function described above.
template <typename ElementPtr>
absl::Status UpdateJointDofMapping(
    const ModelInterface& model, const ElementId& root_id,
    const absl::flat_hash_map<ElementId, ElementPtr>& elements,
    const std::vector<ElementId>& joints,
    absl::flat_hash_map<ElementId, int>& id_to_dof_map,
    std::vector<ElementId>& dof_to_joint_id) {
  // Go through with depth first search and sort given the joint comparison
  std::stack<ElementId> element_stack;
  element_stack.push(root_id);
  id_to_dof_map.clear();
  dof_to_joint_id.clear();
  while (!element_stack.empty()) {
    const ElementId current_element = element_stack.top();
    element_stack.pop();
    // Element has not been added to the id_to_dof_map

    if (id_to_dof_map.find(current_element) == id_to_dof_map.end()) {
      const auto it = elements.find(current_element);

      if (it == elements.end()) {
        return ::intrinsic::InvalidArgumentErrorBuilder()
               << "Element with id " << current_element
               << " is not part of the model. Can't update joint dof mapping.";
      }

      if (ElementIsDof(model, joints, current_element)) {
        id_to_dof_map[current_element] = dof_to_joint_id.size();
        dof_to_joint_id.push_back(current_element);
      }
      auto children = it->second->GetChildElements();

      // Sort the children
      std::vector<int> sorted_child_index;
      sorted_child_index.reserve(children.size());

      // In the context of a group, some children might not be in the model. So
      // we filter the child indexes to keep only the one in the model.
      for (int i = 0; i < children.size(); ++i) {
        auto id_or = GetIdForElement(elements, children[i]);
        if (id_or.ok()) {
          sorted_child_index.push_back(i);
        }
      }

      std::sort(
          sorted_child_index.begin(), sorted_child_index.end(),
          [&](int first, int second) -> bool {
            INTRINSIC_RT_ASSIGN_OR_DIE(
                auto id_j1, GetIdForElement(elements, children[first]));
            INTRINSIC_RT_ASSIGN_OR_DIE(
                auto id_j2, GetIdForElement(elements, children[second]));

            bool id1_is_joint = ElementIsContained(joints, id_j1);
            bool id2_is_joint = ElementIsContained(joints, id_j2);

            if (!id1_is_joint && id2_is_joint) {
              return true;
            }
            if (!id2_is_joint && id1_is_joint) {
              return false;
            }
            if (!id1_is_joint && !id2_is_joint) {
              return elements.find(id_j2)->second->GetParentTThis() <
                     elements.find(id_j1)->second->GetParentTThis();
            }

            INTRINSIC_RT_ASSIGN_OR_DIE(const auto* j1, model.GetJoint(id_j1));
            INTRINSIC_RT_ASSIGN_OR_DIE(const auto* j2, model.GetJoint(id_j2));

            return *j2 < *j1;
          });

      for (auto child_id : sorted_child_index) {
        const auto* child = children[child_id];
        if (child != nullptr) {
          INTRINSIC_RT_ASSIGN_OR_DIE(auto child_id,
                                     GetIdForElement(elements, child));
          element_stack.push(child_id);
        }
      }
    }
  }

  if (dof_to_joint_id.size() > eigenmath::MAX_EIGEN_VECTOR_SIZE) {
    return icon::InternalError(icon::RealtimeStatus::StrCat(
        "MAX_EIGEN_VECTOR_SIZE set to ", eigenmath::MAX_EIGEN_VECTOR_SIZE,
        ", but dof= ", dof_to_joint_id.size()));
  }

  return absl::OkStatus();
}

}  // namespace details
}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_MODEL_INTERFACE_DETAILS_H_
