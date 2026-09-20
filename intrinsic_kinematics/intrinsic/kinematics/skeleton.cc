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

#include "intrinsic/kinematics/skeleton.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/coordinate_frame.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/model_interface_details.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {

using details::ElementIsContained;

Skeleton::Skeleton(absl::string_view name, const Pose3d& offset)
    : ModelInterface(name, offset) {}

std::unique_ptr<Skeleton> Skeleton::Clone() const {
  auto twin = std::make_unique<Skeleton>(name_, skeleton_base_offset_);

  std::stack<ElementId> element_stack;
  element_stack.push(base_element_id_);
  while (!element_stack.empty()) {
    const ElementId current_element_id = element_stack.top();
    element_stack.pop();

    // We assign or die in this function since unless the code is corrupted, all
    // the ids are guaranteed to be valid.
    INTRINSIC_RT_ASSIGN_OR_DIE(auto current_element,
                               GetElement(current_element_id));

    for (const auto* child : current_element->GetChildElements()) {
      INTRINSIC_RT_ASSIGN_OR_DIE(auto child_id, GetElementId(child));
      element_stack.push(child_id);
    }

    ElementId parent_id = kInvalidElementId;
    auto parent_element_or = current_element->GetParentElement();
    if (parent_element_or != std::nullopt) {
      INTRINSIC_RT_ASSIGN_OR_DIE(parent_id, GetElementId(*parent_element_or));
    }

    if (ElementIsContained(joints_, current_element_id)) {
      INTRINSIC_RT_ASSIGN_OR_DIE(const Joint* joint,
                                 GetJoint(current_element_id));

      std::optional<Joint::LinearDependency> optional_linear_dependency =
          std::nullopt;
      if (joint->IsDependent()) {
        INTRINSIC_RT_ASSIGN_OR_DIE(
            const Joint::LinearDependency* linear_dependency,
            joint->GetLinearDependency());
        optional_linear_dependency = *linear_dependency;
      }
      auto maybe_new_joint = Joint::Create(
          joint->GetName(), joint->GetParameters(), optional_linear_dependency);
      CHECK_OK(maybe_new_joint);
      CHECK_OK(twin->AddJointWithId(std::move(maybe_new_joint.value()),
                                    current_element_id, parent_id,
                                    joint->GetParentTThis()));
      continue;
    }

    if (ElementIsContained(links_, current_element_id)) {
      INTRINSIC_RT_ASSIGN_OR_DIE(const Link* link, GetLink(current_element_id));
      CHECK_OK(twin->CreateLinkWithId(link->GetName(), link->GetParameters(),
                                      current_element_id, parent_id,
                                      link->GetParentTThis()));
      continue;
    }

    CHECK(ElementIsContained(coordinate_frames_, current_element_id))
        << "Element " << current_element->GetName()
        << " should be a joint, a link or a coordinate frame.";
    CHECK_OK(twin->CreateCoordinateFrameWithId(
        current_element->GetName(), current_element_id, parent_id,
        current_element->GetParentTThis()));
  }
  // Since `twin` has all the same elements, with the same IDs, as `this`, we
  // can just copy the map of solver keys unchanged.
  twin->ik_solver_keys_ = this->ik_solver_keys_;

  return twin;
}

ElementId Skeleton::GenerateElementId() {
  while (element_id_to_element_.find(++last_id_) !=
         element_id_to_element_.end()) {
  }
  return last_id_;
}

absl::Status Skeleton::AddElement(std::unique_ptr<Element> element,
                                  const ElementId& new_element_id,
                                  const ElementId& parent_id,
                                  const Pose3d& parent_t_this) {
  // Check if the element is valid
  if (element == nullptr) {
    return absl::InvalidArgumentError("Attempted to add non-existing element.");
  }

  // Check if element_id is already in the list
  const auto element_it = element_id_to_element_.find(new_element_id);
  if (element_it != element_id_to_element_.end()) {
    LOG(ERROR) << "All skeleton element: "
               << absl::StrJoin(element_id_to_element_, ",",
                                [](std::string* str, const auto& pair) {
                                  absl::StrAppend(str, pair.first.value());
                                });
    return absl::InvalidArgumentError(
        absl::StrCat("The element with id ", new_element_id.value(),
                     " already exist in the skeleton"));
  }

  // We currently only allow a tree structure with one root element. Check if
  // user attempts to insert second root element.
  if (parent_id == kInvalidElementId && !element_id_to_element_.empty()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "No parent id for non-root element " << element->GetName()
           << " provided. Root already exist.";
  }

  // Make sure element has no children defined.
  if (element->GetNumberOfChildren() != 0) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Element " << element->GetName()
           << " has children defined that are not part of the skeleton. The "
              "construction of a skeleton is constraint from root to leave.";
  }

  // Check parent element
  const auto& parent_it = element_id_to_element_.find(parent_id);
  if (parent_id != kInvalidElementId &&
      parent_it == element_id_to_element_.end()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Provided parent id (" << parent_id << ") for element "
           << element->GetName() << " does not exist in the Skeleton.";
  }

  // Connect child and parent elements.
  if (parent_id != kInvalidElementId) {
    Element* parent_element = parent_it->second.get();
    INTR_RETURN_IF_ERROR(
        element->SetParentElement(parent_element, parent_t_this));
    INTR_RETURN_IF_ERROR(parent_element->AddChildElement(element.get()));
  }

  // Add element to set of elements
  element_id_to_element_[new_element_id] = std::move(element);
  element_ids_.push_back(new_element_id);

  // Update tip ids. By definition, each new element is a tip until it becomes
  // a parent element of a new element.
  // NOTE: This loop creates separate tips even for frames that are attached
  // to some link in the chain, even though the links and joints may form a
  // single chain. Thus, the number of tips cannot be used to diagnose a
  // branching kinematic chain.
  auto it = std::find(tip_ids_.begin(), tip_ids_.end(), parent_id);
  if (it != tip_ids_.end()) {
    *it = new_element_id;
  } else {
    tip_ids_.push_back(new_element_id);
  }

  // If component was root element, save shortcut
  if (parent_id == kInvalidElementId) {
    base_element_id_ = new_element_id;
  }

  // We do update the dof chains here so that they can be retrieved in
  // real-time.
  INTR_ASSIGN_OR_RETURN(tip_dof_chains_, details::UpdateDofChainMapping(*this));

  return absl::OkStatus();
}

absl::StatusOr<ElementId> Skeleton::AddJoint(std::unique_ptr<Joint> element,
                                             const ElementId& parent_id,
                                             const Pose3d& parent_t_this) {
  // Generate new id for element
  ElementId joint_id = GenerateElementId();
  INTR_RETURN_IF_ERROR(
      AddJointWithId(std::move(element), joint_id, parent_id, parent_t_this));
  return joint_id;
}

absl::Status Skeleton::AddJointWithId(std::unique_ptr<Joint> element,
                                      const ElementId& new_element_id,
                                      const ElementId& parent_id,
                                      const Pose3d& parent_t_this) {
  // Add joint
  const bool fixed_joint = (element->GetType() == Joint::FIXED);
  // Check if the joint is valid in terms of parameters and (optional)
  // linear dependency.
  INTR_RETURN_IF_ERROR(element->CheckJointParameters());

  INTR_RETURN_IF_ERROR(
      AddElement(std::move(element), new_element_id, parent_id, parent_t_this));
  // Add to list of joints
  joints_.push_back(new_element_id);

  INTR_RETURN_IF_ERROR(ValidateLinearDependency(new_element_id));

  // Update dof joint mapping
  if (fixed_joint) {
    return absl::OkStatus();
  }

  INTR_RETURN_IF_ERROR(details::UpdateJointDofMapping(
      *this, base_element_id_, element_id_to_element_, joints_,
      element_id_to_dof_index_, dof_index_to_joint_id_));

  return absl::OkStatus();
}

absl::StatusOr<ElementId> Skeleton::AddLink(std::unique_ptr<Link> element,
                                            const ElementId& parent_id,
                                            const Pose3d& parent_t_this) {
  ElementId link_id = GenerateElementId();
  INTR_RETURN_IF_ERROR(
      AddLinkWithId(std::move(element), link_id, parent_id, parent_t_this));
  return link_id;
}

absl::Status Skeleton::AddLinkWithId(std::unique_ptr<Link> element,
                                     const ElementId& new_element_id,
                                     const ElementId& parent_id,
                                     const Pose3d& parent_t_this) {
  INTR_RETURN_IF_ERROR(
      AddElement(std::move(element), new_element_id, parent_id, parent_t_this));

  // Add to list of links
  links_.push_back(new_element_id);

  return absl::OkStatus();
}

absl::StatusOr<ElementId> Skeleton::AddCoordinateFrame(
    std::unique_ptr<CoordinateFrame> element, const ElementId& parent_id,
    const Pose3d& parent_t_this) {
  ElementId cf_id = GenerateElementId();
  INTR_RETURN_IF_ERROR(AddCoordinateFrameWithId(std::move(element), cf_id,
                                                parent_id, parent_t_this));
  return cf_id;
}

absl::Status Skeleton::AddCoordinateFrameWithId(
    std::unique_ptr<CoordinateFrame> element, const ElementId& new_element_id,
    const ElementId& parent_id, const Pose3d& parent_t_this) {
  INTR_RETURN_IF_ERROR(
      AddElement(std::move(element), new_element_id, parent_id, parent_t_this));

  // Add to list of cf
  coordinate_frames_.push_back(new_element_id);

  return absl::OkStatus();
}

absl::StatusOr<ElementId> Skeleton::CreateJoint(
    absl::string_view name, const Joint::Parameters& parameters,
    const ElementId& parent_id, const Pose3d& parent_t_joint) {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<Joint> joint_element,
                        Joint::Create(name, parameters));
  return AddJoint(std::move(joint_element), parent_id, parent_t_joint);
}

absl::StatusOr<ElementId> Skeleton::CreateJoint(
    absl::string_view name, const Joint::Parameters& parameters,
    const Joint::LinearDependency& linear_dependency,
    const ElementId& parent_id, const Pose3d& parent_t_joint) {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<Joint> joint_element,
                        Joint::Create(name, parameters, linear_dependency));
  return AddJoint(std::move(joint_element), parent_id, parent_t_joint);
}

absl::Status Skeleton::CreateJointWithId(absl::string_view name,
                                         const Joint::Parameters& parameters,
                                         const ElementId& new_element_id,
                                         const ElementId& parent_id,
                                         const Pose3d& parent_t_joint) {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<Joint> joint_element,
                        Joint::Create(name, parameters));
  INTR_RETURN_IF_ERROR(AddJointWithId(std::move(joint_element), new_element_id,
                                      parent_id, parent_t_joint));
  return absl::OkStatus();
}

absl::StatusOr<ElementId> Skeleton::CreateLink(
    absl::string_view name, const Link::Parameters& parameters,
    const ElementId& parent, const Pose3d& parent_t_link) {
  auto link_element = std::make_unique<Link>(name, parameters);
  return AddLink(std::move(link_element), parent, parent_t_link);
}

absl::Status Skeleton::CreateLinkWithId(absl::string_view name,
                                        const Link::Parameters& parameters,
                                        const ElementId& new_element_id,
                                        const ElementId& parent,
                                        const Pose3d& parent_t_link) {
  auto link_element = std::make_unique<Link>(name, parameters);
  INTR_RETURN_IF_ERROR(AddLinkWithId(std::move(link_element), new_element_id,
                                     parent, parent_t_link));
  return absl::OkStatus();
}

absl::StatusOr<ElementId> Skeleton::CreateCoordinateFrame(
    absl::string_view name, const ElementId& parent_id,
    const Pose3d& parent_t_cf) {
  auto cf_element = std::make_unique<CoordinateFrame>(name);
  return AddCoordinateFrame(std::move(cf_element), parent_id, parent_t_cf);
}

absl::Status Skeleton::CreateCoordinateFrameWithId(
    absl::string_view name, const ElementId& new_element_id,
    const ElementId& parent_id, const Pose3d& parent_t_cf) {
  auto cf_element = std::make_unique<CoordinateFrame>(name);
  INTR_RETURN_IF_ERROR(AddCoordinateFrameWithId(
      std::move(cf_element), new_element_id, parent_id, parent_t_cf));
  return absl::OkStatus();
}

icon::RealtimeStatusOr<int> Skeleton::GetDofIndexForElementId(
    const ElementId& id) const {
  const auto it = element_id_to_dof_index_.find(id);
  if (it == element_id_to_dof_index_.end()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Provided element id doesn't exist in the skeleton or not a dof. "
        "Id: ",
        id.value()));
  }
  return it->second;
}

icon::RealtimeStatusOr<ElementId> Skeleton::GetElementIdForDofIndex(
    int dof_index) const {
  if (dof_index >= dof_index_to_joint_id_.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Provided dof index does not exist. Index: ", dof_index));
  }
  return dof_index_to_joint_id_[dof_index];
}

icon::RealtimeStatusOr<const Element*> Skeleton::GetElement(
    const ElementId& id) const {
  const auto it = element_id_to_element_.find(id);
  if (it == element_id_to_element_.end()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The element with id ", id.value(), " does not exist in the skeleton"));
  }
  return it->second.get();
}

icon::RealtimeStatusOr<ElementId> Skeleton::GetElementId(
    const Element* element) const {
  for (auto const& [key, val] : element_id_to_element_) {
    if (val.get() == element) {
      return key;
    }
  }
  return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
      "Element ", element->GetName(), " not part of element_map."));
}

}  // namespace kinematics
}  // namespace intrinsic
