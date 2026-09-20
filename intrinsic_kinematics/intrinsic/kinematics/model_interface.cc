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

#include "intrinsic/kinematics/model_interface.h"

#include <algorithm>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/node_hash_map.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/invert_lower_triangular_matrix.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/model_interface_details.h"
#include "intrinsic/kinematics/state_values.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/to_fixed_string.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {

ModelInterface::ModelInterface(absl::string_view name, const Pose3d& offset)
    : name_(name), skeleton_base_offset_(offset) {}

icon::RealtimeStatus ModelInterface::ClampToSystemLimits(
    JointStateP& joint_values) const {
  JointLimits system_limits = GetDofSystemLimits();
  if (system_limits.max_position.size() != joint_values.position.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size of joint value vector is wrong. System limits size is: ",
        system_limits.min_position.size(),
        " Joint value size is: ", joint_values.size()));
  }
  if (!eigenmath::ClampVector(system_limits.min_position,
                              system_limits.max_position,
                              joint_values.position)) {
    return icon::InternalError("Clamping system position limits failed.");
  }

  return icon::OkStatus();
}

icon::RealtimeStatus ModelInterface::ClampToSoftLimits(
    JointStateP& joint_values) const {
  const JointLimits soft_limits = GetDofSoftPositionLimits();
  if (soft_limits.max_position.size() != joint_values.position.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size of joint value vector is wrong. Soft limits size is: ",
        soft_limits.min_position.size(),
        " Joint value size is: ", joint_values.size()));
  }
  if (!eigenmath::ClampVector(soft_limits.min_position,
                              soft_limits.max_position,
                              joint_values.position)) {
    return icon::InternalError("Clamping soft position limits failed");
  }

  return icon::OkStatus();
}

absl::string_view ModelInterface::GetName() const { return name_; }

JointLimits ModelInterface::GetDofSoftPositionLimits() const {
  JointLimits soft_limits;
  auto status = soft_limits.SetSize(GetNumberDegreesOfFreedom());
  DCHECK_EQ(status, icon::OkStatus());
  for (int dof = 0; dof < GetNumberDegreesOfFreedom(); ++dof) {
    INTRINSIC_RT_ASSIGN_OR_DIE(auto element_id, GetElementIdForDofIndex(dof));
    INTRINSIC_RT_ASSIGN_OR_DIE(const auto* element, GetElement(element_id));
    const auto* joint = static_cast<const Joint*>(element);
    const auto soft_limits_joint = joint->GetSoftLimits();
    soft_limits.min_position[dof] = soft_limits_joint.position.lower;
    soft_limits.max_position[dof] = soft_limits_joint.position.upper;
    soft_limits.max_velocity[dof] = soft_limits_joint.velocity;
    soft_limits.max_acceleration[dof] = soft_limits_joint.acceleration;
    soft_limits.max_jerk[dof] = soft_limits_joint.jerk;
    soft_limits.max_torque[dof] = soft_limits_joint.effort;
  }
  return soft_limits;
}

JointLimits ModelInterface::GetDofSystemLimits() const {
  JointLimits system_limits;
  auto status = system_limits.SetSize(GetNumberDegreesOfFreedom());
  DCHECK_EQ(status, icon::OkStatus());
  for (int dof = 0; dof < GetNumberDegreesOfFreedom(); ++dof) {
    INTRINSIC_RT_ASSIGN_OR_DIE(auto element_id, GetElementIdForDofIndex(dof));
    INTRINSIC_RT_ASSIGN_OR_DIE(const auto* element, GetElement(element_id));
    const auto* joint = static_cast<const Joint*>(element);
    const auto system_limits_joint = joint->GetSystemLimits();
    system_limits.min_position[dof] = system_limits_joint.position.lower;
    system_limits.max_position[dof] = system_limits_joint.position.upper;
    system_limits.max_velocity[dof] = system_limits_joint.velocity;
    system_limits.max_acceleration[dof] = system_limits_joint.acceleration;
    system_limits.max_jerk[dof] = system_limits_joint.jerk;
    system_limits.max_torque[dof] = system_limits_joint.effort;
  }

  return system_limits;
}

icon::RealtimeStatusOr<ElementId> ModelInterface::GetCommonAncestor(
    absl::Span<const ElementId> element_ids) const {
  auto common_ancestor_id = element_ids.front();
  for (const auto& element_id : element_ids) {
    if (element_id == common_ancestor_id) continue;
    const auto common_ancestor_id_or =
        GetCommonAncestor(common_ancestor_id, element_id);

    if (!common_ancestor_id_or.ok()) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "Element ", GetElementName(element_id), " id=", element_id.value(),
          " is not connected to previously established common anscestor ",
          GetElementName(common_ancestor_id),
          " id=", common_ancestor_id.value(), ":",
          common_ancestor_id_or.status().message()));
    }
    common_ancestor_id = common_ancestor_id_or.value();
  }
  return common_ancestor_id;
}

icon::RealtimeStatusOr<ElementId> ModelInterface::GetCommonAncestor(
    const ElementId& element_a, const ElementId& element_b) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* element_a_ptr,
                                GetElement(element_a));
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto element_b_ptr, GetElement(element_b));

  // trivial cases
  if (element_a == base_element_id_) {
    return element_a;
  }
  if (element_b == base_element_id_) {
    return element_b;
  }

  if (element_a == element_b) {
    return element_a;
  }

  // Note the following algorithm searches for a common ancestor without memory
  // allocation due to real time requirements.

  // The most common case is that the skeleton represents a kinematic chain,
  // therefore we first handle the kinematic chain case.
  auto parent = element_a_ptr->GetParentElement();
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto root_ptr, GetElement(base_element_id_));
  while (parent.has_value() && parent.value() != root_ptr) {
    if (parent.value() == element_b_ptr) {
      return element_b;
    }
    parent = parent.value()->GetParentElement();
  }
  parent = element_b_ptr->GetParentElement();
  while (parent.has_value() && parent.value() != root_ptr) {
    if (parent.value() == element_a_ptr) {
      return element_a;
    }
    parent = parent.value()->GetParentElement();
  }

  // Double loop due to realtime requirements that do not allow for dynamic
  // memory allocation.
  auto ancestor_a = element_a_ptr->GetParentElement();
  const int max_iterations = element_ids_.size() * element_ids_.size();
  int iteration = 0;
  while (ancestor_a.has_value()) {
    parent = element_b_ptr->GetParentElement();
    while (parent.has_value()) {
      if (parent.value() == ancestor_a.value()) {
        return GetElementId(parent.value());
      }
      parent = parent.value()->GetParentElement();
      if (++iteration > max_iterations) {
        return icon::InternalError(icon::RealtimeStatus::StrCat(
            "Reached max iterations: ", max_iterations,
            " for the two elements ", element_a_ptr->GetName(), " and ",
            element_b_ptr->GetName()));
      }
    }
    ancestor_a = ancestor_a.value()->GetParentElement();
  }

  // We should not get to this point.
  return icon::InternalError(icon::RealtimeStatus::StrCat(
      "An error occurred while searching for the common ancestor for the two "
      "elements: ",
      element_a_ptr->GetName(), " and ", element_b_ptr->GetName()));
}

// TODO(kmuelling): Validate the use of CHECK in this function.
JointStateP ModelInterface::GetDefaultConfiguration() const {
  JointStateP default_config;
  auto status = default_config.SetSize(GetNumberDegreesOfFreedom());
  DCHECK_EQ(status, icon::OkStatus());
  for (int dof = 0; dof < GetNumberDegreesOfFreedom(); ++dof) {
    INTRINSIC_RT_ASSIGN_OR_DIE(auto element_id, GetElementIdForDofIndex(dof));
    INTRINSIC_RT_ASSIGN_OR_DIE(const auto* element, GetElement(element_id));
    const auto* joint = static_cast<const Joint*>(element);
    default_config.position[dof] = joint->GetDefaultConfiguration();
  }
  return default_config;
}

icon::RealtimeStatusOr<StateValues> ModelInterface::GetDefaultStateValues()
    const {
  // All except position limits are the system limits
  auto state_limits = GetDofSystemLimits();
  auto position_soft_limits = GetDofSoftPositionLimits();
  state_limits.max_position = position_soft_limits.max_position;
  state_limits.min_position = position_soft_limits.min_position;

  if (!state_limits.IsValid()) {
    // This should never happen since we already checked the validity of the
    // joint parameters. Kept here in case something is really wrong.
    LOG(FATAL) << "Limits are invalid: "
               << absl::string_view(ToFixedString(state_limits));
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const eigenmath::MatrixNd joint_dependency_matrix,
      GetJointDependencyMatrix());

  return StateValues{GetDefaultConfiguration(), state_limits,
                     joint_dependency_matrix};
}

icon::RealtimeStatusOr<ElementId> ModelInterface::FindElementIdByName(
    const absl::string_view name) const {
  ElementId element_id;
  bool found = false;
  for (const auto& id : GetAllElementIds()) {
    INTRINSIC_RT_ASSIGN_OR_DIE(const auto* element, GetElement(id));
    DCHECK_NE(element, nullptr);
    if (element->GetName() == name) {
      if (found) {
        // Found more than one element with this name.
        return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
            "Model contains multiple instances of elements with the name ",
            name, "."));
      }
      element_id = id;
      found = true;
    }
  }
  if (found) {
    return element_id;
  }
  return icon::NotFoundError(icon::RealtimeStatus::StrCat(
      "Model does not contain element with name ", name));
}

std::vector<ElementId> ModelInterface::FindElementIdByNameContains(
    const absl::string_view name) const {
  std::vector<ElementId> result;
  for (const auto& id : GetAllElementIds()) {
    INTRINSIC_RT_ASSIGN_OR_DIE(const auto* element, GetElement(id));
    if (absl::StrContains(element->GetName(), name)) {
      result.push_back(id);
    }
  }
  return result;
}

std::string ModelInterface::GetElementName(const ElementId& id) const {
  auto element_or = GetElement(id);
  if (element_or.ok()) {
    return std::string(element_or.value()->GetName());
  }
  return absl::StrCat("InvalidId(", id.value(), ")");
}

icon::RealtimeStatusOr<const Joint*> ModelInterface::GetJoint(
    ElementId id) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* element, GetElement(id));

  // Make sure element id is joint
  const auto joint_it = std::find(joints_.begin(), joints_.end(), id);
  if (joint_it == joints_.end()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Requested element is not a joint. Id: ", id.value()));
  }
  return static_cast<const Joint*>(element);
}

icon::RealtimeStatusOr<const Link*> ModelInterface::GetLink(
    ElementId id) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* element, GetElement(id));

  // Make sure element id is link
  auto link_it = std::find(links_.begin(), links_.end(), id);
  if (link_it == links_.end()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Requested element is not a link. Id: ", id.value()));
  }
  return static_cast<const Link*>(element);
}

absl::Span<const ElementId> ModelInterface::GetAllElementIds() const {
  return element_ids_;
}

absl::Span<const ElementId> ModelInterface::GetDofIds() const {
  return dof_index_to_joint_id_;
}

absl::Span<const ElementId> ModelInterface::GetAllJointIds() const {
  return joints_;
}

absl::Span<const ElementId> ModelInterface::GetAllLinkIds() const {
  return links_;
}

absl::Span<const ElementId> ModelInterface::GetAllCoordinateFrameIds() const {
  return coordinate_frames_;
}

absl::Span<const std::string> ModelInterface::GetAllSolverKeys(
    const ElementId& start_frame, const ElementId& end_frame) const {
  const auto start_frame_it = ik_solver_keys_.find(start_frame);
  if (start_frame_it != ik_solver_keys_.end()) {
    const auto& start_frame_map = start_frame_it->second;
    const auto end_frame_it = start_frame_map.find(end_frame);
    if (end_frame_it != start_frame_map.end()) {
      return end_frame_it->second;
    }
  }
  return {};
}

absl::Status ModelInterface::SetSolverKey(const ElementId& start_frame,
                                          const ElementId& end_frame,
                                          absl::string_view solver_key) {
  // Check if elements exist and if they are connected through common ancestor
  INTR_RETURN_IF_ERROR(GetCommonAncestor(start_frame, end_frame).status());

  if (start_frame == end_frame) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Start and end element id need to differ.";
  }
  ik_solver_keys_[start_frame][end_frame].emplace_back(solver_key);
  return absl::OkStatus();
}

absl::Span<const ElementId> ModelInterface::GetTipIds() const {
  return tip_ids_;
}

ElementId ModelInterface::GetBaseId() const { return base_element_id_; }

icon::RealtimeStatusOr<absl::Span<const ElementId>>
ModelInterface::GetDofChainForTip(const ElementId& tip_element_id) const {
  auto chain_iter = tip_dof_chains_.find(tip_element_id);
  if (chain_iter == tip_dof_chains_.end()) {
    return icon::NotFoundError(icon::RealtimeStatus::StrCat(
        "Skeleton does not contain a chain for tip element ",
        GetElementName(tip_element_id), " with id ", tip_element_id.value()));
  }
  return absl::Span<const ElementId>(chain_iter->second);
}

icon::RealtimeStatusOr<
    FixedVector<ElementId, eigenmath::VectorNd::MaxSizeAtCompileTime>>
ModelInterface::GetDofChainForElement(const ElementId& element_id) const {
  FixedVector<ElementId, eigenmath::VectorNd::MaxSizeAtCompileTime>
      dof_id_chain;

  // Return early if element is a tip. The DoF chain is already stored in the
  // model and available via GetDofChainForTip.
  const icon::RealtimeStatusOr<absl::Span<const ElementId>> dof_ids_or =
      GetDofChainForTip(element_id);
  if (dof_ids_or.ok()) {
    dof_id_chain.insert(dof_id_chain.end(), dof_ids_or.value().begin(),
                        dof_ids_or.value().end());
    return dof_id_chain;
  }

  // If the element is not a tip, we need to build the chain of DoF ids from the
  // base to the element.
  ElementId current_id = element_id;

  // Handle the case in which the element is a dof itself.
  if (const icon::RealtimeStatusOr<const Joint*> joint_or =
          GetJoint(current_id);
      joint_or.ok() && joint_or.value()->IsDof()) {
    dof_id_chain.push_back(current_id);
  }
  // Proceed from the element to the base to identify DoF in the chain. When we
  // reach the base, the previous DoF in the three will be an invalid element.
  while (current_id != kInvalidElementId) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        current_id, details::GetPreviousDofInTree(*this, current_id));
    if (current_id != kInvalidElementId) {
      dof_id_chain.push_back(current_id);
    }
  }
  std::reverse(dof_id_chain.begin(), dof_id_chain.end());

  return dof_id_chain;
}

int ModelInterface::GetNumberDegreesOfFreedom() const {
  return dof_index_to_joint_id_.size();
}

bool ModelInterface::HasOneTip() const { return (tip_ids_.size() == 1); }

bool ModelInterface::IsNonBranchingKinematicChain() const {
  // Catch the simple cases of a one-tip chain.
  // Note: `FindNonBranchingKinematicChainTip()` cannot take this shortcut,
  // because it should only return joint or link elements. Even if the model has
  // only a single tip, that tip might be a frame!
  if (tip_ids_.size() == 1) {
    return true;
  }

  return FindNonBranchingKinematicChainTip().ok();
}

icon::RealtimeStatusOr<kinematics::ElementId>
ModelInterface::FindNonBranchingKinematicChainTip() const {
  // Walk the kinematics model, starting from the base element.
  //
  // We do not consider frames here, only links and joints.
  kinematics::ElementId current_element_id = GetBaseId();
  INTRINSIC_RT_ASSIGN_OR_RETURN(const kinematics::Element* current_element_ptr,
                                GetElement(current_element_id));
  auto child_elements = current_element_ptr->GetChildElements();

  while (!child_elements.empty()) {
    bool found_non_frame_child = false;
    kinematics::ElementId child_id;

    // Is there more than one non-frame child?
    for (auto it = child_elements.begin(); it < child_elements.end(); ++it) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(child_id, GetElementId(*it));

      if (GetJoint(child_id).ok() || GetLink(child_id).ok()) {
        if (found_non_frame_child) {
          return icon::FailedPreconditionError(icon::RealtimeStatus::StrCat(
              "Model ", GetName(), " is not a chain. Link '",
              current_element_ptr->GetName(),
              "' has multiple children that aren't frames."));
        }
        // Update `current_element_id` to continue looping over the chain.
        // *Don't* update `current_element_ptr` yet! We want to keep that in
        // case we find a second non-frame child, so we can report the element
        // name in the error message.
        current_element_id = child_id;
        found_non_frame_child = true;
      }
    }

    // If all children were frames we can exit as frames cannot have children.
    if (!found_non_frame_child) {
      break;
    }

    // *Now* we can update `current_element_ptr`
    INTRINSIC_RT_ASSIGN_OR_RETURN(current_element_ptr,
                                  GetElement(current_element_id));
    // ... and `child_elements`, to keep exploring the kinematic chain.
    child_elements = current_element_ptr->GetChildElements();
  }
  return current_element_id;
}

Pose3d ModelInterface::GetAnchorPointOffset() const {
  return skeleton_base_offset_;
}

icon::RealtimeStatusOr<double> ModelInterface::ComputeJointsDerivedValue(
    const ElementId& joint_id, const JointStateP& dof_input_values) const {
  // Make sure element id is joint
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Joint* joint, GetJoint(joint_id));

  // Special case fixed joint
  // TODO(b/187084540): As we add the feature to treat joints more general, this
  // line should become part of the joint value computation.
  if (joint->GetType() == Joint::FIXED) {
    return 0.0;
  }

  // Joint is dof.
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto dof_index,
                                GetDofIndexForElementId(joint_id));

  // Special case non-dependent joint with trivial return value.
  if (!joint->IsDependent()) {
    return dof_input_values.position[dof_index];
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const Joint::LinearDependency* linear_dependency,
      joint->GetLinearDependency());

  // Compute the derived value of the joint, evaluate the linear weighted
  // combination of the input values of the joint and its leading joints.
  double dof_derived_value =
      linear_dependency->alpha_self * dof_input_values.position[dof_index];
  for (const auto& [leading_joint_id, alpha_leading] :
       linear_dependency->alpha_leading) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const int dof_index,
                                  GetDofIndexForElementId(leading_joint_id));
    dof_derived_value += alpha_leading * dof_input_values.position[dof_index];
  }

  return dof_derived_value;
}

icon::RealtimeStatus IsAncestorRigidlyAttached(const ModelInterface& model,
                                               const ElementId& child,
                                               const ElementId& ancestor) {
  // Return if they are the same element.
  if (child == ancestor) {
    return icon::OkStatus();
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const Element* child_element,
                                model.GetElement(child));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Element* ancestor_element,
                                model.GetElement(ancestor));

  // Validate elements do not have any movable joints between them.
  std::optional<const Element*> current = child_element->GetParentElement();
  while (current.has_value() && *current != nullptr) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(ElementId current_id,
                                  model.GetElementId(*current));
    if (current_id == ancestor) {
      return icon::OkStatus();
    }
    // If there is a DoF index for `current_id` in `model`, then
    // `current_element` is a (non-fixed) joint.
    if (model.GetDofIndexForElementId(current_id).ok()) {
      return icon::FailedPreconditionError(icon::RealtimeStatus::StrCat(
          "Joint '", (*current)->GetName(), "' is movable."));
    }
    current = (*current)->GetParentElement();
  }
  return icon::FailedPreconditionError(icon::RealtimeStatus::StrCat(
      "Expected element '", child_element->GetName(), "' to be a child of '",
      ancestor_element->GetName(), "'."));
}

icon::RealtimeStatus AreElementsRigidlyAttached(const ModelInterface& model,
                                                const ElementId& e1,
                                                const ElementId& e2) {
  // Algorithm: Find the least common ancestor of the two elements, then
  // separately check that the connection between each element and that ancestor
  // is rigid.
  INTRINSIC_RT_ASSIGN_OR_RETURN(ElementId ancestor,
                                model.GetCommonAncestor(e1, e2));
  // As a subtle point, note that if ancestor is itself a movable joint, we
  // consider its outboard frame. This means that two objects rigidly attached
  // to a movable joint are rigidly attached to each other.
  INTRINSIC_RT_RETURN_IF_ERROR(IsAncestorRigidlyAttached(model, e1, ancestor));
  return IsAncestorRigidlyAttached(model, e2, ancestor);
}

icon::RealtimeStatusOr<eigenmath::MatrixNd>
ModelInterface::GetJointDependencyMatrix() const {
  const int ndof = GetNumberDegreesOfFreedom();

  // Initialize the joint dependency matrix to the identity matrix. Elements
  // will be altered in place.
  eigenmath::MatrixNd joint_dependency_matrix =
      eigenmath::MatrixNd::Identity(ndof, ndof);

  for (int i = 0; i < GetNumberDegreesOfFreedom(); ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const ElementId joint_id,
                                  GetElementIdForDofIndex(i));
    INTRINSIC_RT_ASSIGN_OR_RETURN(const Joint* joint, GetJoint(joint_id));
    if (!joint->IsDependent()) {
      continue;
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const Joint::LinearDependency* linear_dependency,
        joint->GetLinearDependency());
    joint_dependency_matrix(i, i) = linear_dependency->alpha_self;
    for (const auto& [leading_joint_id, alpha] :
         linear_dependency->alpha_leading) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(const int dof_index,
                                    GetDofIndexForElementId(leading_joint_id));
      joint_dependency_matrix(i, dof_index) = alpha;
    }
  }

  if (!joint_dependency_matrix.isLowerTriangular()) {
    return icon::FailedPreconditionError(
        "Joint dependency matrix is not lower triangular. There is an invalid "
        "joint dependency.");
  }

  return joint_dependency_matrix;
}

icon::RealtimeStatusOr<eigenmath::MatrixNd>
ModelInterface::GetJointDependencyMatrixInverse() const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const eigenmath::MatrixNd joint_dependency_matrix,
      GetJointDependencyMatrix());
  return InvertLowerTriangularMatrix(joint_dependency_matrix);
}

icon::RealtimeStatusOr<bool> ModelInterface::HasDependentJoints() const {
  for (const auto& joint_id : joints_) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint, GetJoint(joint_id));
    if (joint->IsDependent()) {
      return true;
    }
  }
  return false;
}

absl::Status ModelInterface::ValidateLinearDependency(
    const ElementId joint_id) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Joint* joint, GetJoint(joint_id));
  if (!joint->IsDependent()) {
    return absl::OkStatus();
  }

  INTR_ASSIGN_OR_RETURN(const auto* linear_dependency,
                        joint->GetLinearDependency());

  for (const auto& [leading_joint_id, alpha] :
       linear_dependency->alpha_leading) {
    if (leading_joint_id == joint_id) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Element id ", leading_joint_id.value(),
          ", which is specified in a linear dependency is the same as the "
          "joint itself. A joint cannot depend on itself."));
    }

    // Check that the leading joint is actually preceding the dependent joint in
    // the kinematic chain. Leading joints are not allowed to be children of the
    // dependent joint.
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        bool leading_joint_is_ancestor_of_this_joint,
        IsAncestorOf(/*child=*/joint_id,
                     /*ancestor_to_check=*/leading_joint_id));
    if (!leading_joint_is_ancestor_of_this_joint) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Element id ", leading_joint_id.value(),
          ", which is specified in a linear dependency is not an ancestor "
          "of the dependent joint with id ",
          joint_id.value(), "."));
    }

    // Check if the leading joint is a DoF.
    INTRINSIC_RT_ASSIGN_OR_RETURN(const Joint* leading_joint,
                                  GetJoint(leading_joint_id));
    if (!leading_joint->IsDof()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Element with id ", leading_joint_id.value(),
                       ", which is specified as a leading joint in a linear "
                       "dependency, is not a DoF."));
    }
    // Check that the leading joint is not a dependent joint by itself.
    // TODO(b/425567348): we don't support chains of dependencies yet.
    // As of today, dependencies need to be manually resolved wrt lowest
    // possible leading joint.
    if (leading_joint->IsDependent()) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Element id ", leading_joint_id.value(),
          ", which is specified in a linear dependency is a "
          "dependent joint itself. Currently, dependent joints are not "
          "allowed to be dependent joints themselves."));
    }
  }
  return absl::OkStatus();
}

icon::RealtimeStatusOr<bool> ModelInterface::IsAncestorOf(
    const ElementId child, const ElementId ancestor_to_check) const {
  if (child == kInvalidElementId || ancestor_to_check == kInvalidElementId) {
    return icon::InvalidArgumentError("Elements must be valid.");
  }
  if (child == ancestor_to_check) {
    return true;
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Element* child_element,
                                GetElement(child));

  std::optional<const Element*> maybe_parent_element =
      child_element->GetParentElement();
  if (!maybe_parent_element.has_value()) {
    return false;
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const ElementId parent_element_id,
                                GetElementId(*maybe_parent_element));
  return IsAncestorOf(parent_element_id, ancestor_to_check);
}

}  // namespace kinematics
}  // namespace intrinsic
