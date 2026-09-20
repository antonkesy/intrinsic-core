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

#include "intrinsic/kinematics/proto/kinematics_conversion.h"

#include <algorithm>
#include <list>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/coordinate_frame.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/proto/kinematics.pb.h"
#include "intrinsic/kinematics/proto/skeleton.pb.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {
namespace details {

namespace {

intrinsic_proto::Joint_Type ToProto(const Joint::Type& type) {
  switch (type) {
    case Joint::REVOLUTE:
      return intrinsic_proto::Joint::REVOLUTE;
    case Joint::PRISMATIC:
      return intrinsic_proto::Joint::PRISMATIC;
    case Joint::FIXED:
      return intrinsic_proto::Joint::FIXED;
  }
}

absl::StatusOr<Joint::Type> FromProto(const intrinsic_proto::Joint_Type& type) {
  switch (type) {
    case intrinsic_proto::Joint::REVOLUTE:
      return Joint::REVOLUTE;
    case intrinsic_proto::Joint::PRISMATIC:
      return Joint::PRISMATIC;
    case intrinsic_proto::Joint::FIXED:
      return Joint::FIXED;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Invalid joint type: ", type));
  }
}

intrinsic_proto::LimitPair ToProto(const Joint::PositionLimits& limits) {
  intrinsic_proto::LimitPair out_limits;
  out_limits.set_lower(limits.lower);
  out_limits.set_upper(limits.upper);
  return out_limits;
}

Joint::PositionLimits FromProto(const intrinsic_proto::LimitPair& limits) {
  Joint::PositionLimits out_limits;
  out_limits.lower = limits.lower();
  out_limits.upper = limits.upper();
  return out_limits;
}

intrinsic_proto::Limits ToProto(const Joint::Limits& limits) {
  intrinsic_proto::Limits out_limits;
  *out_limits.mutable_position() = ToProto(limits.position);
  out_limits.set_velocity(limits.velocity);
  out_limits.set_acceleration(limits.acceleration);
  out_limits.set_jerk(limits.jerk);
  out_limits.set_effort(limits.effort);
  return out_limits;
}

absl::StatusOr<Joint::Limits> FromProto(const intrinsic_proto::Limits& limits) {
  Joint::Limits out_limits;
  out_limits.position = FromProto(limits.position());
  if (limits.velocity() < 0) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Symmetrical velocity limits can't be negative.";
  }
  out_limits.velocity = limits.velocity();
  if (limits.acceleration() < 0) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Symmetrical acceleration limits can't be negative.";
  }
  out_limits.acceleration = limits.acceleration();
  if (limits.jerk() < 0) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Symmetrical jerk limits can't be negative.";
  }
  out_limits.jerk = limits.jerk();
  if (limits.effort() < 0) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Symmetrical effort limits can't be negative.";
  }
  out_limits.effort = limits.effort();
  return out_limits;
}

}  // namespace

intrinsic_proto::Element ToProto(const DeferredElement& element) {
  intrinsic_proto::Element out_proto;
  out_proto.set_id(element.id.value());
  out_proto.set_name(element.outbound_element->GetName());
  *out_proto.mutable_parent_t_this() =
      ToProto(element.outbound_element->GetParentTThis());

  out_proto.set_parent(element.parent_id.value());

  auto* children = out_proto.mutable_children();
  children->Reserve(element.children_ids.size());
  for (const auto& child_id : element.children_ids) {
    *children->Add() = child_id.value();
  }

  return out_proto;
}

absl::StatusOr<DeferredElement> FromProto(
    const intrinsic_proto::Element& element_proto) {
  DeferredElement element;
  element.inbound_element = nullptr;
  element.id = ElementId(element_proto.id());
  element.parent_id = ElementId(element_proto.parent());
  element.children_ids = {element_proto.children().begin(),
                          element_proto.children().end()};
  if (element_proto.has_parent_t_this()) {
    INTR_ASSIGN_OR_RETURN(element.parent_t_this,
                          FromProto(element_proto.parent_t_this()));
  } else {
    element.parent_t_this = Pose3d::Identity();
  }

  return element;
}

absl::StatusOr<intrinsic_proto::Joint> ToProto(
    const details::DeferredJoint& joint_element) {
  QCHECK(joint_element.deferred_element.outbound_element != nullptr);
  const auto* joint = dynamic_cast<const Joint*>(
      joint_element.deferred_element.outbound_element);
  if (joint == nullptr) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Element ", joint_element.deferred_element.outbound_element->GetName(),
        " is not a joint"));
  }
  intrinsic_proto::Joint out_proto;
  *out_proto.mutable_element() = ToProto(joint_element.deferred_element);

  auto* parameters = out_proto.mutable_parameters();
  parameters->set_type(ToProto(joint->GetType()));
  *parameters->mutable_axis() = ::intrinsic::ToProto(joint->GetAxis());
  *parameters->mutable_system_limits() = ToProto(joint->GetSystemLimits());
  *parameters->mutable_soft_limits() = ToProto(joint->GetSoftLimits());
  parameters->set_default_configuration(joint->GetDefaultConfiguration());

  auto* dynamic_parameters = parameters->mutable_dynamics();
  dynamic_parameters->set_static_friction(
      joint->GetParameters().dynamics.static_friction);
  dynamic_parameters->set_damping(joint->GetParameters().dynamics.damping);

  if (joint->IsDependent()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const Joint::LinearDependency* linear_dependency,
        joint->GetLinearDependency());
    *out_proto.mutable_linear_dependency() = ToProto(*linear_dependency);
  }

  return out_proto;
}

absl::StatusOr<details::DeferredJoint> FromProto(
    const intrinsic_proto::Joint& joint_proto) {
  details::DeferredJoint deferred_joint;
  INTR_ASSIGN_OR_RETURN(deferred_joint.deferred_element,
                        details::FromProto(joint_proto.element()));

  Joint::Parameters parameters;
  if (joint_proto.has_parameters()) {
    INTR_ASSIGN_OR_RETURN(parameters.type,
                          FromProto(joint_proto.parameters().type()));
    parameters.axis =
        intrinsic_proto::FromProto(joint_proto.parameters().axis());
    INTR_ASSIGN_OR_RETURN(parameters.system_limits,
                          FromProto(joint_proto.parameters().system_limits()));
    INTR_ASSIGN_OR_RETURN(parameters.soft_limits,
                          FromProto(joint_proto.parameters().soft_limits()));
    parameters.default_configuration =
        joint_proto.parameters().default_configuration();

    parameters.dynamics.static_friction =
        joint_proto.parameters().dynamics().static_friction();
    parameters.dynamics.damping = joint_proto.parameters().dynamics().damping();
  }

  std::optional<Joint::LinearDependency> linear_dependency;
  if (joint_proto.has_linear_dependency()) {
    linear_dependency = kinematics::FromProto(joint_proto.linear_dependency());
  }

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<Joint> joint,
                        Joint::Create(joint_proto.element().name(), parameters,
                                      linear_dependency));
  deferred_joint.deferred_element.inbound_element = joint.release();

  return deferred_joint;
}

absl::StatusOr<intrinsic_proto::Link> ToProto(
    const details::DeferredLink& link_element) {
  const auto* link =
      dynamic_cast<const Link*>(link_element.deferred_element.outbound_element);
  if (link == nullptr) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Element ", link_element.deferred_element.outbound_element->GetName(),
        " is not a link"));
  }

  intrinsic_proto::Link out_proto;
  *out_proto.mutable_element() = ToProto(link_element.deferred_element);

  auto* parameters = out_proto.mutable_parameters();
  parameters->set_mass(link->GetParameters().mass);
  *parameters->mutable_center_of_gravity() =
      ToProto(link->GetParameters().center_of_gravity);
  if (!link->GetParameters().inertia.isApprox(eigenmath::Matrix3d::Zero())) {
    *parameters->mutable_inertia() =
        ::intrinsic::ToProto(link->GetParameters().inertia);
  }

  return out_proto;
}

absl::StatusOr<details::DeferredLink> FromProto(
    const intrinsic_proto::Link& link_proto) {
  details::DeferredLink deferred_link;
  INTR_ASSIGN_OR_RETURN(deferred_link.deferred_element,
                        details::FromProto(link_proto.element()));

  Link::Parameters parameters;
  if (link_proto.has_parameters()) {
    parameters.mass = link_proto.parameters().mass();
    if (link_proto.parameters().has_center_of_gravity()) {
      INTR_ASSIGN_OR_RETURN(
          parameters.center_of_gravity,
          FromProto(link_proto.parameters().center_of_gravity()));
    } else {
      parameters.center_of_gravity = Pose3d::Identity();
    }
    if (link_proto.parameters().has_inertia()) {
      INTR_ASSIGN_OR_RETURN(parameters.inertia,
                            FromProto(link_proto.parameters().inertia()));
    }
  }

  deferred_link.deferred_element.inbound_element =
      new Link(link_proto.element().name(), parameters);

  return deferred_link;
}

}  // namespace details

absl::StatusOr<intrinsic_proto::Skeleton> ToProto(const Skeleton& skeleton) {
  intrinsic_proto::Skeleton out_proto;

  out_proto.set_name(skeleton.GetName());

  *out_proto.mutable_anchor_offset() = ToProto(skeleton.GetAnchorPointOffset());

  auto all_element_ids = skeleton.GetAllElementIds();

  absl::flat_hash_map<ElementId, details::DeferredElement> deferred_elements;
  for (const auto id : all_element_ids) {
    const auto* element = skeleton.GetElement(id).value();
    QCHECK(element != nullptr);

    details::DeferredElement& deferred_element = deferred_elements[id];
    deferred_element.outbound_element = element;
    deferred_element.id = id;

    auto parent_element = element->GetParentElement();
    ElementId parent_id = kInvalidElementId;
    if (parent_element != std::nullopt) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          parent_id, skeleton.GetElementId(parent_element.value()),
          "While getting parent");
    }
    deferred_element.parent_id = parent_id;

    for (const auto* child : element->GetChildElements()) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(ElementId child_id,
                                    skeleton.GetElementId(child),
                                    "While getting child");
      deferred_element.children_ids.insert(child_id);
    }
  }

  auto all_joint_ids = skeleton.GetAllJointIds();
  for (const auto id : all_joint_ids) {
    details::DeferredJoint deferred_joint;
    deferred_joint.deferred_element = deferred_elements[id];
    INTR_ASSIGN_OR_RETURN(*out_proto.add_joints(), ToProto(deferred_joint));

    auto id_or_status = skeleton.GetDofIndexForElementId(id);
    if (id_or_status.ok()) {
      (*out_proto.mutable_element_id_to_dof_index())[id.value()] =
          id_or_status.value();
    }
  }

  for (const auto id : skeleton.GetAllLinkIds()) {
    INTR_ASSIGN_OR_RETURN(
        *out_proto.add_links(),
        ToProto(details::DeferredLink({deferred_elements[id]})));
  }

  for (const auto id : skeleton.GetAllCoordinateFrameIds()) {
    *out_proto.add_coordinate_frames()->mutable_element() =
        ToProto(deferred_elements[id]);
  }

  return out_proto;
}

absl::StatusOr<std::unique_ptr<Skeleton>> FromProto(
    const intrinsic_proto::Skeleton& skeleton_proto) {
  Pose3d offset;
  if (skeleton_proto.has_anchor_offset()) {
    INTR_ASSIGN_OR_RETURN(offset, FromProto(skeleton_proto.anchor_offset()));
  }

  auto skeleton = std::make_unique<Skeleton>(skeleton_proto.name(), offset);

  absl::flat_hash_map<ElementId, details::DeferredElement>
      all_deferred_elements;

  std::vector<details::DeferredLink> deferred_links;
  for (const auto& link : skeleton_proto.links()) {
    INTR_ASSIGN_OR_RETURN(auto deferred_link, details::FromProto(link));
    deferred_links.push_back(deferred_link);
    auto last_link = deferred_links.back();

    all_deferred_elements[last_link.deferred_element.id] =
        last_link.deferred_element;
  }

  std::list<details::DeferredJoint> deferred_joints;
  for (const auto& joint : skeleton_proto.joints()) {
    INTR_ASSIGN_OR_RETURN(auto deferred_joint, details::FromProto(joint));

    deferred_joints.push_back(deferred_joint);
    auto last_joint = deferred_joints.back();

    all_deferred_elements[last_joint.deferred_element.id] =
        last_joint.deferred_element;
  }

  std::list<details::DeferredElement> deferred_coordinate_frames;
  for (const auto& coordinate_frame : skeleton_proto.coordinate_frames()) {
    INTR_ASSIGN_OR_RETURN(auto deferred_cf,
                          details::FromProto(coordinate_frame.element()));

    auto* cf = new CoordinateFrame(coordinate_frame.element().name());
    deferred_cf.inbound_element = cf;

    deferred_coordinate_frames.push_back(deferred_cf);
    auto last_cf = deferred_coordinate_frames.back();

    all_deferred_elements[last_cf.id] = last_cf;
  }

  // Find the root
  details::DeferredElement* root = nullptr;
  for (auto& [id, element] : all_deferred_elements) {
    if (element.inbound_element == nullptr) {
      continue;
    }
    if (element.parent_id == kInvalidElementId) {
      root = &element;
      break;
    }
  }
  if (root == nullptr) {
    return icon::InvalidArgumentError("Couldn't find a root element");
  }

  std::list<ElementId> element_id_stack;
  element_id_stack.push_back(root->id);
  while (!element_id_stack.empty()) {
    // We keep the stack sorted so that we create element in increasing id
    // order
    element_id_stack.sort();

    ElementId current_id = element_id_stack.front();
    element_id_stack.pop_front();

    const auto& current_element = all_deferred_elements[current_id];
    for (const auto& child_id : current_element.children_ids) {
      element_id_stack.push_back(child_id);
    }

    auto link_it =
        std::find_if(deferred_links.begin(), deferred_links.end(),
                     [&current_id](const auto& deferred_link) {
                       return deferred_link.deferred_element.id == current_id;
                     });
    if (link_it != deferred_links.end()) {
      std::unique_ptr<Link> link;
      auto* link_ptr =
          dynamic_cast<Link*>(link_it->deferred_element.inbound_element);
      QCHECK(link_ptr != nullptr);
      link.reset(link_ptr);
      INTR_RETURN_IF_ERROR(skeleton->AddLinkWithId(
          std::move(link), current_id, link_it->deferred_element.parent_id,
          link_it->deferred_element.parent_t_this));
      continue;
    }

    auto joint_it =
        std::find_if(deferred_joints.begin(), deferred_joints.end(),
                     [&current_id](const auto& deferred_joint) {
                       return deferred_joint.deferred_element.id == current_id;
                     });
    if (joint_it != deferred_joints.end()) {
      std::unique_ptr<Joint> joint;
      auto* joint_ptr =
          dynamic_cast<Joint*>(joint_it->deferred_element.inbound_element);
      QCHECK(joint_ptr != nullptr);
      joint.reset(joint_ptr);

      INTR_RETURN_IF_ERROR(skeleton->AddJointWithId(
          std::move(joint), /*new_element_id=*/current_id,
          joint_it->deferred_element.parent_id,
          joint_it->deferred_element.parent_t_this));
      INTR_RETURN_IF_ERROR(skeleton->ValidateLinearDependency(current_id));
      continue;
    }

    auto cf_it = std::find_if(
        deferred_coordinate_frames.begin(), deferred_coordinate_frames.end(),
        [&current_id](const auto& deferred_coordinate_frame) {
          return deferred_coordinate_frame.id == current_id;
        });
    if (cf_it != deferred_coordinate_frames.end()) {
      std::unique_ptr<CoordinateFrame> cf;
      auto* cf_ptr = dynamic_cast<CoordinateFrame*>(cf_it->inbound_element);
      QCHECK(cf_ptr != nullptr);
      cf.reset(cf_ptr);
      INTR_RETURN_IF_ERROR(skeleton->AddCoordinateFrameWithId(
          std::move(cf), current_id, cf_it->parent_id, cf_it->parent_t_this));
      continue;
    }

    return absl::InvalidArgumentError(absl::StrCat(
        "Element id=", current_id.value(), " is not a known element type"));
  }

  return skeleton;
}

intrinsic_proto::Joint::LinearDependency ToProto(
    const Joint::LinearDependency& linear_dependency) {
  intrinsic_proto::Joint::LinearDependency linear_dependency_proto;
  linear_dependency_proto.set_alpha_self(linear_dependency.alpha_self);
  for (const auto& [joint_id, alpha] : linear_dependency.alpha_leading) {
    linear_dependency_proto.mutable_alpha_leading()->insert(
        {joint_id.value(), alpha});
  }
  return linear_dependency_proto;
}

Joint::LinearDependency FromProto(
    const intrinsic_proto::Joint::LinearDependency& linear_dependency_proto) {
  Joint::LinearDependency linear_dependency;
  linear_dependency.alpha_self = linear_dependency_proto.alpha_self();
  for (const auto& [joint_id, alpha] :
       linear_dependency_proto.alpha_leading()) {
    linear_dependency.alpha_leading.insert({ElementId(joint_id), alpha});
  }
  return linear_dependency;
}

}  // namespace kinematics
}  // namespace intrinsic
